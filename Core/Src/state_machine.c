/***************************************************************************
 * state_machine.c
 * created by Sebastian Forenza 2026
 *
 * Central control loop. Drives the system through:
 *
 *   DISCONNECTED_IDLE → (BLE connect) → CONNECTED_IDLE
 *   CONNECTED_IDLE    → (armed)       → STABILIZING
 *   STABILIZING       → (3 s still)   → LOCKED
 *   STABILIZING       → (15 s elapsed) → CONNECTED_IDLE
 *   LOCKED            → (motion)      → ALARM_ACTIVE
 *   ALARM_ACTIVE      → (melody done + no motion) → LOCKED
 *
 * Cable plug / unplug events are handled in this file too:
 *   PB4 (BQ251_PG, falling-edge EXTI) sets stayAwakeFlag. On unplug,
 *   stayAwakeFlag is held for CABLE_UNPLUG_AWAKE_MS before the device
 *   is allowed back to deep sleep.
 ***************************************************************************/

#include "state_machine.h"
#include "lights.h"
#include "sound.h"
#include "battery.h"
#include "lockservice_app.h"
#include "loyalty.h"
#include "accelerometer.h"
#include "lis2dux12_app.h"
#include "motion_logger.h"
#include "power_management.h"
#include "app_ble.h"
#include "app_common.h"

#define M24CXX_MODEL 0
#include "m24cxx.h"

extern I2C_HandleTypeDef hi2c1;

volatile SystemState_t currentState = STATE_CONNECTED_IDLE;
volatile SystemState_t previousState = STATE_CONNECTED_IDLE;
volatile uint8_t deviceState = 0;
volatile uint8_t deviceInfo = 0;
volatile uint8_t deviceBattery = 100;
volatile uint8_t connectionStatus = 0;

static uint32_t stateEntryTime = 0;

volatile uint8_t stayAwakeFlag = 0;

// Tick at which the post-connect motion-grace window expires. While the
// current tick is below this, LOCKED suppresses motion-triggered alarm
// transitions so the UCF reload from RestoreAll doesn't fire the alarm
// the moment the user picks the device up to pair it.
static volatile uint32_t motion_grace_until = 0;

void StateMachine_StartMotionGrace(uint32_t ms)
{
    uint32_t until = HAL_GetTick() + ms;
    if ((int32_t)(until - motion_grace_until) > 0) {
        motion_grace_until = until;
    }
}

static uint8_t MotionGrace_Active(void)
{
    return (int32_t)(motion_grace_until - HAL_GetTick()) > 0;
}

static uint32_t cableUnplugTime = 0;
static uint8_t  cableWasPlugged = 0;

static volatile uint8_t findMyActive = 0;

/***************************************************************************
 * LED_ChargingPulse — red→yellow→green pulse based on cached SOC
 *   0 % = pure red, 50 % = yellow, 100 % = pure green.
 ***************************************************************************/
static void LED_ChargingPulse(void)
{
    uint16_t soc = BATTERY_GetSOC();
    if (soc > 100) soc = 100;

    uint8_t r, g;
    if (soc <= 50) {
        r = 255;
        g = (uint8_t)((soc * 255) / 50);
    } else {
        r = (uint8_t)(((100 - soc) * 255) / 50);
        g = 255;
    }
    LED_Pulse(4000, r, g, 0, 255);
}

/***************************************************************************
 * CablePlug_IRQCallback — called from GPIOB IRQ when PB4 falls (cable in)
 *   Safe from interrupt context. Sets stayAwakeFlag so the main loop can
 *   restore peripherals and show charging status.
 ***************************************************************************/
void CablePlug_IRQCallback(void)
{
    stayAwakeFlag = 1;
}

/***************************************************************************
 * StateMachine_Init — set defaults: LOW sens, CALM alarm, lights+logging on
 ***************************************************************************/
void StateMachine_Init(void)
{
    currentState = STATE_DISCONNECTED_IDLE;
    stateEntryTime = HAL_GetTick();

    deviceState = 0;
    SET_ARMED_BIT(deviceState, 0);
    SET_ALARM_TYPE(deviceState, ALARM_CALM);
    SET_SENSITIVITY(deviceState, SENSITIVITY_LOW);
    SET_LIGHTS_BIT(deviceState, 1);
    SET_LOGGING_BIT(deviceState, 1);
    SET_SILENCE_BIT(deviceState, 0);

    deviceInfo = 0;

    cableWasPlugged = IS_CABLE_PLUGGED() ? 1 : 0;
    cableUnplugTime = 0;
}

/***************************************************************************
 * CablePlug_UpdateState — edge-detect cable + manage post-unplug awake window
 *   Drives the loyalty reset window on debounced VBUS edges: rising edge
 *   opens it (CLAIM may overwrite the EEPROM token for ~10 s), falling edge
 *   closes it. The 50 ms debounce prevents an insertion bounce from racking
 *   up multiple Start/Cancel cycles. Boot-with-VBUS-already-high is handled
 *   inside Loyalty_Init, not here, since this function never sees that edge.
 ***************************************************************************/
#define CABLE_EDGE_DEBOUNCE_MS  50u

// Cap on how long STABILIZING will pulse blue waiting for stillness before
// bailing back to CONNECTED_IDLE. Without this the device sits forever if
// motion never settles; the fall-back un-arms via the ARMED-bit clear in
// StateMachine_ChangeState.
#define STABILIZE_TIMEOUT_MS    15000u

static void CablePlug_UpdateState(void)
{
    static uint32_t last_edge_ms = 0;
    uint8_t pluggedNow = IS_CABLE_PLUGGED() ? 1 : 0;

    if (pluggedNow) {
        if (!cableWasPlugged) {
            uint32_t now = HAL_GetTick();
            if ((now - last_edge_ms) >= CABLE_EDGE_DEBOUNCE_MS) {
                Loyalty_StartResetWindow();
                last_edge_ms = now;
            }
            LED_PlugIn_Start();
        }
        stayAwakeFlag = 1;
        cableUnplugTime = 0;
        cableWasPlugged = 1;
        return;
    }

    if (cableWasPlugged) {
        uint32_t now = HAL_GetTick();
        if ((now - last_edge_ms) >= CABLE_EDGE_DEBOUNCE_MS) {
            Loyalty_CancelResetWindow();
            last_edge_ms = now;
        }
        cableUnplugTime = now;
        cableWasPlugged = 0;
        LED_PlugOut_Start();
    }

    if (cableUnplugTime != 0) {
        if ((HAL_GetTick() - cableUnplugTime) < CABLE_UNPLUG_AWAKE_MS) {
            stayAwakeFlag = 1;
        } else {
            // Window expired. Don't touch stayAwakeFlag — the state loop
            // will re-decide based on the current state.
            cableUnplugTime = 0;
        }
    }
}

void State_Disconnected_Idle_Loop(void)
{
    if (IS_CABLE_PLUGGED()) {
        if (PowerMgmt_IsLowPower()) {
            PowerMgmt_RestoreAll();
        }
        stayAwakeFlag = 1;

        if (LED_PlugIn_InProgress()) {
            LED_PlugIn_Tick();
        } else if (IS_CHARGING_NOW() && !BATTERY_IsFullCached()) {
            LED_ChargingPulse();
        } else if (BATTERY_IsFullCached()) {
            LED_Solid(0, 255, 0, 255);
        } else {
            LED_Off();
        }
    } else {
        if (cableUnplugTime == 0) {
            stayAwakeFlag = 0;
        }

        if (LED_PlugOut_InProgress()) {
            LED_PlugOut_Tick();
        } else {
            LED_Off();
        }

        if (!stayAwakeFlag && !PowerMgmt_IsLowPower()) {
            PowerMgmt_EnterLowPower_Idle();
        }
    }

    if (connectionStatus) {
        if (PowerMgmt_IsLowPower()) {
            PowerMgmt_RestoreAll();
        }
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
    }
}

void State_Connected_Idle_Loop(void)
{
    stayAwakeFlag = 1;

    if (!findMyActive) {
        if (IS_CABLE_PLUGGED()) {
            if (LED_PlugIn_InProgress()) {
                LED_PlugIn_Tick();
            } else if (IS_CHARGING_NOW() && !BATTERY_IsFullCached()) {
                LED_ChargingPulse();
            } else if (BATTERY_IsFullCached()) {
                LED_Solid(0, 255, 0, 255);
            } else {
                LED_Off();
            }
        } else if (LED_PlugOut_InProgress()) {
            LED_PlugOut_Tick();
        } else if (GET_LIGHTS_BIT(deviceState)) {
            LED_Rainbow(10, LedBrightness_Get());
        } else {
            LED_Off();
        }
    }

    if (!connectionStatus) {
        StateMachine_ChangeState(STATE_DISCONNECTED_IDLE);
    }

    if (GET_ARMED_BIT(deviceState)) {
        LIS2DUX12_ClearMotion();
        StateMachine_ChangeState(STATE_STABILIZING);
    }
}

/***************************************************************************
 * State_Stabilizing_Loop — wait for 3 s of stillness before locking
 *   Pulsing blue LED while waiting; both the MLC interrupt and a 10 Hz
 *   poll reset the still-timer when motion is detected. Bails to
 *   CONNECTED_IDLE after STABILIZE_TIMEOUT_MS so a never-settling device
 *   can't pulse blue forever.
 ***************************************************************************/
void State_Stabilizing_Loop(void)
{
    static uint32_t last_still_time = 0;
    static uint32_t stabilize_entry_time = 0;
    static uint8_t  stabilize_started = 0;

    // STABILIZING is only reachable from CONNECTED_IDLE, which already sets
    // this every iteration — but re-asserting here keeps the invariant
    // "stayAwakeFlag is 1 whenever currentState implies a live BLE session"
    // locally provable. See the comment in State_Locked_Loop's connected
    // branch for the chip-level DEEPSTOP-between-events failure mode this
    // flag exists to suppress.
    stayAwakeFlag = 1;

    if (!GET_ARMED_BIT(deviceState)) {
        stabilize_started = 0;
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        LED_Off();
        return;
    }

    if (!findMyActive) {
        if (GET_LIGHTS_BIT(deviceState)) {
            LED_Pulse(1000, 0, 0, 255, LedBrightness_Get());
        } else {
            LED_Off();
        }
    }

    if (!stabilize_started) {
        last_still_time = HAL_GetTick();
        stabilize_entry_time = HAL_GetTick();
        stabilize_started = 1;
        LIS2DUX12_ClearMotion();
    }

    if (LIS2DUX12_IsMotionDetected()) {
        uint8_t mlc_out;
        lis2dux12_app_get_mlc_output(&mlc_out);
        if (mlc_out == MLC_STATE_IN_MOTION || mlc_out == MLC_STATE_SHAKEN) {
            last_still_time = HAL_GetTick();
        }
    }

    static uint32_t last_poll = 0;
    if (HAL_GetTick() - last_poll >= 100) {
        last_poll = HAL_GetTick();
        uint8_t mlc_out;
        if (lis2dux12_app_get_mlc_output(&mlc_out) == 0) {
            if (mlc_out == MLC_STATE_IN_MOTION || mlc_out == MLC_STATE_SHAKEN) {
                last_still_time = HAL_GetTick();
            }
        }
    }

    if (HAL_GetTick() - last_still_time >= 3000) {
        stabilize_started = 0;
        StateMachine_ChangeState(STATE_LOCKED);
        return;
    }

    if (HAL_GetTick() - stabilize_entry_time >= STABILIZE_TIMEOUT_MS) {
        stabilize_started = 0;
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        return;
    }
}

/***************************************************************************
 * State_Locked_Loop — main armed loop, including LP wake-from-motion path
 *
 * Wake-from-LP behaviour (HIGH-sens path keeps MLC alive across sleep):
 *   - INT alone never fires the alarm.
 *   - Read MLC + FSM first; if MLC says IN_MOTION/SHAKEN seed the deferred
 *     alert and transition to ALARM_ACTIVE.
 *   - If MLC still says STATIONARY (a "breeze" wake), motion_assessing
 *     keeps peripherals up for ~10 s while the regular MLC/FSM loop
 *     watches for an escalation.
 *
 * Deferred alert: when MLC says IN_MOTION/SHAKEN we trigger the alarm
 * immediately but DEFER the log/alert until MLC settles, so the door
 * detector can override IN_MOTION with DOOR_OPENED if rotation crossed
 * the threshold. Impact and freefall (FSM) always fire immediately.
 ***************************************************************************/
void State_Locked_Loop(void)
{
    static uint8_t  motion_assessing = 0;
    static uint32_t motion_assess_start = 0;
    #define MOTION_ASSESS_TIMEOUT_MS 10000

    static uint8_t      motion_pending = 0;
    static MotionType_t pending_type   = MOTION_TYPE_NONE;
    static uint32_t     motion_pending_tick = 0;

    if (!GET_ARMED_BIT(deviceState)) {
        motion_assessing = 0;
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        LED_Off();
        return;
    }

    if (PowerMgmt_IsLowPower() && LIS2DUX12_PeekMotionStatus()) {
        PowerMgmt_RestoreForMotion();
        LIS2DUX12_ClearMotion();
        stayAwakeFlag = 1;
        motion_assessing = 1;
        motion_assess_start = HAL_GetTick();

        // ONE log per wake. The prior firmware logged MOTION_TYPE_IN_MOTION
        // unconditionally here ("up-front so brief motions still recorded"),
        // then logged a SECOND event below for the FSM impact/freefall
        // branch, plus eventually a third via motion_pending → settle. That
        // tripled every wake's footprint in the ring. Track whether the
        // classifier produced a qualifying log; only fall back to a generic
        // IN_MOTION at the end if it did not.
        uint8_t wake_handled = 0;

        // Significant-motion fast-path (MEDIUM + HIGH only): if |a| deviates
        // from 1 g by more than 250 mg, fire the alarm without waiting for
        // MLC. Compared in mg² to avoid a sqrt.
        uint8_t fast_fired = 0;
        if (GET_SENSITIVITY(deviceState) != SENSITIVITY_LOW) {
            int16_t ax_mg = 0, ay_mg = 0, az_mg = 0;
            if (lis2dux12_app_read_accel_mg(&ax_mg, &ay_mg, &az_mg) == 0) {
                int32_t mag2 = (int32_t)ax_mg * ax_mg
                             + (int32_t)ay_mg * ay_mg
                             + (int32_t)az_mg * az_mg;
                const int32_t hi2 = 1250L * 1250L;   // 1 g + 250 mg
                const int32_t lo2 =  750L *  750L;   // 1 g - 250 mg
                if (mag2 > hi2 || mag2 < lo2) {
                    if (!motion_pending) {
                        motion_pending      = 1;
                        pending_type        = MOTION_TYPE_IN_MOTION;
                        motion_pending_tick = HAL_GetTick();
                    }
                    motion_assessing = 0;
                    if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                        StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                    }
                    fast_fired = 1;
                    // The pending_type path will log on settle (or via the
                    // 3-s safety timeout). Don't fall back to a generic log.
                    wake_handled = 1;
                }
            }
        }

        uint8_t mlc_out;
        if (!fast_fired && lis2dux12_app_get_mlc_output(&mlc_out) == 0) {
            lis2dux12_app_update_cached_state(mlc_out);

            uint8_t impact = 0, freefall = 0;
            lis2dux12_app_check_fsm_events(&impact, &freefall);
            if (impact || freefall) {
                MotionType_t mt = impact ? MOTION_TYPE_IMPACT : MOTION_TYPE_FREEFALL;
                motion_assessing = 0;
                if (GET_LOGGING_BIT(deviceState)) {
                    MotionLogger_LogEvent(mt);
                    LOCKSERVICE_SendMotionAlert(mt);
                }
                if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                    StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                }
                wake_handled = 1;
            } else if (mlc_out == MLC_STATE_IN_MOTION ||
                       mlc_out == MLC_STATE_SHAKEN) {
                if (!motion_pending) {
                    motion_pending      = 1;
                    pending_type = (mlc_out == MLC_STATE_SHAKEN)
                        ? MOTION_TYPE_SHAKEN : MOTION_TYPE_IN_MOTION;
                    motion_pending_tick = HAL_GetTick();
                }
                motion_assessing = 0;
                if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                    StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                }
                wake_handled = 1;
            }
        }

        // Fallback: INT latched but neither FSM nor MLC qualified. The wake
        // itself is evidence of motion, so log a generic IN_MOTION so brief
        // blips don't slip through. No alarm transition without a qualifying
        // classification — motion_assessing's timeout governs the decision.
        if (!wake_handled && GET_LOGGING_BIT(deviceState)) {
            MotionLogger_LogEvent(MOTION_TYPE_IN_MOTION);
        }
    }

    if (!findMyActive) {
        if (GET_LIGHTS_BIT(deviceState) && !PowerMgmt_IsLowPower()) {
            LED_Armed(10, LedBrightness_Get());
        } else {
            LED_Off();
        }
    }

    if (!PowerMgmt_IsLowPower() && !MotionGrace_Active()) {

        if (LIS2DUX12_IsMotionDetected()) {
            uint8_t mlc_out;
            lis2dux12_app_get_mlc_output(&mlc_out);
            lis2dux12_app_update_cached_state(mlc_out);

            uint8_t impact, freefall;
            lis2dux12_app_check_fsm_events(&impact, &freefall);
            if (impact || freefall) {
                MotionType_t mt = impact ? MOTION_TYPE_IMPACT : MOTION_TYPE_FREEFALL;
                stayAwakeFlag = 1;
                motion_assessing = 0;
                if (GET_LOGGING_BIT(deviceState)) {
                    MotionLogger_LogEvent(mt);
                    LOCKSERVICE_SendMotionAlert(mt);
                }
                if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                    StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                }
            }

            if (mlc_out == MLC_STATE_IN_MOTION || mlc_out == MLC_STATE_SHAKEN) {
                if (!motion_pending) {
                    motion_pending = 1;
                    pending_type = (mlc_out == MLC_STATE_SHAKEN)
                        ? MOTION_TYPE_SHAKEN : MOTION_TYPE_IN_MOTION;
                    motion_pending_tick = HAL_GetTick();
                }
                stayAwakeFlag = 1;
                motion_assessing = 0;
                if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                    StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                }
            }

            if (mlc_out == MLC_STATE_STATIONARY_UPRIGHT ||
                mlc_out == MLC_STATE_STATIONARY_NOT_UPRIGHT) {
                if (motion_pending) {
                    if (GET_LOGGING_BIT(deviceState)) {
                        MotionLogger_LogEvent(pending_type);
                        LOCKSERVICE_SendMotionAlert(pending_type);
                    }
                    motion_pending = 0;
                }
            }
        }

        // 3 s safety timeout — flush a deferred alert that never settled.
        if (motion_pending && (HAL_GetTick() - motion_pending_tick > 3000)) {
            if (GET_LOGGING_BIT(deviceState)) {
                MotionLogger_LogEvent(pending_type);
                LOCKSERVICE_SendMotionAlert(pending_type);
            }
            motion_pending = 0;
        }

        static uint32_t last_poll = 0;
        if (HAL_GetTick() - last_poll >= 100) {
            last_poll = HAL_GetTick();

            // Poll the MLC class alongside FSM. The INT-driven path above
            // only sees MLC *transitions* (rising edge of INT1 on a class
            // change). If MLC transitioned to IN_MOTION/SHAKEN during a
            // window where the INT was consumed without acting on it — the
            // 2 s post-reconnect motion-grace drain is the canonical case:
            // motion that starts in-grace and persists past it leaves MLC
            // latched in IN_MOTION with no further edge to wake the active
            // branch — the alarm never fires until MLC settles back to
            // STATIONARY, by which time mlc_out no longer qualifies.
            // Polling here closes that gap and also covers any missed INT
            // (chip glitch, brief EXTI masking during a peripheral reinit).
            uint8_t mlc_out;
            if (lis2dux12_app_get_mlc_output(&mlc_out) == 0) {
                lis2dux12_app_update_cached_state(mlc_out);
                if (mlc_out == MLC_STATE_IN_MOTION ||
                    mlc_out == MLC_STATE_SHAKEN) {
                    if (!motion_pending) {
                        motion_pending = 1;
                        pending_type = (mlc_out == MLC_STATE_SHAKEN)
                            ? MOTION_TYPE_SHAKEN : MOTION_TYPE_IN_MOTION;
                        motion_pending_tick = HAL_GetTick();
                    }
                    stayAwakeFlag = 1;
                    motion_assessing = 0;
                    if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                        StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                    }
                }
            }

            uint8_t impact, freefall;
            lis2dux12_app_check_fsm_events(&impact, &freefall);
            if (impact || freefall) {
                MotionType_t mt = impact ? MOTION_TYPE_IMPACT : MOTION_TYPE_FREEFALL;
                stayAwakeFlag = 1;
                motion_assessing = 0;
                if (GET_LOGGING_BIT(deviceState)) {
                    MotionLogger_LogEvent(mt);
                    LOCKSERVICE_SendMotionAlert(mt);
                }
                if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                    StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                }
            }
        }

    } else if (!PowerMgmt_IsLowPower() && MotionGrace_Active()) {
        // During the post-connect grace window, drain any motion the UCF
        // reload + user handling produced so it doesn't latch and fire the
        // alarm the instant grace ends.
        if (LIS2DUX12_IsMotionDetected()) {
            uint8_t mlc_out;
            lis2dux12_app_get_mlc_output(&mlc_out);
            lis2dux12_app_update_cached_state(mlc_out);
        }
        uint8_t impact = 0, freefall = 0;
        lis2dux12_app_check_fsm_events(&impact, &freefall);
        (void)impact; (void)freefall;
        motion_pending = 0;
    }

    if (!connectionStatus) {
        LED_Off();

        if (motion_assessing) {
            if (HAL_GetTick() - motion_assess_start >= MOTION_ASSESS_TIMEOUT_MS) {
                motion_assessing = 0;
                stayAwakeFlag = 0;
            }
        } else {
            if (cableUnplugTime == 0 && !IS_CABLE_PLUGGED()) {
                stayAwakeFlag = 0;
            }
        }

        if (!stayAwakeFlag && !PowerMgmt_IsLowPower()) {
            motion_pending = 0;
            PowerMgmt_EnterLowPower_Armed();
        }
    } else {
        if (PowerMgmt_IsLowPower()) {
            PowerMgmt_RestoreAll();
        }
        motion_assessing = 0;

        // stayAwakeFlag gates the BLE stack's chip-level LPM in
        // app_entry.c::App_PowerSaveLevel_Check. With it clear AND a live
        // connection, UTIL_SEQ_Idle picks POWER_SAVE_LEVEL_STOP_LS_CLOCK_ON
        // and the chip enters DEEPSTOP between BLE connection events.
        // That's catastrophic in connected-and-locked: the main loop only
        // ticks at the connection interval (iOS often picks ~1 s on a
        // known-bonded reconnect), TIM2/TIM16 state isn't preserved
        // across DEEPSTOP so the LED pulse flails, and motion polling
        // gets crushed to the same ~1 Hz cadence — alarms miss, MLC
        // logs never get pushed.
        //
        // State_Connected_Idle_Loop and State_Alarm_Active_Loop both
        // set this as their first line; State_Locked_Loop was the gap.
        // The disconnect branch above intentionally clears the flag to
        // allow LP-armed entry, so on reconnect-while-locked we land
        // here with stayAwakeFlag=0 and need to re-assert it.
        stayAwakeFlag = 1;
    }
}

/***************************************************************************
 * State_Alarm_Active_Loop — drive alarm sound/lights, exit when motion stops
 *   Sounds the looping alarm pattern until alarm_duration_seconds elapses
 *   with no motion. Every qualifying motion event (MLC IN_MOTION/SHAKEN,
 *   FSM impact/freefall) HARD-RESETS the countdown to the full duration —
 *   the timer never extends, never partially drains. Duration 0 means stop
 *   the instant motion stops; a fresh motion event re-triggers from LOCKED.
 *   MLC INTs only fire on state changes, so MLC+FSM are also polled at 2 Hz.
 ***************************************************************************/
void State_Alarm_Active_Loop(void)
{
    stayAwakeFlag = 1;

    if (PowerMgmt_IsLowPower()) {
        PowerMgmt_RestoreAll();
    }

    static uint32_t last_motion_time = 0;
    static uint8_t  alarm_started    = 0;
    uint8_t motion_this_iter = 0;

    if (!GET_ARMED_BIT(deviceState)) {
        BUZZER_Stop();
        alarm_started = 0;
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        return;
    }

    // Read once per iteration so a mid-alarm setting change from iOS takes
    // effect immediately and the log line never lags the actual countdown.
    uint8_t  alarm_duration_s  = AlarmDuration_Get();
    uint32_t alarm_duration_ms = (uint32_t)alarm_duration_s * 1000u;

    if (!alarm_started) {
        uint8_t alarmType  = GET_ALARM_TYPE(deviceState);
        uint8_t showLights = GET_LIGHTS_BIT(deviceState);
        uint8_t led_b      = LedBrightness_Get();
        switch (alarmType) {
            case ALARM_NONE:
                break;
            case ALARM_CALM:
                if (showLights) LED_Alarm(300, 255, 0, 0, led_b);
                BUZZER_StartCalmAlarm();
                break;
            case ALARM_NORMAL:
                if (showLights) LED_Alarm(300, 255, 0, 0, led_b);
                BUZZER_StartNormalAlarm();
                break;
            case ALARM_LOUD:
                if (showLights) LED_Alarm(125, 255, 225, 0, led_b);
                BUZZER_StartSuperLoudAlarm();
                break;
            default:
                break;
        }

        alarm_started     = 1;
        last_motion_time  = HAL_GetTick();
        // Treat the entry tick as "fresh motion" so a duration-of-0 alarm
        // doesn't immediately satisfy the exit check on its first iteration.
        motion_this_iter  = 1;
    }

    if (LIS2DUX12_IsMotionDetected()) {
        uint8_t mlc_out;
        lis2dux12_app_get_mlc_output(&mlc_out);
        lis2dux12_app_update_cached_state(mlc_out);

        MotionType_t motionType = MOTION_TYPE_IN_MOTION;
        if (mlc_out == MLC_STATE_SHAKEN) motionType = MOTION_TYPE_SHAKEN;

        uint8_t impact, freefall;
        lis2dux12_app_check_fsm_events(&impact, &freefall);
        if (impact)   motionType = MOTION_TYPE_IMPACT;
        if (freefall) motionType = MOTION_TYPE_FREEFALL;

        uint8_t qualifying = (mlc_out == MLC_STATE_IN_MOTION || mlc_out == MLC_STATE_SHAKEN
                              || impact || freefall);
        if (qualifying) {
            last_motion_time = HAL_GetTick();
            motion_this_iter = 1;

            // Gate the log+alert on a real classification. Previously the
            // log was outside this if-block and a stationary→stationary INT
            // (MLC still STATIONARY, no FSM event) would land as a spurious
            // MOTION_TYPE_IN_MOTION entry — visible to the user as junk
            // events during long alarms.
            if (GET_LOGGING_BIT(deviceState)) {
                MotionLogger_LogEvent(motionType);
                LOCKSERVICE_SendMotionAlert(motionType);
            }
        }
    }

    static uint32_t last_mlc_poll = 0;
    if (HAL_GetTick() - last_mlc_poll > 500) {
        last_mlc_poll = HAL_GetTick();

        uint8_t mlc_out;
        if (lis2dux12_app_get_mlc_output(&mlc_out) == 0) {
            lis2dux12_app_update_cached_state(mlc_out);
            if (mlc_out == MLC_STATE_IN_MOTION || mlc_out == MLC_STATE_SHAKEN) {
                last_motion_time = HAL_GetTick();
                motion_this_iter = 1;
            }
        }

        uint8_t impact, freefall;
        lis2dux12_app_check_fsm_events(&impact, &freefall);
        if (impact || freefall) {
            last_motion_time = HAL_GetTick();
            motion_this_iter = 1;
            if (GET_LOGGING_BIT(deviceState)) {
                MotionType_t mt = impact ? MOTION_TYPE_IMPACT : MOTION_TYPE_FREEFALL;
                MotionLogger_LogEvent(mt);
                LOCKSERVICE_SendMotionAlert(mt);
            }
        }
    }

    // Final gate: the cached MLC state. INT-driven and 500 ms poll updates
    // can leave gaps where motion is physically continuous but no fresh
    // event was seen this iteration. Without this check, duration = 0 +
    // sustained motion would chatter between LOCKED and ALARM_ACTIVE every
    // few ms. Cached states 2/3 = IN_MOTION/SHAKEN.
    uint8_t cached_mlc = lis2dux12_app_get_cached_mlc_state();
    uint8_t still_in_motion = (cached_mlc == 2 || cached_mlc == 3);

    if (!motion_this_iter && !still_in_motion
        && (HAL_GetTick() - last_motion_time) >= alarm_duration_ms) {
        BUZZER_Stop();
        alarm_started = 0;
        StateMachine_ChangeState(STATE_LOCKED);
        return;
    }
}

/***************************************************************************
 * StateMachine_ChangeState — set ARMED bit, push BLE notify, record entry
 *   Status byte 6 carries CACHED_STATE_STABILIZING (0xFE) while the device
 *   is settling so the iOS app can show "Stabilizing…". Centrally enforces
 *   the alarmDisabled gate: any incoming STATE_ALARM_ACTIVE is dropped when
 *   the persisted flag is set, so every motion-trigger site is covered
 *   without per-site changes.
 ***************************************************************************/
void StateMachine_ChangeState(SystemState_t newState)
{
    if (newState == STATE_ALARM_ACTIVE && AlarmDisabled_Get()) {
        // Suppress at the trigger. Motion logging + BLE motion alerts have
        // already run in the caller; only the alarm path itself is gated.
        return;
    }

    if (newState != currentState) {
        previousState = currentState;
        currentState = newState;
        stateEntryTime = HAL_GetTick();

        if (newState == STATE_STABILIZING || newState == STATE_LOCKED || newState == STATE_ALARM_ACTIVE) {
            SET_ARMED_BIT(deviceState, 1);
        } else {
            SET_ARMED_BIT(deviceState, 0);
        }

        // EEPROM motion writes block the main loop ~15-25 ms (HAL_Delay +
        // i2c_wait on the M24C08 page commit), which freezes BUZZER_Update
        // and stretches whatever tone TIM16 happens to be playing. Defer
        // while ALARM_ACTIVE so the alarm pattern stays clean; flush after
        // BUZZER_Stop has already silenced the buzzer.
        if (newState == STATE_ALARM_ACTIVE) {
            MotionLogger_SetDeferEEPROM(1);
        } else if (previousState == STATE_ALARM_ACTIVE) {
            MotionLogger_SetDeferEEPROM(0);
            MotionLogger_FlushPending();
        }

        // Checkpoint the iOS-sync time anchor to EEPROM on LOCKED entry so
        // events logged on this boot still resolve to correct calendar times
        // if the device resets during the upcoming alarm window. No-op if
        // iOS hasn't synced the time yet.
        if (newState == STATE_LOCKED) {
            MotionLogger_PersistAnchor();
        }

        lis2dux12_app_set_stabilizing(newState == STATE_STABILIZING ? 1 : 0);

        // When ARMED clears, reset the cached MLC byte so the status notify
        // we're about to push doesn't carry a stale "Moving"/"Shaken" value
        // latched during LOCKED or ALARM_ACTIVE. The cache otherwise stays
        // pinned until the next class-change INT — which in MEDIUM/LOW
        // sensitivity may never come because LP wakeup wipes the MLC. Motion
        // classification is not meaningful while disarmed, so forcing to
        // STATIONARY is honest.
        if (newState != STATE_STABILIZING && newState != STATE_LOCKED && newState != STATE_ALARM_ACTIVE) {
            lis2dux12_app_update_cached_state(MLC_STATE_STATIONARY_UPRIGHT);
        }

        LOCKSERVICE_SendStatusUpdate();
    }
}

void ChargingCheck(void)
{
    static uint32_t last_check = 0;
    if ((HAL_GetTick() - last_check) < 1000) return;
    last_check = HAL_GetTick();

    if (IS_CABLE_PLUGGED()) {
        if (IS_CHARGING_NOW() && !BATTERY_IsFullCached()) {
            SET_BATTERY_CHARGING(deviceBattery);
        } else {
            CLEAR_BATTERY_CHARGING(deviceBattery);
        }
    }
}

/***************************************************************************
 * Find My Device — non-blocking ping with a green LED synced to each tone
 ***************************************************************************/

void FindMyDevice_Start(void)
{
    BUZZER_StartFindMe();
    findMyActive = 1;
}

void FindMyDevice_Update(void)
{
    if (!findMyActive) return;

    if (!BUZZER_IsPlaying()) {
        LED_Off();
        findMyActive = 0;
        return;
    }

    if (BUZZER_IsToneActive()) {
        LED_Solid(0, 255, 0, LedBrightness_Get());
    } else {
        LED_Off();
    }
}

void StateMachine_Run(void)
{
    CablePlug_UpdateState();

    ChargingCheck();
    BUZZER_Update();
    FindMyDevice_Update();

    switch (currentState) {
        case STATE_DISCONNECTED_IDLE:
            State_Disconnected_Idle_Loop();
            break;
        case STATE_CONNECTED_IDLE:
            State_Connected_Idle_Loop();
            break;
        case STATE_STABILIZING:
            State_Stabilizing_Loop();
            break;
        case STATE_LOCKED:
            State_Locked_Loop();
            break;
        case STATE_ALARM_ACTIVE:
            State_Alarm_Active_Loop();
            break;
        default:
            StateMachine_ChangeState(STATE_DISCONNECTED_IDLE);
            break;
    }
}

/***************************************************************************
 * Persisted device-state record — EEPROM-backed mirror of the user-facing
 * settings byte (alarm type, sensitivity, lights, logging, silence) plus
 * deviceInfo bit 0 (HIGH_PERF). The ARMED bit is intentionally NOT
 * persisted: boot always comes up disarmed so a power glitch can't leave a
 * stolen device armed without the owner re-arming it from the app.
 *
 * EEPROM record (3 bytes at 0x1E):
 *   [0] magic = 0xC6
 *   [1] deviceState (ARMED bit forced to 0 on save)
 *   [2] deviceInfo HIGH_PERF (bit 0 only; bit 1 alarmDisabled lives in its
 *       own record at 0x1C, owned by sound.c)
 ***************************************************************************/

#define EEPROM_DEVICE_SETTINGS_ADDR  0x1E
#define EEPROM_DEVICE_SETTINGS_LEN   3
#define EEPROM_DEVICE_SETTINGS_MAGIC 0xC6

static M24CXX_HandleTypeDef s_sm_eeprom;

void DeviceSettings_Init(void)
{
    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_sm_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        PowerMgmt_EEPROM_PowerOff();
        return;
    }

    uint8_t buf[EEPROM_DEVICE_SETTINGS_LEN] = {0};
    if (m24cxx_read(&s_sm_eeprom, EEPROM_DEVICE_SETTINGS_ADDR, buf,
                    EEPROM_DEVICE_SETTINGS_LEN) == M24CXX_Ok) {
        if (buf[0] == EEPROM_DEVICE_SETTINGS_MAGIC) {
            // Apply persisted bits, force ARMED clear.
            deviceState = buf[1];
            SET_ARMED_BIT(deviceState, 0);
            // Preserve any deviceInfo bits already loaded by other inits
            // (alarmDisabled doesn't touch the RAM byte at boot, so this
            // is mostly defensive for future bits).
            deviceInfo = (deviceInfo & ~0x01) | (buf[2] & 0x01);
        } else {
            // Blank EEPROM / wrong magic — seed from the StateMachine_Init
            // defaults that already populated deviceState/deviceInfo.
            uint8_t fresh[EEPROM_DEVICE_SETTINGS_LEN];
            fresh[0] = EEPROM_DEVICE_SETTINGS_MAGIC;
            fresh[1] = deviceState & ~0x01;   // ARMED off
            fresh[2] = deviceInfo  &  0x01;
            (void)m24cxx_write(&s_sm_eeprom, EEPROM_DEVICE_SETTINGS_ADDR,
                               fresh, EEPROM_DEVICE_SETTINGS_LEN);
        }
    }

    PowerMgmt_EEPROM_PowerOff();
}

/***************************************************************************
 * DeviceSettings_Persist — write the current deviceState/deviceInfo to
 *   EEPROM. Called after the iOS settings dispatcher applies a write.
 *   Skips the I2C transaction when the persisted-relevant bits are
 *   unchanged, to avoid wear on settings notifications that don't actually
 *   modify any user-facing flag.
 ***************************************************************************/
void DeviceSettings_Persist(void)
{
    static uint8_t cached_state = 0xFF;   // forces first write
    static uint8_t cached_info  = 0xFF;

    uint8_t to_save_state = deviceState & ~0x01;   // never persist ARMED
    uint8_t to_save_info  = deviceInfo  &  0x01;

    if (to_save_state == cached_state && to_save_info == cached_info) {
        return;
    }

    cached_state = to_save_state;
    cached_info  = to_save_info;

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_sm_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
        uint8_t buf[EEPROM_DEVICE_SETTINGS_LEN];
        buf[0] = EEPROM_DEVICE_SETTINGS_MAGIC;
        buf[1] = to_save_state;
        buf[2] = to_save_info;
        (void)m24cxx_write(&s_sm_eeprom, EEPROM_DEVICE_SETTINGS_ADDR, buf,
                           EEPROM_DEVICE_SETTINGS_LEN);
    }

    PowerMgmt_EEPROM_PowerOff();
}
