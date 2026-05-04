/***************************************************************************
 * state_machine.c
 * created by Sebastian Forenza 2026
 *
 * Central control loop. Drives the system through:
 *
 *   DISCONNECTED_IDLE → (BLE connect) → CONNECTED_IDLE
 *   CONNECTED_IDLE    → (armed)       → STABILIZING
 *   STABILIZING       → (3 s still)   → LOCKED
 *   LOCKED            → (motion)      → ALARM_ACTIVE
 *   ALARM_ACTIVE      → (melody done + no motion) → LOCKED
 *
 * Cable plug / unplug events are handled in this file too:
 *   PB4 (BQ251_PG, falling-edge EXTI) sets cablePlugFlag + stayAwakeFlag.
 *   On unplug, stayAwakeFlag is held for CABLE_UNPLUG_AWAKE_MS before the
 *   device is allowed back to deep sleep.
 ***************************************************************************/

#include "state_machine.h"
#include "lights.h"
#include "sound.h"
#include "battery.h"
#include "lockservice_app.h"
#include "accelerometer.h"
#include "lis2dux12_app.h"
#include "motion_logger.h"
#include "power_management.h"
#include "app_ble.h"
#include "alarm_duration.h"
#include "app_common.h"

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

volatile uint8_t cablePlugFlag = 0;
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
 *   Safe from interrupt context. Sets cablePlugFlag + stayAwakeFlag so the
 *   main loop can restore peripherals and show charging status.
 ***************************************************************************/
void CablePlug_IRQCallback(void)
{
    cablePlugFlag = 1;
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
 ***************************************************************************/
static void CablePlug_UpdateState(void)
{
    uint8_t pluggedNow = IS_CABLE_PLUGGED() ? 1 : 0;

    if (pluggedNow) {
        if (!cableWasPlugged) {
            LED_PlugIn_Start();
        }
        stayAwakeFlag = 1;
        cableUnplugTime = 0;
        cableWasPlugged = 1;
        return;
    }

    if (cableWasPlugged) {
        cableUnplugTime = HAL_GetTick();
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

    cablePlugFlag = 0;
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
            LED_Rainbow(10, 255);
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
 *   poll reset the still-timer when motion is detected.
 ***************************************************************************/
void State_Stabilizing_Loop(void)
{
    if (!GET_ARMED_BIT(deviceState)) {
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        LED_Off();
        return;
    }

    if (!findMyActive) {
        if (GET_LIGHTS_BIT(deviceState)) {
            LED_Pulse(1000, 0, 0, 255, 255);
        } else {
            LED_Off();
        }
    }

    static uint32_t last_still_time = 0;
    static uint8_t  stabilize_started = 0;

    if (!stabilize_started) {
        last_still_time = HAL_GetTick();
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

        // Log the wake event up-front so brief motions that stop before any
        // further classification still get recorded.
        if (GET_LOGGING_BIT(deviceState)) {
            MotionLogger_LogEvent(MOTION_TYPE_IN_MOTION);
        }

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
            }
        }
    }

    if (!findMyActive) {
        if (GET_LIGHTS_BIT(deviceState) && !PowerMgmt_IsLowPower()) {
            LED_Armed(10, 255);
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
        switch (alarmType) {
            case ALARM_NONE:
                break;
            case ALARM_CALM:
                if (showLights) LED_Alarm(300, 255, 0, 0, 255);
                BUZZER_StartCalmAlarm();
                break;
            case ALARM_NORMAL:
                if (showLights) LED_Alarm(300, 255, 0, 0, 255);
                BUZZER_StartNormalAlarm();
                break;
            case ALARM_LOUD:
                if (showLights) LED_Alarm(125, 255, 225, 0, 255);
                BUZZER_StartLaCucaracha();
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

        if (mlc_out == MLC_STATE_IN_MOTION || mlc_out == MLC_STATE_SHAKEN
            || impact || freefall) {
            last_motion_time = HAL_GetTick();
            motion_this_iter = 1;
            APP_DBG_MSG("Alarm timer reset → %us\n", alarm_duration_s);
        }

        if (GET_LOGGING_BIT(deviceState)) {
            MotionLogger_LogEvent(motionType);
            LOCKSERVICE_SendMotionAlert(motionType);
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
                APP_DBG_MSG("Alarm timer reset → %us\n", alarm_duration_s);
            }
        }

        uint8_t impact, freefall;
        lis2dux12_app_check_fsm_events(&impact, &freefall);
        if (impact || freefall) {
            last_motion_time = HAL_GetTick();
            motion_this_iter = 1;
            APP_DBG_MSG("Alarm timer reset → %us\n", alarm_duration_s);
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
 *   is settling so the iOS app can show "Stabilizing…".
 ***************************************************************************/
void StateMachine_ChangeState(SystemState_t newState)
{
    if (newState != currentState) {
        previousState = currentState;
        currentState = newState;
        stateEntryTime = HAL_GetTick();

        if (newState == STATE_STABILIZING || newState == STATE_LOCKED || newState == STATE_ALARM_ACTIVE) {
            SET_ARMED_BIT(deviceState, 1);
        } else {
            SET_ARMED_BIT(deviceState, 0);
        }

        lis2dux12_app_set_stabilizing(newState == STATE_STABILIZING ? 1 : 0);

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
        LED_Solid(0, 255, 0, 255);
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
