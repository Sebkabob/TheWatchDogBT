/***************************************************************************
 * state_machine.c
 * created by Sebastian Forenza 2026
 *
 * Main code loop with melody-duration-based alarm timeout
 *
 * V2 PCB:
 *   - Charging detect: STAT       (PA11) LOW = charging
 *   - Cable detect:    BQ251_PG   (PB4)  LOW = cable plugged in
 *   - Accel interrupt: ACCEL_INT  (PB15)
 *   - Debug hold:      DEBUG_GPIO (PB5)  HIGH = stay awake
 *
 * LOW POWER FIX:
 *   - Enter PowerMgmt_EnterLowPower_Idle when disconnected + no cable
 *   - Enter PowerMgmt_EnterLowPower_Armed when locked + disconnected
 *   - Restore peripherals on reconnection or cable plug-in
 *
 * CABLE PLUG INTERRUPT:
 *   - PB4 is now EXTI falling-edge: fires when cable is plugged in
 *   - ISR sets cablePlugFlag + stayAwakeFlag
 *   - After cable is removed, device stays awake for CABLE_UNPLUG_AWAKE_MS
 *     then returns to low power
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

/* Global state variables */
volatile SystemState_t currentState = STATE_CONNECTED_IDLE;
volatile SystemState_t previousState = STATE_CONNECTED_IDLE;
volatile uint8_t deviceState = 0;
volatile uint8_t deviceInfo = 0;
volatile uint8_t deviceBattery = 100;
volatile uint8_t connectionStatus = 0;

/* Static variables for timing */
static uint32_t stateEntryTime = 0;

volatile uint8_t stayAwakeFlag = 0;

/* Tick at which the post-connect motion-grace window expires.
 * Set via StateMachine_StartMotionGrace().  While HAL_GetTick() is
 * below this, the LOCKED state suppresses motion-triggered alarm
 * transitions so a BLE connect (which runs RestoreAll → UCF reload)
 * doesn't immediately fire the alarm. */
static volatile uint32_t motion_grace_until = 0;

void StateMachine_StartMotionGrace(uint32_t ms)
{
    uint32_t until = HAL_GetTick() + ms;
    /* Don't shorten an existing window */
    if ((int32_t)(until - motion_grace_until) > 0) {
        motion_grace_until = until;
    }
}

static uint8_t MotionGrace_Active(void)
{
    return (int32_t)(motion_grace_until - HAL_GetTick()) > 0;
}

/* Cable plug detection */
volatile uint8_t cablePlugFlag = 0;
static uint32_t cableUnplugTime = 0;      /* tick when cable was last seen removed */
static uint8_t  cableWasPlugged = 0;       /* tracks previous cable state for edge detect */

static volatile uint8_t findMyActive = 0;

/***************************************************************************
 * CHARGING LED COLOR — smooth red→yellow→green gradient based on SOC
 *   0%  = pure red    (255, 0,   0)
 *  50%  = yellow      (255, 255, 0)
 * 100%  = pure green  (0,   255, 0)
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
 * CABLE PLUG ISR CALLBACK
 * Called from GPIOB_IRQHandler when PB4 fires (falling edge = cable in)
 ***************************************************************************/
void CablePlug_IRQCallback(void)
{
    cablePlugFlag = 1;
    stayAwakeFlag = 1;
}

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

    /* Initialise cable state tracking */
    cableWasPlugged = IS_CABLE_PLUGGED() ? 1 : 0;
    cableUnplugTime = 0;
}

/***************************************************************************
 * CABLE PLUG MANAGEMENT
 * Handles the post-unplug awake window so the device doesn't slam
 * straight back to deep sleep.
 ***************************************************************************/
static void CablePlug_UpdateState(void)
{
    uint8_t pluggedNow = IS_CABLE_PLUGGED() ? 1 : 0;

    if (pluggedNow) {
        /* Rising edge: cable just plugged in. Kick off the LED fade-out
         * transition before any state-loop LED routine runs. */
        if (!cableWasPlugged) {
            LED_PlugIn_Start();
        }
        /* Cable is in — stay awake, clear unplug timer */
        stayAwakeFlag = 1;
        cableUnplugTime = 0;
        cableWasPlugged = 1;
        return;
    }

    /* Cable is NOT plugged in */
    if (cableWasPlugged) {
        /* Just unplugged — start the awake window and the LED fade-out */
        cableUnplugTime = HAL_GetTick();
        cableWasPlugged = 0;
        LED_PlugOut_Start();
    }

    /* If we're in the post-unplug awake window, keep stayAwakeFlag set */
    if (cableUnplugTime != 0) {
        if ((HAL_GetTick() - cableUnplugTime) < CABLE_UNPLUG_AWAKE_MS) {
            stayAwakeFlag = 1;
        } else {
            /* Window expired — let the state machine decide */
            cableUnplugTime = 0;
            /* Don't clear stayAwakeFlag here — the state loop will
             * set it appropriately based on the current state */
        }
    }

    /* Clear the ISR flag after we've handled it */
    cablePlugFlag = 0;
}

void State_Disconnected_Idle_Loop(void)
{
    /* === Cable plugged in: stay awake, show charging status === */
    if (IS_CABLE_PLUGGED()) {
        /* Restore peripherals if we were in low power */
        if (PowerMgmt_IsLowPower()) {
            PowerMgmt_RestoreAll();
        }
        stayAwakeFlag = 1;

        if (LED_PlugIn_InProgress()) {
            LED_PlugIn_Tick();                  /* fade-to-black + 250 ms gap */
        } else if (IS_CHARGING_NOW() && !BATTERY_IsFullCached()) {
            LED_ChargingPulse();                /* SOC gradient: red→green */
        } else if (BATTERY_IsFullCached()) {
            LED_Solid(0, 255, 0, 255);          /* green solid - charged */
        } else {
            LED_Off();                          /* cable in, charger not started yet */
        }
    } else {
        /* === No cable, no connection: enter low power === */

        /* Only clear stayAwakeFlag if the post-unplug window has expired */
        if (cableUnplugTime == 0) {
            stayAwakeFlag = 0;
        }

        if (LED_PlugOut_InProgress()) {
            LED_PlugOut_Tick();                 /* fade-to-black after unplug */
        } else {
            LED_Off();
        }

        if (!stayAwakeFlag && !PowerMgmt_IsLowPower()) {
            PowerMgmt_EnterLowPower_Idle();
        }
    }

    /* State Switch */
    if (connectionStatus) {
        /* Restore everything before entering connected state */
        if (PowerMgmt_IsLowPower()) {
            PowerMgmt_RestoreAll();
        }
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
    }
}

void State_Connected_Idle_Loop(void)
{
    stayAwakeFlag = 1;

    /* Lights — skip while find-my is active so it gets clean LED control */
    if (!findMyActive) {
        if (IS_CABLE_PLUGGED()) {
            if (LED_PlugIn_InProgress()) {
                LED_PlugIn_Tick();                  /* fade-to-black + 250 ms gap */
            } else if (IS_CHARGING_NOW() && !BATTERY_IsFullCached()) {
                LED_ChargingPulse();                /* SOC gradient: red→green */
            } else if (BATTERY_IsFullCached()) {
                LED_Solid(0, 255, 0, 255);          /* green solid - charged */
            } else {
                LED_Off();                          /* cable in, charger not started yet */
            }
        } else if (LED_PlugOut_InProgress()) {
            LED_PlugOut_Tick();                 /* fade-to-black after unplug */
        } else if (GET_LIGHTS_BIT(deviceState)) {
            LED_Rainbow(10, 255);  /* rainbow - normal */
        } else {
            LED_Off();
        }
    }

    /* State Switch - DISCONNECTED */
    if (!connectionStatus) {
        StateMachine_ChangeState(STATE_DISCONNECTED_IDLE);
    }

    /* State Switch - STABILIZING (wait for stillness before locking) */
    if (GET_ARMED_BIT(deviceState)) {
        LIS2DUX12_ClearMotion();
        StateMachine_ChangeState(STATE_STABILIZING);
    }
}

void State_Stabilizing_Loop(void)
{
    /* Disarmed — cancel stabilizing */
    if (!GET_ARMED_BIT(deviceState)) {
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        LED_Off();
        return;
    }

    /* Disconnected while stabilizing — keep going, we're still armed */
    if (!connectionStatus) {
        /* but don't sleep yet, we need the sensor */
    }

    /* Pulsing blue LED = stabilizing (skip during find-my) */
    if (!findMyActive) {
        if (GET_LIGHTS_BIT(deviceState)) {
            LED_Pulse(1000, 0, 0, 255, 255);
        } else {
            LED_Off();
        }
    }

    static uint32_t last_still_time = 0;
    static uint8_t  stabilize_started = 0;

    /* First call after entering this state — reset timer */
    if (!stabilize_started) {
        last_still_time = HAL_GetTick();
        stabilize_started = 1;
        LIS2DUX12_ClearMotion();
    }

    /* Check MLC interrupt for motion */
    if (LIS2DUX12_IsMotionDetected()) {
        uint8_t mlc_out;
        lis2dux12_app_get_mlc_output(&mlc_out);
        if (mlc_out == MLC_STATE_IN_MOTION || mlc_out == MLC_STATE_SHAKEN) {
            last_still_time = HAL_GetTick();  /* reset timer */
        }
    }

    /* Poll MLC at 10 Hz to catch motion the interrupt might miss */
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

    /* 3 seconds of no motion — lock */
    if (HAL_GetTick() - last_still_time >= 3000) {
        stabilize_started = 0;
        StateMachine_ChangeState(STATE_LOCKED);
    }
}

void State_Locked_Loop(void)
{
    /* Motion assessment state for wake-from-LP */
    static uint8_t  motion_assessing = 0;
    static uint32_t motion_assess_start = 0;
    #define MOTION_ASSESS_TIMEOUT_MS 10000

    /* Deferred motion alert state — declared up-front so the LP-wake
     * fast-paths (HIGH/MEDIUM sensitivity) can seed it before the
     * MLC settle path picks it up. */
    static uint8_t      motion_pending = 0;
    static MotionType_t pending_type   = MOTION_TYPE_NONE;
    static uint32_t     motion_pending_tick = 0;

    /* State Switch - UNLOCKED */
    if (!GET_ARMED_BIT(deviceState)) {
        motion_assessing = 0;
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        LED_Off();
        return;
    }

    /*--------------------------------------------------------------
     * Wake from armed LP on motion interrupt.
     *
     * The MLC kept running across MCU sleep (UCF stays loaded — see
     * LIS2DUX12_ConfigArmedSleep), so the very first MLC read after
     * peripherals come back will already reflect the motion that
     * woke us.  No UCF reload, no MLC accumulation window.
     *
     * Behaviour:
     *   - INT alone never fires the alarm.
     *   - We read MLC + FSM data first.
     *   - If MLC says IN_MOTION / SHAKEN → seed deferred alert and
     *     transition to ALARM_ACTIVE.
     *   - If MLC still says STATIONARY (a "breeze" wake) → leave
     *     motion_assessing armed for ~10 s; the regular MLC/FSM
     *     processing block below will catch any escalation.
     *--------------------------------------------------------------*/
    if (PowerMgmt_IsLowPower() && LIS2DUX12_PeekMotionStatus()) {
        PowerMgmt_RestoreForMotion();
        LIS2DUX12_ClearMotion();
        stayAwakeFlag = 1;
        motion_assessing = 1;
        motion_assess_start = HAL_GetTick();

        /* Log the wake event immediately so brief motions that stopped
         * before any further classification still get recorded. */
        if (GET_LOGGING_BIT(deviceState)) {
            MotionLogger_LogEvent(MOTION_TYPE_IN_MOTION);
        }

        /* Significant-motion fast-path (MEDIUM + HIGH only).
         * LOW always waits for MLC.  For MEDIUM/HIGH, if the raw
         * acceleration vector deviates from 1 g by more than 250 mg,
         * fire the alarm immediately — no need to wait for MLC.
         * Threshold compared in mg² to avoid a sqrt. */
        uint8_t fast_fired = 0;
        if (GET_SENSITIVITY(deviceState) != SENSITIVITY_LOW) {
            int16_t ax_mg = 0, ay_mg = 0, az_mg = 0;
            if (lis2dux12_app_read_accel_mg(&ax_mg, &ay_mg, &az_mg) == 0) {
                int32_t mag2 = (int32_t)ax_mg * ax_mg
                             + (int32_t)ay_mg * ay_mg
                             + (int32_t)az_mg * az_mg;
                /* 1g ± 250 mg gate:  750 mg² … 1250 mg² */
                const int32_t hi2 = 1250L * 1250L;
                const int32_t lo2 =  750L *  750L;
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

        /* Read MLC directly — chip already has a fresh classification. */
        uint8_t mlc_out;
        if (!fast_fired && lis2dux12_app_get_mlc_output(&mlc_out) == 0) {
            lis2dux12_app_update_cached_state(mlc_out);

            /* FSM impact / freefall — discrete events, fire immediately. */
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
                /* Seed deferred alert so the MLC-settle path resolves
                 * door events + sends the BLE alert when motion stops. */
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
            /* else: STATIONARY — wake-up engine tripped on tiny motion
             * but MLC didn't escalate.  Stay awake briefly via the
             * motion_assessing window in case it does. */
        }
    }

    if (!findMyActive) {
        if (GET_LIGHTS_BIT(deviceState) && !PowerMgmt_IsLowPower()) {
            LED_Armed(10, 255);
        } else {
            LED_Off();
        }
    }

    /*--------------------------------------------------------------
     * Deferred motion alert.
     * When MLC says "in motion" we trigger the alarm immediately
     * but DEFER the log/alert.  When it settles we check the door:
     *   - door moved  → log DOOR_OPENED  (suppress in-motion)
     *   - door same   → log the pending motion type
     * Impact/freefall are always sent immediately (discrete events).
     * (motion_pending / pending_type / motion_pending_tick declared
     *  at the top of the function so the LP-wake fast-path can seed
     *  them before the MLC catches up.)
     *--------------------------------------------------------------*/

    /* Only run MLC/FSM processing when peripherals are up */
    if (!PowerMgmt_IsLowPower() && !MotionGrace_Active()) {

        /* MLC interrupt fired — read classification */
        if (LIS2DUX12_IsMotionDetected()) {
            uint8_t mlc_out;
            lis2dux12_app_get_mlc_output(&mlc_out);
            lis2dux12_app_update_cached_state(mlc_out);

            /* Impact / freefall — always send immediately */
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

            /* Motion started — defer the alert, but trigger alarm now */
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

            /* MLC returned to stationary — resolve the pending motion */
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

        /* Timeout: if motion pending >3s without settling, send it anyway */
        if (motion_pending && (HAL_GetTick() - motion_pending_tick > 3000)) {
            if (GET_LOGGING_BIT(deviceState)) {
                MotionLogger_LogEvent(pending_type);
                LOCKSERVICE_SendMotionAlert(pending_type);
            }
            motion_pending = 0;
        }

        /* 100 ms poll (~10 Hz) — FSM + door position */
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
        /* During the post-connect grace window, drain any motion the
         * UCF reload + user handling produced so it doesn't latch
         * and fire the alarm the instant grace ends. */
        if (LIS2DUX12_IsMotionDetected()) {
            uint8_t mlc_out;
            lis2dux12_app_get_mlc_output(&mlc_out);
            lis2dux12_app_update_cached_state(mlc_out);
        }
        uint8_t impact = 0, freefall = 0;
        lis2dux12_app_check_fsm_events(&impact, &freefall);
        (void)impact; (void)freefall;
        motion_pending = 0;
    } /* end if (!PowerMgmt_IsLowPower()) */

    /* === If not connected while locked === */
    if (!connectionStatus) {
        LED_Off();

        if (motion_assessing) {
            /* Assessment timeout: no classifiable motion in 10s → sleep */
            if (HAL_GetTick() - motion_assess_start >= MOTION_ASSESS_TIMEOUT_MS) {
                motion_assessing = 0;
                stayAwakeFlag = 0;
            }
        } else {
            /* Only clear stayAwakeFlag if the post-unplug window is done */
            if (cableUnplugTime == 0 && !IS_CABLE_PLUGGED()) {
                stayAwakeFlag = 0;
            }
        }

        if (!stayAwakeFlag && !PowerMgmt_IsLowPower()) {
            motion_pending = 0;
            PowerMgmt_EnterLowPower_Armed();
        }
    } else {
        /* Connected: make sure peripherals are restored */
        if (PowerMgmt_IsLowPower()) {
            PowerMgmt_RestoreAll();
        }
        motion_assessing = 0;
    }
}

void State_Alarm_Active_Loop(void)
{
    stayAwakeFlag = 1;

    /* Make sure peripherals are up for alarm */
    if (PowerMgmt_IsLowPower()) {
        PowerMgmt_RestoreAll();
    }

    static uint32_t last_motion_time = 0;
    static uint8_t alarm_started = 0;
    static uint32_t melody_duration_ms = 0;

    /* Check if disarmed - EXIT INSTANTLY */
    if (!GET_ARMED_BIT(deviceState)) {
        BUZZER_Stop();
        alarm_started = 0;
        melody_duration_ms = 0;
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        return;
    }

    /* Start the appropriate alarm if not already playing */
    if (!alarm_started) {
        uint8_t alarmType = GET_ALARM_TYPE(deviceState);
        uint8_t showLights = GET_LIGHTS_BIT(deviceState);
        switch (alarmType) {
            case ALARM_NONE:
                melody_duration_ms = 1000;
                break;
            case ALARM_CALM:
                if (showLights) LED_Alarm(300, 255, 0, 0, 255);
                BUZZER_StartCalmAlarm();
                melody_duration_ms = BUZZER_GetCalmAlarmDuration();
                break;
            case ALARM_NORMAL:
                if (showLights) LED_Alarm(300, 255, 0, 0, 255);
                BUZZER_StartNormalAlarm();
                melody_duration_ms = BUZZER_GetNormalAlarmDuration();
                break;
            case ALARM_LOUD:
                if (showLights) LED_Alarm(125, 255, 225, 0, 255);
                BUZZER_StartLaCucaracha();
                melody_duration_ms = BUZZER_GetLaCucarachaDuration();
                break;
            default:
                melody_duration_ms = 1000;
                break;
        }

        alarm_started = 1;
        last_motion_time = HAL_GetTick();
    }

    /* Check for new motion via MLC interrupt — RESET TIMER */
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

        /* Only reset alarm timer on active motion states */
        if (mlc_out == MLC_STATE_IN_MOTION || mlc_out == MLC_STATE_SHAKEN
            || impact || freefall) {
            last_motion_time = HAL_GetTick();
        }

        if (GET_LOGGING_BIT(deviceState)) {
            MotionLogger_LogEvent(motionType);
            LOCKSERVICE_SendMotionAlert(motionType);
        }
    }

    /* Poll MLC + door detector to keep alarm alive during continued motion.
     * The MLC interrupt only fires on STATE CHANGES, so continuous motion
     * won't re-trigger it. Polling catches this. */

    /* MLC + FSM + door — 500 ms */
    static uint32_t last_mlc_poll = 0;
    if (HAL_GetTick() - last_mlc_poll > 500) {
        last_mlc_poll = HAL_GetTick();

        uint8_t mlc_out;
        if (lis2dux12_app_get_mlc_output(&mlc_out) == 0) {
            lis2dux12_app_update_cached_state(mlc_out);
            if (mlc_out == MLC_STATE_IN_MOTION || mlc_out == MLC_STATE_SHAKEN) {
                last_motion_time = HAL_GetTick();
            }
        }

        /* Also poll FSM */
        uint8_t impact, freefall;
        lis2dux12_app_check_fsm_events(&impact, &freefall);
        if (impact || freefall) {
            last_motion_time = HAL_GetTick();
            if (GET_LOGGING_BIT(deviceState)) {
                MotionType_t mt = impact ? MOTION_TYPE_IMPACT : MOTION_TYPE_FREEFALL;
                MotionLogger_LogEvent(mt);
                LOCKSERVICE_SendMotionAlert(mt);
            }
        }
    }

    /* Exit alarm only after at least one full melody duration with no motion */
    if ((HAL_GetTick() - last_motion_time) >= melody_duration_ms) {
        BUZZER_Stop();
        alarm_started = 0;
        melody_duration_ms = 0;
        StateMachine_ChangeState(STATE_LOCKED);
        return;
    }
}

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

        /* Status byte 6 carries CACHED_STATE_STABILIZING (0xFE) while
         * the device is settling so the app can show "Stabilizing…". */
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
 * FIND MY DEVICE — non-blocking ping with synced green LED
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
        /* Sequence finished — clean up */
        LED_Off();
        findMyActive = 0;
        return;
    }

    /* Green LED on during each tone, off during gaps */
    if (BUZZER_IsToneActive()) {
        LED_Solid(0, 255, 0, 255);
    } else {
        LED_Off();
    }
}

void StateMachine_Run(void)
{
    /* Handle cable plug/unplug events (ISR flag + debounce + timeout) */
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
