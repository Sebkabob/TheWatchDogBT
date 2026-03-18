/***************************************************************************
 * state_machine.c
 * created by Sebastian Forenza 2026
 *
 * Main code loop with melody-duration-based alarm timeout
 *
 * NEW PCB changes:
 *   - Charging detect: BQ251_STAT (PA9) LOW = charging
 *   - Cable detect:    BQ251_PG   (PB4) LOW = cable plugged in
 *   - Accel interrupt: ACCEL_INT  (PB15)
 *
 * LOW POWER FIX:
 *   - Enter PowerMgmt_EnterLowPower_Idle when disconnected + no cable
 *   - Enter PowerMgmt_EnterLowPower_Armed when locked + disconnected
 *   - Restore peripherals on reconnection or cable plug-in
 ***************************************************************************/

#include "state_machine.h"
#include "lights.h"
#include "sound.h"
#include "battery.h"
#include "lockservice_app.h"
#include "accelerometer.h"
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
static uint32_t lastActivityTime = 0;

volatile uint8_t stayAwakeFlag = 0;

static uint32_t lastBLEActivityTime = 0;
#define BLE_INACTIVITY_TIMEOUT_MS  10000  /* 10 seconds */

void StateMachine_UpdateBLEActivity(void) {
    lastBLEActivityTime = HAL_GetTick();
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
}

void StateMachine_UpdateActivity(void) {
    lastActivityTime = HAL_GetTick();
}

void StateMachine_CheckInactivityTimeout(void) {
    if (currentState == STATE_ALARM_ACTIVE) {
        return;
    }

    /* If cable is plugged in, reset timeout */
    if (IS_CABLE_PLUGGED()) {
        lastActivityTime = HAL_GetTick();
        return;
    }

    if ((HAL_GetTick() - lastActivityTime) >= BLE_INACTIVITY_TIMEOUT_MS) {
        StateMachine_ChangeState(STATE_SLEEP);
    }
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

        if (IS_CHARGING_NOW()) {
            LED_Pulse(4000, 255, 100, 0, 60); /* orange pulse - charging */
        } else {
            LED_Pulse(4000, 0, 255, 0, 60);   /* green pulse - charged */
        }
    } else {
        /* === No cable, no connection: enter low power === */
        stayAwakeFlag = 0;
        LED_Off();

        if (!PowerMgmt_IsLowPower()) {
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

    /* Lights */
    if (IS_CABLE_PLUGGED()) {
        if (IS_CHARGING_NOW()) {
            LED_Pulse(4000, 255, 100, 0, 60); /* orange pulse - charging */
        } else {
            LED_Pulse(4000, 0, 255, 0, 60);   /* green pulse - charged */
        }
    } else {
        LED_Rainbow(5, 15);  /* rainbow - normal */
    }

    /* State Switch - DISCONNECTED */
    if (!connectionStatus) {
        StateMachine_ChangeState(STATE_DISCONNECTED_IDLE);
    }

    /* State Switch - LOCKED */
    if (GET_ARMED_BIT(deviceState)) {
        LIS2DUX12_ClearMotion();
        StateMachine_ChangeState(STATE_LOCKED);
        HAL_Delay(10);
    }
}

void State_Locked_Loop(void)
{
    static uint8_t lastMotionState = GPIO_PIN_RESET;

    /* State Switch - UNLOCKED */
    if (!GET_ARMED_BIT(deviceState)) {
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        lastMotionState = GPIO_PIN_RESET;
        LED_Off();
        return;
    }

    if (GET_LIGHTS_BIT(deviceState)) {
        LED_Armed(10, 150);
    } else {
        LED_Off();
    }

    /* Motion detection via ACCEL_INT (PB15) */
    uint8_t currentMotionState = HAL_GPIO_ReadPin(ACCEL_INT_GPIO_Port, ACCEL_INT_Pin);
    if (currentMotionState == GPIO_PIN_SET && lastMotionState == GPIO_PIN_RESET) {
        if (GET_LOGGING_BIT(deviceState)) {
            stayAwakeFlag = 1;
            MotionLogger_LogEvent(1);
            LOCKSERVICE_SendMotionAlert();
        }

        if (!GET_SILENCE_BIT(deviceState)) {
            StateMachine_ChangeState(STATE_ALARM_ACTIVE);
        } else {
            while (HAL_GPIO_ReadPin(ACCEL_INT_GPIO_Port, ACCEL_INT_Pin) == GPIO_PIN_SET) {
                HAL_Delay(10);
            }
        }
    }
    lastMotionState = currentMotionState;

    /* === If not connected while locked, enter armed low power === */
    if (!connectionStatus) {
        LED_Off();
        stayAwakeFlag = 0;

        if (!PowerMgmt_IsLowPower()) {
            /* Armed low power keeps accel interrupt active */
            PowerMgmt_EnterLowPower_Armed();
        }
    } else {
        /* Connected: make sure peripherals are restored */
        if (PowerMgmt_IsLowPower()) {
            PowerMgmt_RestoreAll();
        }
    }
}

void State_Sleep_Loop(void)
{
    /* Placeholder for deep sleep entry */
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
        switch (alarmType) {
            case ALARM_NONE:
                melody_duration_ms = 1000;
                break;
            case ALARM_CALM:
                LED_Alarm(300, 255, 0, 0, 255);
                BUZZER_StartCalmAlarm();
                melody_duration_ms = BUZZER_GetCalmAlarmDuration();
                break;
            case ALARM_NORMAL:
                LED_Alarm(300, 255, 0, 0, 255);
                BUZZER_StartNormalAlarm();
                melody_duration_ms = BUZZER_GetNormalAlarmDuration();
                break;
            case ALARM_LOUD:
                LED_Alarm(125, 255, 225, 0, 100);
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

    /* Check for new motion - RESET TIMER */
    if (LIS2DUX12_IsMotionDetected()) {
        last_motion_time = HAL_GetTick();

        if (GET_LOGGING_BIT(deviceState)) {
            MotionLogger_LogEvent(1);
            LOCKSERVICE_SendMotionAlert();
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

        if (newState == STATE_LOCKED || newState == STATE_ALARM_ACTIVE) {
            SET_ARMED_BIT(deviceState, 1);
        } else {
            SET_ARMED_BIT(deviceState, 0);
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
        if (IS_CHARGING_NOW()) {
            SET_BATTERY_CHARGING(deviceBattery);
        } else {
            CLEAR_BATTERY_CHARGING(deviceBattery);
        }
    }
}

void StateMachine_Run(void)
{
    ChargingCheck();
    BUZZER_Update();

    switch (currentState) {
        case STATE_DISCONNECTED_IDLE:
            State_Disconnected_Idle_Loop();
            break;
        case STATE_CONNECTED_IDLE:
            State_Connected_Idle_Loop();
            break;
        case STATE_LOCKED:
            State_Locked_Loop();
            break;
        case STATE_SLEEP:
            State_Sleep_Loop();
            break;
        case STATE_ALARM_ACTIVE:
            State_Alarm_Active_Loop();
            break;
        default:
            StateMachine_ChangeState(STATE_DISCONNECTED_IDLE);
            break;
    }
}
