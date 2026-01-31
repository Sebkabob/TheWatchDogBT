/***************************************************************************
 * state_machine.c
 * created by Sebastian Forenza 2026
 *
 * Main code loop
 ***************************************************************************/

#include "state_machine.h"
#include "lights.h"
#include "sound.h"
#include "battery.h"
#include "lockservice_app.h"
#include "accelerometer.h"
#include "motion_logger.h"
#include "power_management.h"

// Global state variables
volatile SystemState_t currentState = STATE_CONNECTED_IDLE;
volatile SystemState_t previousState = STATE_CONNECTED_IDLE;
volatile uint8_t deviceState = 0;
volatile uint8_t deviceInfo = 0;
volatile uint8_t deviceBattery = 100;

// Static variables for timing
static uint32_t stateEntryTime = 0;
static uint32_t lastActivityTime = 0;

// Inactivity timeout constant
#define INACTIVITY_TIMEOUT_MS  10000

void StateMachine_Init(void)
{
    currentState = STATE_CONNECTED_IDLE;
    stateEntryTime = HAL_GetTick();

    deviceState = 0;
    SET_ARMED_BIT(deviceState, 0);
    SET_ALARM_TYPE(deviceState, ALARM_CALM);
    SET_SENSITIVITY(deviceState, SENSITIVITY_MEDIUM);
    SET_LIGHTS_BIT(deviceState, 1);
    SET_LOGGING_BIT(deviceState, 1);
    SET_SILENCE_BIT(deviceState, 1);

    deviceInfo = 0;
}

void StateMachine_UpdateActivity(void) {
    lastActivityTime = HAL_GetTick();
}

void StateMachine_CheckInactivityTimeout(void) {
    if (currentState == STATE_ALARM_ACTIVE) {
        return;
    }

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0) {
        lastActivityTime = HAL_GetTick();
        return;
    }

    if ((HAL_GetTick() - lastActivityTime) >= INACTIVITY_TIMEOUT_MS) {
        StateMachine_ChangeState(STATE_SLEEP);
    }
}

void State_Disconnected_Idle_Loop(){
    // bluetoothPairingLED
}

void State_Connected_Idle_Loop(){
    if (GET_ARMED_BIT(deviceState)) {
    	LIS2DUX12_ClearMotion();
        StateMachine_ChangeState(STATE_LOCKED);
        HAL_Delay(10);
    } else {
        if (GET_LIGHTS_BIT(deviceState)) {
            //LED_Rainbow(10,20);
        } else {
        	LED_Off();
        }
    }
}

void State_Locked_Loop(){
    static uint8_t lastMotionState = GPIO_PIN_RESET;

    // If no longer armed, switch out of locked state
    if (!GET_ARMED_BIT(deviceState)) {
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        lastMotionState = GPIO_PIN_RESET;
        return;
    }

    if (GET_LIGHTS_BIT(deviceState)) {
    	//Locked LED
        LED_Armed(10,150);
    } else {
        LED_Off();
    }

    // Currently very crude motion detection, in future will use algorithm
    // TODO: Add motion sensing algorithm
    uint8_t currentMotionState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
    if (currentMotionState == GPIO_PIN_SET && lastMotionState == GPIO_PIN_RESET) {
        if (GET_LOGGING_BIT(deviceState)) {
            MotionLogger_LogEvent(1);
            LOCKSERVICE_SendMotionAlert();
        }

        if (!GET_SILENCE_BIT(deviceState)){
            StateMachine_ChangeState(STATE_ALARM_ACTIVE);
        } else {
            while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET) {
                HAL_Delay(10);
            }
        }
    }
    lastMotionState = currentMotionState;

}

void State_Sleep_Loop(){
    // ============ ENTERING SLEEP ============
    // Debug sequence: 3 descending tones
	BUZZER_Tone(400, 50);
    HAL_Delay(100);
    BUZZER_Tone(300, 50);
    HAL_Delay(100);
    BUZZER_Tone(200, 50);
    HAL_Delay(100);

    LED_Off();

    // Enter deep stop mode
    Enter_DeepStop_Mode();

    // ============ WOKE UP FROM SLEEP ============
    // System execution resumes here after wakeup

    // SHORT DELAY before any peripheral access
    HAL_Delay(100);

    // Debug sequence: 3 ascending tones to confirm wakeup
    BUZZER_Tone(200, 50);
    HAL_Delay(100);
    BUZZER_Tone(300, 50);
    HAL_Delay(100);
    BUZZER_Tone(400, 50);
    HAL_Delay(200);

    // Reinitialize system
    Wakeup_System_Init();

    // Confirm reinitialization with rapid beeps
    BUZZER_Tone(500, 30);
    HAL_Delay(50);
    BUZZER_Tone(500, 30);
    HAL_Delay(50);
    BUZZER_Tone(500, 30);
    HAL_Delay(200);

    // Go back to armed/locked state
    StateMachine_ChangeState(STATE_LOCKED);
}

void State_Alarm_Active_Loop(){
    if (!GET_ARMED_BIT(deviceState)) {
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        return;
    }

    uint8_t alarmType = GET_ALARM_TYPE(deviceState);

    switch(alarmType) {
        case ALARM_NONE:
            break;
        case ALARM_CALM:
        	SOUND_CalmAlarm();
            break;
        case ALARM_NORMAL:
        	SOUND_NormalAlarm();
            break;
        case ALARM_LOUD:
        	SOUND_LoudAlarm();
            break;
        default:
            break;
    }

    StateMachine_ChangeState(STATE_LOCKED);
}

void StateMachine_ChangeState(SystemState_t newState)
{
    if(newState != currentState) {
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

void ChargingCheck(){
	static uint8_t previousBattery = 0xFF;

	if (IS_CHARGING(HAL_GPIO_ReadPin(GPIOB, CHARGE_Pin))) {
		SET_BATTERY_CHARGING(deviceBattery);
		if (!GET_ARMED_BIT(deviceState)){
		    LED_Charging(10,25);
		}
	} else {
		CLEAR_BATTERY_CHARGING(deviceBattery);
	}

	if (deviceBattery != previousBattery) {
	    LOCKSERVICE_SendStatusUpdate();
	    previousBattery = deviceBattery;
		LED_Off();
	}
}

void StateMachine_Run(void)
{
	ChargingCheck();

    switch(currentState)
    {
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
