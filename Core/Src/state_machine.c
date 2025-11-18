
#include "state_machine.h"
#include "lights.h"
#include "sound.h"
#include "battery.h"
#include "lockservice_app.h"

// Global state variables
volatile SystemState_t currentState = STATE_CONNECTED_IDLE;
volatile SystemState_t previousState = STATE_CONNECTED_IDLE;
volatile uint8_t lockState = 0;
volatile uint8_t deviceInfo = 0;

// Static variables for timing
static uint32_t stateEntryTime = 0;


void StateMachine_Init(void)
{
    currentState = STATE_CONNECTED_IDLE;
    stateEntryTime = HAL_GetTick();
}

void StateMachine_ChangeState(SystemState_t newState)
{
    if(newState != currentState) {
        previousState = currentState;
        currentState = newState;
        stateEntryTime = HAL_GetTick();

        // Update BLE characteristic when state changes
        LOCKSERVICE_SendStatusUpdate();
    }
}

void StateMachine_Run(void)
{
    switch(currentState)
    {
        case STATE_SLEEP:

            break;

        case STATE_ARMED:
            testLED(10);
            if (Battery_IsCharging()){
            	StateMachine_ChangeState(STATE_CHARGING);
            }
            else if (lockState == 0xF){
        		StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        	}
            break;

        case STATE_ALARM_ACTIVE:
        	for (int i = 0; i < 5; i++){
        		playTone(4186,400);
        		playTone(3520,400);
        	}
        	StateMachine_ChangeState(STATE_CONNECTED_IDLE);
            break;

        case STATE_CHARGING:
        	ChargeLED(50);
            break;

        case STATE_CONNECTED_IDLE:
            if (Battery_IsCharging()){
            	StateMachine_ChangeState(STATE_CHARGING);
            }
        	else if (lockState == 0xF){
        		turnOffLED();
        		deviceInfo = 0;
        		LOCKSERVICE_SendStatusUpdate();
        	}
        	else if (lockState == 1){
                deviceInfo = 1;
                StateMachine_ChangeState(STATE_ARMED);
            }
            else if (lockState == 2) {
            	StateMachine_ChangeState(STATE_ALARM_ACTIVE);
            }
            else {
            	turnOffLED();
            }
            break;

        default:
            StateMachine_ChangeState(STATE_CONNECTED_IDLE);
            break;
    }
}
