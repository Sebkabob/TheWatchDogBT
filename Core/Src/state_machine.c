#include "state_machine.h"
#include "lights.h"
#include "sound.h"
#include "battery.h"
#include "lockservice_app.h"
#include "accelerometer.h"

// Global state variables
volatile SystemState_t currentState = STATE_CONNECTED_IDLE;
volatile SystemState_t previousState = STATE_CONNECTED_IDLE;
volatile uint8_t deviceState = 0;  // This is the settings byte (from iOS or local changes)
volatile uint8_t deviceInfo = 0;
volatile uint8_t deviceBattery = 100;

// Static variables for timing
static uint32_t stateEntryTime = 0;

// Helper macros to extract bits from settings byte
#define GET_ARMED_BIT(byte)       ((byte) & 0x01)
#define GET_ALARM_TYPE(byte)      (((byte) >> 1) & 0x03)
#define GET_SENSITIVITY(byte)     (((byte) >> 3) & 0x03)
#define GET_LIGHTS_BIT(byte)      (((byte) >> 5) & 0x01)
#define GET_LOGGING_BIT(byte)     (((byte) >> 6) & 0x01)

// Helper macros to set bits in settings byte
#define SET_ARMED_BIT(byte, val)       do { if(val) (byte) |= 0x01; else (byte) &= ~0x01; } while(0)
#define SET_ALARM_TYPE(byte, val)      do { (byte) = ((byte) & ~0x06) | (((val) & 0x03) << 1); } while(0)
#define SET_SENSITIVITY(byte, val)     do { (byte) = ((byte) & ~0x18) | (((val) & 0x03) << 3); } while(0)
#define SET_LIGHTS_BIT(byte, val)      do { if(val) (byte) |= 0x20; else (byte) &= ~0x20; } while(0)
#define SET_LOGGING_BIT(byte, val)     do { if(val) (byte) |= 0x40; else (byte) &= ~0x40; } while(0)

// Alarm type constants (matches iOS AlarmType enum)
#define ALARM_NONE        0x00
#define ALARM_CALM        0x01
#define ALARM_NORMAL      0x02
#define ALARM_LOUD        0x03

// Sensitivity constants (matches iOS SensitivityLevel enum)
#define SENSITIVITY_LOW    0x00
#define SENSITIVITY_MEDIUM 0x01
#define SENSITIVITY_HIGH   0x02

void StateMachine_Init(void)
{
    currentState = STATE_CONNECTED_IDLE;
    stateEntryTime = HAL_GetTick();

    // Initialize deviceState with default settings
    // Armed=0, Alarm=Normal(0b10), Sensitivity=Medium(0b01), Lights=On, Logging=Off
    deviceState = 0;
    SET_ARMED_BIT(deviceState, 0);           // Bit 0: Not armed
    SET_ALARM_TYPE(deviceState, ALARM_NORMAL); // Bits 1-2: Normal alarm
    SET_SENSITIVITY(deviceState, SENSITIVITY_MEDIUM); // Bits 3-4: Medium sensitivity
    SET_LIGHTS_BIT(deviceState, 1);          // Bit 5: Lights on
    SET_LOGGING_BIT(deviceState, 0);         // Bit 6: Logging off

    // Initialize deviceInfo (for future use)
    deviceInfo = 0;
}

/***************************************************************************
 * No Bluetooth connection. Awaiting connection or it will go back to sleep.
 ***************************************************************************/
void State_Disconnected_Idle_Loop(){
    // In disconnected state, just maintain current settings
    // When connection happens, we'll send deviceState to the app
}

/***************************************************************************
 * Connected via Bluetooth to a device. Awaiting commands, after some time
 * -out device will go back to sleep.
 ***************************************************************************/
void State_Connected_Idle_Loop(){
    // Check if armed bit is set
    if (GET_ARMED_BIT(deviceState)) {
        // Armed - transition to ARMED state
        StateMachine_ChangeState(STATE_ARMED);
    } else {
        // Not armed - turn off LED if lights are enabled
        if (GET_LIGHTS_BIT(deviceState)) {
            turnOffLED();
        }
    }

    // eventually time out and go to sleep after n seconds
}

/***************************************************************************
 * Device is Armed. Movement should trigger the alarm unless it is disabled.
 * Motion events should be logged in the alarm state
 ***************************************************************************/
void State_Armed_Loop(){
    // Check if iOS sent a disarm command
    if (!GET_ARMED_BIT(deviceState)) {
        // Disarmed - go back to idle
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        return;
    }

    // Show armed LED if lights are enabled
    if (GET_LIGHTS_BIT(deviceState)) {
        testLED(10);
    } else {
        turnOffLED();
    }

    // Check for motion detection using accelerometer
    if (LIS2DUX12_IsMotionDetected()) {
        // Log motion event if logging is enabled
        if (GET_LOGGING_BIT(deviceState)) {
            // TODO: Log to EEPROM with timestamp
        }

        // Trigger alarm
        StateMachine_ChangeState(STATE_ALARM_ACTIVE);
        return;
    }

    // eventually time out and go to sleep after n seconds
}

/***************************************************************************
 * When entering this loop, the device should go to sleep. Before it sleeps,
 * it must set up the proper wake-ups depending on the device state. When
 * exiting loop, should transition to last state??
 ***************************************************************************/
void State_Sleep_Loop(){
    // Get ready to go to sleep
    // Sleep
    // Re-init everything to be awake
}

/***************************************************************************
 * Motion has been detected when the device was armed. Alarm sound depends
 * on device state, no matter what, log data to EEPROM and/or send over
 * Bluetooth to device
 ***************************************************************************/
void State_Alarm_Active_Loop(){
    // Check if user disarmed during alarm
    if (!GET_ARMED_BIT(deviceState)) {
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        return;
    }

    // Play alarm based on alarm type setting
    uint8_t alarmType = GET_ALARM_TYPE(deviceState);

    switch(alarmType) {
        case ALARM_NONE:
            // No alarm sound, just log
            break;

        case ALARM_CALM:
            // Play calm alarm (non-blocking)
            // TODO: Implement calm alarm sound
        	SOUND_CalmAlarm();
            break;

        case ALARM_NORMAL:
            // Play normal alarm (non-blocking)
            // TODO: Implement normal alarm sound
        	SOUND_NormalAlarm();
            break;

        case ALARM_LOUD:
            // Play loud alarm (non-blocking)
            // TODO: Implement loud alarm sound
        	SOUND_LoudAlarm();
            break;

        default:
            break;
    }

    // Show alarm LED if lights are enabled
    if (GET_LIGHTS_BIT(deviceState)) {
        // Flash LED aggressively
        // TODO: Implement alarm LED pattern
    }

    // Log motion event if logging is enabled
    if (GET_LOGGING_BIT(deviceState)) {
        // TODO: Log alarm event to EEPROM if not connected to phone
    }

    // After alarm plays, go back to armed state
    StateMachine_ChangeState(STATE_ARMED);
}

void StateMachine_ChangeState(SystemState_t newState)
{
    if(newState != currentState) {
        previousState = currentState;
        currentState = newState;
        stateEntryTime = HAL_GetTick();

        // Update armed bit in deviceState based on new state
        if (newState == STATE_ARMED || newState == STATE_ALARM_ACTIVE) {
            SET_ARMED_BIT(deviceState, 1);
        } else {
            SET_ARMED_BIT(deviceState, 0);
        }

        // Send notification when state changes
        LOCKSERVICE_SendStatusUpdate();
    }
}

void StateMachine_Run(void)
{
	// Track previous battery state (add this as a static variable or global)
	static uint8_t previousBattery = 0xFF;  // Initialize to invalid value to force first update

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0) {
		Battery_IsCharging();
	    deviceBattery |= 0b10000000;  // Set bit 7
	    testLED(50);
	} else {
	    deviceBattery &= ~0b10000000;  // Clear bit 7
	}

	// Only send update if battery state changed
	if (deviceBattery != previousBattery) {
	    LOCKSERVICE_SendStatusUpdate();
	    previousBattery = deviceBattery;
	}

    switch(currentState)
    {
        case STATE_DISCONNECTED_IDLE:
            State_Disconnected_Idle_Loop();
            break;

        case STATE_CONNECTED_IDLE:
            State_Connected_Idle_Loop();
            break;

        case STATE_ARMED:
            State_Armed_Loop();
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
