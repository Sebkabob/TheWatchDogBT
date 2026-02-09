#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "main.h"

// State definitions
typedef enum {
    STATE_SLEEP,
    STATE_ALARM_ACTIVE,
    STATE_LOCKED,
    STATE_DISCONNECTED_IDLE,
    STATE_CONNECTED_IDLE
} SystemState_t;

// Battery bit macros
#define BATTERY_CHARGING_BIT    7
#define BATTERY_CHARGING_MASK   (1 << BATTERY_CHARGING_BIT)  // 0b10000000
#define IS_CHARGING(pin_state)  ((pin_state) == GPIO_PIN_RESET)
#define SET_BATTERY_CHARGING(status)    ((status) |= BATTERY_CHARGING_MASK)
#define CLEAR_BATTERY_CHARGING(status)  ((status) &= ~BATTERY_CHARGING_MASK)
#define IS_BATTERY_CHARGING(status)     ((status) & BATTERY_CHARGING_MASK)

// DeviceState bit field getters
#define GET_ARMED_BIT(byte)       ((byte) & 0x01)
#define GET_ALARM_TYPE(byte)      (((byte) >> 1) & 0x03)
#define GET_SENSITIVITY(byte)     (((byte) >> 3) & 0x03)
#define GET_LIGHTS_BIT(byte)      (((byte) >> 5) & 0x01)
#define GET_LOGGING_BIT(byte)     (((byte) >> 6) & 0x01)
#define GET_SILENCE_BIT(byte)     (((byte) >> 7) & 0x01)

// DeviceState bit field setters
#define SET_ARMED_BIT(byte, val)       do { if(val) (byte) |= 0x01; else (byte) &= ~0x01; } while(0)
#define SET_ALARM_TYPE(byte, val)      do { (byte) = ((byte) & ~0x06) | (((val) & 0x03) << 1); } while(0)
#define SET_SENSITIVITY(byte, val)     do { (byte) = ((byte) & ~0x18) | (((val) & 0x03) << 3); } while(0)
#define SET_LIGHTS_BIT(byte, val)      do { if(val) (byte) |= 0x20; else (byte) &= ~0x20; } while(0)
#define SET_LOGGING_BIT(byte, val)     do { if(val) (byte) |= 0x40; else (byte) &= ~0x40; } while(0)
#define SET_SILENCE_BIT(byte, val)     do { if(val) (byte) |= 0x80; else (byte) &= ~0x80; } while(0)

// Alarm type constants
#define ALARM_NONE        0x00
#define ALARM_CALM        0x01
#define ALARM_NORMAL      0x02
#define ALARM_LOUD        0x03

// Sensitivity constants
#define SENSITIVITY_LOW    0x00
#define SENSITIVITY_MEDIUM 0x01
#define SENSITIVITY_HIGH   0x02

// Global state variables
extern volatile SystemState_t currentState;
extern volatile SystemState_t previousState;
extern volatile uint8_t deviceState;
extern volatile uint8_t deviceInfo;
extern volatile uint8_t deviceBattery;

extern volatile uint8_t stayAwakeFlag;

void StateMachine_UpdateBLEActivity(void);

// Function prototypes
void StateMachine_Init(void);
void StateMachine_Run(void);
void StateMachine_ChangeState(SystemState_t newState);
void StateMachine_UpdateActivity(void);
void StateMachine_CheckInactivityTimeout(void);

#endif
