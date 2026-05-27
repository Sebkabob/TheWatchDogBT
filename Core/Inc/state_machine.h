/***************************************************************************
 * state_machine.h
 * created by Sebastian Forenza 2026
 *
 * Public state-machine API + the deviceState / deviceInfo / deviceBattery
 * bit-field accessors used everywhere else in the firmware.
 *
 * deviceState bit layout:
 *   0    ARMED          0/1
 *   2:1  ALARM_TYPE     0=none 1=calm 2=normal 3=loud
 *   4:3  SENSITIVITY    0=low  1=medium 2=high
 *   5    LIGHTS         0/1
 *   6    LOGGING        0/1
 *   7    SILENCE        0/1
 *
 * deviceInfo bit 0 (HIGH_PERF): when set, BLE status updates run at 50 Hz.
 ***************************************************************************/

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "main.h"

typedef enum {
    STATE_ALARM_ACTIVE,
    STATE_LOCKED,
    STATE_STABILIZING,
    STATE_DISCONNECTED_IDLE,
    STATE_CONNECTED_IDLE
} SystemState_t;

#define BATTERY_CHARGING_BIT    7
#define BATTERY_CHARGING_MASK   (1 << BATTERY_CHARGING_BIT)
#define SET_BATTERY_CHARGING(status)    ((status) |= BATTERY_CHARGING_MASK)
#define CLEAR_BATTERY_CHARGING(status)  ((status) &= ~BATTERY_CHARGING_MASK)

#define GET_ARMED_BIT(byte)       ((byte) & 0x01)
#define GET_ALARM_TYPE(byte)      (((byte) >> 1) & 0x03)
#define GET_SENSITIVITY(byte)     (((byte) >> 3) & 0x03)
#define GET_LIGHTS_BIT(byte)      (((byte) >> 5) & 0x01)
#define GET_LOGGING_BIT(byte)     (((byte) >> 6) & 0x01)
#define GET_SILENCE_BIT(byte)     (((byte) >> 7) & 0x01)

#define SET_ARMED_BIT(byte, val)       do { if(val) (byte) |= 0x01; else (byte) &= ~0x01; } while(0)
#define SET_ALARM_TYPE(byte, val)      do { (byte) = ((byte) & ~0x06) | (((val) & 0x03) << 1); } while(0)
#define SET_SENSITIVITY(byte, val)     do { (byte) = ((byte) & ~0x18) | (((val) & 0x03) << 3); } while(0)
#define SET_LIGHTS_BIT(byte, val)      do { if(val) (byte) |= 0x20; else (byte) &= ~0x20; } while(0)
#define SET_LOGGING_BIT(byte, val)     do { if(val) (byte) |= 0x40; else (byte) &= ~0x40; } while(0)
#define SET_SILENCE_BIT(byte, val)     do { if(val) (byte) |= 0x80; else (byte) &= ~0x80; } while(0)

#define ALARM_NONE        0x00
#define ALARM_CALM        0x01
#define ALARM_NORMAL      0x02
#define ALARM_LOUD        0x03

#define SENSITIVITY_LOW    0x00
#define SENSITIVITY_MEDIUM 0x01
#define SENSITIVITY_HIGH   0x02

#define CABLE_UNPLUG_AWAKE_MS   5000

extern volatile SystemState_t currentState;
extern volatile SystemState_t previousState;
extern volatile uint8_t deviceState;
extern volatile uint8_t deviceInfo;
extern volatile uint8_t deviceBattery;

extern volatile uint8_t stayAwakeFlag;

// Suppress motion-triggered alarm transitions for the next <ms> ms. Used
// after BLE connect / RestoreAll so the UCF reload + user handling the
// device while pairing doesn't immediately re-trigger the alarm.
void StateMachine_StartMotionGrace(uint32_t ms);

void StateMachine_Init(void);
void StateMachine_Run(void);
void StateMachine_ChangeState(SystemState_t newState);

// EEPROM-backed mirror of deviceState (alarm type / sensitivity / lights /
// logging / silence, AND the ARMED bit) and deviceInfo bit 0 HIGH_PERF.
// Init reads from EEPROM and applies on top of the StateMachine_Init
// defaults; Persist is called from the iOS settings dispatcher after each
// settings write.
//
// ARMED persistence: as of the central-EEPROM-map commit, ARMED is
// persisted. A device that browns out while locked comes back up locked;
// see the long policy block above the EEPROM record in state_machine.c.
void DeviceSettings_Init(void);
void DeviceSettings_Persist(void);

// Called from main() after DeviceSettings_Init and the rest of system
// init have run. If the persisted ARMED bit is set, transitions
// currentState directly to STATE_LOCKED (skipping STABILIZING, which is
// reserved for user-initiated arm). Safe to call when ARMED is clear —
// it just returns. Intended to be a one-shot at boot.
void StateMachine_RestoreArmedFromEEPROM(void);

// Called from GPIOB ISR when PB4 (BQ251_PG) fires. Safe from interrupt context.
void CablePlug_IRQCallback(void);

void FindMyDevice_Start(void);
void FindMyDevice_Update(void);

#endif
