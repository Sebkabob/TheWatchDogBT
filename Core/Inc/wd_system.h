/***************************************************************************
 * wd_system.h
 * created by Sebastian Forenza 2026
 *
 * Draft system-level types and helpers — RESERVED for future use. Nothing
 * in the firmware currently includes this header; the live state-machine
 * lives in state_machine.{c,h}.
 *
 * Configuration byte (single hex byte over BLE) — DRAFT layout:
 *   bit 7    Armed/Disarmed
 *   bit 6    Alarm Type    (0=quiet, 1=loud)
 *   bits 5:4 Sensitivity   (00=low, 01=med, 10=high, 11=ultra)
 *   bit 3    Logging enable
 *   bits 2:0 Reserved
 ***************************************************************************/

#ifndef INC_WD_SYSTEM_H_
#define INC_WD_SYSTEM_H_

#include <stdint.h>
#include <stdbool.h>

#define CONFIG_ARMED_MASK           0x80  // bit 7
#define CONFIG_ALARM_TYPE_MASK      0x40  // bit 6
#define CONFIG_SENSITIVITY_MASK     0x30  // bits 5:4
#define CONFIG_LOGGING_MASK         0x08  // bit 3
#define CONFIG_RESERVED_MASK        0x07  // bits 2:0

#define CONFIG_ARMED_BIT            7
#define CONFIG_ALARM_TYPE_BIT       6
#define CONFIG_SENSITIVITY_SHIFT    4
#define CONFIG_LOGGING_BIT          3

typedef enum {
    STATE_DISARMED = 0,
    STATE_ARMED = 1
} ArmedState_t;

typedef enum {
    ALARM_QUIET = 0,    // BLE notification only
    ALARM_LOUD = 1      // buzzer + BLE notification
} AlarmType_t;

typedef enum {
    SENSITIVITY_LOW = 0,     // ~500 mg threshold
    SENSITIVITY_MEDIUM = 1,  // ~250 mg threshold
    SENSITIVITY_HIGH = 2,    // ~100 mg threshold
    SENSITIVITY_ULTRA = 3    // ~50 mg threshold
} SensitivityLevel_t;

typedef enum {
    OP_STATE_INIT,
    OP_STATE_IDLE,
    OP_STATE_MONITORING,
    OP_STATE_ALARM_TRIGGERED,
    OP_STATE_ALARM_ACTIVE,
    OP_STATE_COOLDOWN,
    OP_STATE_ERROR
} OperationalState_t;

typedef enum {
    TRIGGER_NONE = 0,
    TRIGGER_MOTION_DETECTED,
    TRIGGER_TAMPER_DETECTED,
    TRIGGER_MANUAL_TEST
} TriggerReason_t;

typedef struct {
    ArmedState_t armed;
    AlarmType_t alarm_type;
    SensitivityLevel_t sensitivity;
    bool logging_enabled;
} WatchDogConfig_t;

typedef struct {
    OperationalState_t current_state;
    OperationalState_t previous_state;
    TriggerReason_t trigger_reason;
    uint32_t state_entry_time;
    uint32_t alarm_start_time;
    uint16_t motion_event_count;
    bool bluetooth_connected;
    bool accelerometer_fault;
} WatchDogRuntimeState_t;

void WatchDog_ParseConfigByte(uint8_t config_byte, WatchDogConfig_t *config);
bool WatchDog_ApplyConfiguration(const WatchDogConfig_t *config);

// Acceleration threshold (milli-g) for the given sensitivity level.
uint16_t WatchDog_GetThreshold(SensitivityLevel_t sensitivity);

void WatchDog_StateMachine_Init(void);
void WatchDog_StateMachine_Update(void);
void WatchDog_StateMachine_SetState(OperationalState_t new_state);
OperationalState_t WatchDog_StateMachine_GetState(void);

void WatchDog_HandleMotionEvent(uint16_t magnitude);
void WatchDog_TriggerAlarm(TriggerReason_t reason);
void WatchDog_AcknowledgeAlarm(void);

#define IS_ARMED(byte)          ((byte & CONFIG_ARMED_MASK) >> CONFIG_ARMED_BIT)
#define GET_ALARM_TYPE(byte)    ((byte & CONFIG_ALARM_TYPE_MASK) >> CONFIG_ALARM_TYPE_BIT)
#define GET_SENSITIVITY(byte)   ((byte & CONFIG_SENSITIVITY_MASK) >> CONFIG_SENSITIVITY_SHIFT)
#define IS_LOGGING_ENABLED(byte) ((byte & CONFIG_LOGGING_MASK) >> CONFIG_LOGGING_BIT)

#define BUILD_CONFIG_BYTE(armed, alarm, sens, log) \
    (((armed) << CONFIG_ARMED_BIT) | \
     ((alarm) << CONFIG_ALARM_TYPE_BIT) | \
     ((sens) << CONFIG_SENSITIVITY_SHIFT) | \
     ((log) << CONFIG_LOGGING_BIT))

#endif // WATCHDOG_CONFIG_H

#endif /* INC_WD_SYSTEM_H_ */
