/*
 * wd_system.h
 *
 *  Created on: Oct 30, 2025
 *      Author: sebkabob
 */

#ifndef INC_WD_SYSTEM_H_
#define INC_WD_SYSTEM_H_

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * CONFIGURATION BYTE STRUCTURE (received via Bluetooth)
 * ============================================================================
 *
 * Bit allocation for single hex byte (0x00 - 0xFF):
 *
 * Bit 7: Armed/Disarmed
 * Bit 6: Alarm Type (0=Quiet, 1=Loud)
 * Bits 5-4: Sensitivity Level (00=Low, 01=Medium, 10=High, 11=Ultra)
 * Bit 3: Motion Logging Enabled
 * Bit 2: Reserved for future use
 * Bits 1-0: Reserved for future use
 */

// Configuration byte bit masks
#define CONFIG_ARMED_MASK           0x80  // Bit 7
#define CONFIG_ALARM_TYPE_MASK      0x40  // Bit 6
#define CONFIG_SENSITIVITY_MASK     0x30  // Bits 5-4
#define CONFIG_LOGGING_MASK         0x08  // Bit 3
#define CONFIG_RESERVED_MASK        0x07  // Bits 2-0

// Configuration byte bit positions
#define CONFIG_ARMED_BIT            7
#define CONFIG_ALARM_TYPE_BIT       6
#define CONFIG_SENSITIVITY_SHIFT    4
#define CONFIG_LOGGING_BIT          3

/* ============================================================================
 * ENUMERATION TYPES
 * ============================================================================ */

/**
 * @brief Device arming state
 */
typedef enum {
    STATE_DISARMED = 0,
    STATE_ARMED = 1
} ArmedState_t;

/**
 * @brief Alarm mode types
 */
typedef enum {
    ALARM_QUIET = 0,    // Bluetooth notification only
    ALARM_LOUD = 1      // Buzzers + Bluetooth notification
} AlarmType_t;

/**
 * @brief Motion sensitivity levels
 */
typedef enum {
    SENSITIVITY_LOW = 0,     // ~500 mg threshold
    SENSITIVITY_MEDIUM = 1,  // ~250 mg threshold
    SENSITIVITY_HIGH = 2,    // ~100 mg threshold
    SENSITIVITY_ULTRA = 3    // ~50 mg threshold
} SensitivityLevel_t;

/**
 * @brief Operational states for state machine
 */
typedef enum {
    OP_STATE_INIT,              // Initial power-on state
    OP_STATE_IDLE,              // Disarmed, no monitoring
    OP_STATE_MONITORING,        // Armed, monitoring for motion
    OP_STATE_ALARM_TRIGGERED,   // Alarm condition detected
    OP_STATE_ALARM_ACTIVE,      // Alarm sounding/notifying
    OP_STATE_COOLDOWN,          // Post-alarm cooldown period
    OP_STATE_ERROR              // Error condition
} OperationalState_t;

/**
 * @brief Alarm trigger reasons (for logging/notification)
 */
typedef enum {
    TRIGGER_NONE = 0,
    TRIGGER_MOTION_DETECTED,
    TRIGGER_TAMPER_DETECTED,
    TRIGGER_MANUAL_TEST
} TriggerReason_t;

/* ============================================================================
 * CONFIGURATION STRUCTURE
 * ============================================================================ */

/**
 * @brief Parsed configuration from Bluetooth command byte
 */
typedef struct {
    ArmedState_t armed;
    AlarmType_t alarm_type;
    SensitivityLevel_t sensitivity;
    bool logging_enabled;
} WatchDogConfig_t;

/**
 * @brief Runtime state information
 */
typedef struct {
    OperationalState_t current_state;
    OperationalState_t previous_state;
    TriggerReason_t trigger_reason;
    uint32_t state_entry_time;      // Timestamp when entered current state
    uint32_t alarm_start_time;      // Timestamp when alarm triggered
    uint16_t motion_event_count;    // Counter for logged events
    bool bluetooth_connected;
    bool accelerometer_fault;
} WatchDogRuntimeState_t;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

/**
 * @brief Parse configuration byte received via Bluetooth
 * @param config_byte Raw hex byte from Bluetooth
 * @param config Pointer to configuration structure to populate
 */
void WatchDog_ParseConfigByte(uint8_t config_byte, WatchDogConfig_t *config);

/**
 * @brief Apply new configuration to the system
 * @param config Pointer to new configuration
 * @return true if configuration valid and applied, false otherwise
 */
bool WatchDog_ApplyConfiguration(const WatchDogConfig_t *config);

/**
 * @brief Get acceleration threshold based on sensitivity setting
 * @param sensitivity Sensitivity level
 * @return Threshold value in milli-g (mg)
 */
uint16_t WatchDog_GetThreshold(SensitivityLevel_t sensitivity);

/**
 * @brief Initialize the state machine
 */
void WatchDog_StateMachine_Init(void);

/**
 * @brief Main state machine update function (call in main loop)
 */
void WatchDog_StateMachine_Update(void);

/**
 * @brief Force a state transition (for external events)
 * @param new_state Target state
 */
void WatchDog_StateMachine_SetState(OperationalState_t new_state);

/**
 * @brief Get current operational state
 * @return Current state
 */
OperationalState_t WatchDog_StateMachine_GetState(void);

/**
 * @brief Handle motion detection event
 * @param magnitude Motion magnitude in milli-g
 */
void WatchDog_HandleMotionEvent(uint16_t magnitude);

/**
 * @brief Trigger alarm with specified reason
 * @param reason Why the alarm was triggered
 */
void WatchDog_TriggerAlarm(TriggerReason_t reason);

/**
 * @brief Acknowledge and silence alarm
 */
void WatchDog_AcknowledgeAlarm(void);

/* ============================================================================
 * UTILITY MACROS
 * ============================================================================ */

// Extract individual fields from config byte
#define IS_ARMED(byte)          ((byte & CONFIG_ARMED_MASK) >> CONFIG_ARMED_BIT)
#define GET_ALARM_TYPE(byte)    ((byte & CONFIG_ALARM_TYPE_MASK) >> CONFIG_ALARM_TYPE_BIT)
#define GET_SENSITIVITY(byte)   ((byte & CONFIG_SENSITIVITY_MASK) >> CONFIG_SENSITIVITY_SHIFT)
#define IS_LOGGING_ENABLED(byte) ((byte & CONFIG_LOGGING_MASK) >> CONFIG_LOGGING_BIT)

// Build config byte from individual settings
#define BUILD_CONFIG_BYTE(armed, alarm, sens, log) \
    (((armed) << CONFIG_ARMED_BIT) | \
     ((alarm) << CONFIG_ALARM_TYPE_BIT) | \
     ((sens) << CONFIG_SENSITIVITY_SHIFT) | \
     ((log) << CONFIG_LOGGING_BIT))

#endif // WATCHDOG_CONFIG_H

#endif /* INC_WD_SYSTEM_H_ */
