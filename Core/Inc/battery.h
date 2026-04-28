#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>

// Initialization
bool BATTERY_Init(void);
bool BATTERY_TestCapacityRead(uint16_t *design_cap);

// NEW: Cached battery state functions (call BATTERY_UpdateState first)
bool BATTERY_UpdateState(void);      // Call once per second to update all values
uint16_t BATTERY_GetVoltage(void);   // Get cached voltage in mV
int16_t BATTERY_GetCurrent(void);    // Get cached current in mA
uint16_t BATTERY_GetSOC(void);       // Get cached state of charge %
bool BATTERY_IsCharging(void);       // Get cached charging status
bool BATTERY_IsFullCached(void);     // Get cached full battery status
bool BATTERY_IsLowCached(void);      // Get cached low battery status
bool BATTERY_IsCriticallyCached(void); // Get cached critical battery status

// Diagnostic / learning telemetry
uint8_t  BATTERY_GetSOC_Unfiltered(void);        // unfiltered % from gauge
uint16_t BATTERY_GetFlags(void);                 // last raw Flags()
uint16_t BATTERY_GetControlStatus(void);         // last raw CONTROL_STATUS
uint16_t BATTERY_GetRemainingCapacity(void);     // mAh
uint16_t BATTERY_GetFullChargeCapacity(void);    // mAh
int16_t  BATTERY_GetTemperature_0_1K(void);      // 0.1 K units
bool     BATTERY_IsQmaxLearned(void);            // CONTROL_STATUS bit 9
bool     BATTERY_IsResistanceLearned(void);      // CONTROL_STATUS bit 8
bool     BATTERY_IsVoltageOK(void);              // CONTROL_STATUS bit 1 (VOK)
bool     BATTERY_IsBatteryDetected(void);        // FLAG bit 3 (BAT_DET)
bool     BATTERY_IsOverTemp(void);               // FLAG bit 15 (OT)
bool     BATTERY_IsUnderTemp(void);              // FLAG bit 14 (UT)
bool     BATTERY_IsOcvTaken(void);               // FLAG bit 7
bool     BATTERY_IsItpor(void);                  // FLAG bit 5

// v3 BatteryDiagnostic — gauge config readback / power / calibration
uint16_t BATTERY_GetDesignCapacity(void);        // mAh, cached from gauge
uint16_t BATTERY_GetTerminateVoltage(void);      // mV, cached from gauge
uint16_t BATTERY_GetTaperRate(void);             // 0.1h units, cached from gauge
uint16_t BATTERY_GetOpConfig(void);              // raw OpConfig register (Subclass 64, off 0)
int16_t  BATTERY_GetAveragePower(void);          // mW, signed (refreshed every update)
int8_t   BATTERY_GetBoardOffset(void);           // counts, signed (Subclass 104, off 0)
uint8_t  BATTERY_GetDeadband(void);              // mA       (Subclass 107, off 1)

/**
 * @brief Re-read all "static" gauge-config values into the cache (Design Capacity,
 *        Terminate Voltage, Taper Rate, OpConfig, Board Offset, Deadband).
 *        Each call enters/exits CONFIG UPDATE mode several times — only invoke
 *        from BATTERY_Init() or after an explicit reconfigure window.
 */
void     BATTERY_RefreshConfigCache(void);

// --- Diagnostic instrumentation (BatteryDiagnostic v4) -----------------------
typedef struct __attribute__((packed)) {
    uint8_t  attempts_used;        // 1, 2, or 3 — which retry succeeded; 0 if none
    uint8_t  entered;              // 1 if any enter_config attempt succeeded
    uint8_t  exited;               // 1 if exit_config returned true
    uint16_t flags_in_cfgmode;     // raw flags read while supposedly in CFGUPMODE
    uint16_t design_cap_raw;       // value read from gauge in this session
    uint8_t  user_ctrl_at_entry;   // _user_config_control state at function start
} battery_refresh_diag_t;

_Static_assert(sizeof(battery_refresh_diag_t) == 8,
               "battery_refresh_diag_t must be exactly 8 bytes");

battery_refresh_diag_t BATTERY_GetRefreshDiag(void);
uint16_t               BATTERY_GetTestDesignCap(void);   // standalone read at end of Init
uint8_t                BATTERY_GetReconfigCount(void);    // # of times reconfigure ran
uint16_t               BATTERY_GetPreInitDesignCap(void); // design cap read BEFORE first refresh

// --- v5 init-failure tracker ------------------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t  init_fail_stage;          // see codes below; 0 = success / not yet
    uint8_t  init_completed;           // 1 if BATTERY_Init reached the end
    uint8_t  was_sealed;               // 1 if gauge was sealed at start of Init
    uint8_t  chem_id_fail_stage;       // bq27427_get_chem_id_fail_stage() snapshot
    uint16_t init_current_capacity;    // bq27427_capacity(DESIGN) at needs_config check
    uint16_t init_current_terminate_v; // bq27427_terminate_voltage() at same point
    uint16_t init_current_taper_rate;  // bq27427_taper_rate() at same point
    uint16_t init_current_opconfig;    // bq27427_op_config() at same point
    uint8_t  init_sleep_enabled;       // bit 5 of OpConfig
    uint8_t  init_itpor_flag;          // bq27427_itpor_flag()
} battery_init_diag_t;

_Static_assert(sizeof(battery_init_diag_t) == 14,
               "battery_init_diag_t must be exactly 14 bytes");

battery_init_diag_t BATTERY_GetInitDiag(void);

uint8_t BATTERY_GetTestDesignCapByte6(void);  // raw byte: bq27427_read_extended_data(STATE, 6)
uint8_t BATTERY_GetTestDesignCapByte7(void);  // raw byte: bq27427_read_extended_data(STATE, 7)

/**
 * @brief Increments on the very first line of BATTERY_Init(). If this stays 0
 *        on every BLE diag read, BATTERY_Init() is never being called and the
 *        BQ27427 telemetry pipeline is dead at its root.
 */
uint8_t BATTERY_GetInitCallCount(void);

/* init_fail_stage codes:
 *   0 = init completed successfully
 *   1 = bq27427_init() failed
 *   2 = device_type wrong
 *   3 = INITCOMP timeout
 *   4 = bq27427_set_chem_id failed (continued anyway in v5+)
 *   5 = enter_config failed in main reconfigure block
 *   6 = one of the set_* writes failed
 *   7 = exit_config failed
 */

// LEGACY: Direct I2C read functions (use cached versions above instead)
uint16_t BATTERY_SOC(void);
int16_t BATTERY_Current(void);
uint16_t BATTERY_Voltage(void);
bool BATTERY_Charging(void);
bool BATTERY_IsCriticallyLow(void);
bool BATTERY_IsLow(void);
bool BATTERY_IsFull(void);
bool BATTERY_GetStatus(uint16_t *voltage_mV, uint16_t *soc_percent, bool *is_charging);

// Debug functions
bool BATTERY_SelfTest(void);

#endif // BATTERY_H
