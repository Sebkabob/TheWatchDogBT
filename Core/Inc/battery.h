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

// --- v9 Calibration subclass dump (temporary, one-shot diagnostic) ---------
// Returns the raw bytes captured from Subclass 104 (BQ27427_ID_CALIB_DATA),
// offsets 0..15. Filled by BATTERY_RefreshConfigCache. Layout per BQ27427 TRM:
//   0..3  CC Gain    (4-byte TI custom float)
//   4..7  CC Delta   (4-byte TI custom float)
//   8..9  CC Offset  (int16)
//   10..11 candidate Board Offset (silicon-rev dependent)
//   12..15 spare/other
const uint8_t *BATTERY_GetCalibBytes(void);

// v10 init-failure tracker (temporary). See init_fail_stage codes in battery.c.
uint8_t  BATTERY_GetInitFailStage(void);
uint8_t  BATTERY_GetInitCompleted(void);
uint8_t  BATTERY_GetPostResetFired(void);
uint16_t BATTERY_GetChemIdRead(void);

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
