/***************************************************************************
 * battery.h
 * created by Sebastian Forenza 2026
 *
 * BQ27427 fuel-gauge wrapper API. Cached state (voltage, current, SOC,
 * flags, learned bits, gauge config) is updated once per second by
 * BATTERY_UpdateState(); accessors read the cache so the state machine
 * never blocks on I2C.
 ***************************************************************************/

#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>

bool BATTERY_Init(void);

// Refresh the cache (rate-limited to once per second). Call from main loop.
bool BATTERY_UpdateState(void);

uint16_t BATTERY_GetVoltage(void);          // mV
int16_t  BATTERY_GetCurrent(void);          // mA, signed (charge positive)
uint16_t BATTERY_GetSOC(void);              // 0..100 %
bool     BATTERY_IsCharging(void);
bool     BATTERY_IsFullCached(void);
bool     BATTERY_IsLowCached(void);
bool     BATTERY_IsCriticallyCached(void);

uint8_t  BATTERY_GetSOC_Unfiltered(void);
uint16_t BATTERY_GetFlags(void);
uint16_t BATTERY_GetControlStatus(void);
uint16_t BATTERY_GetRemainingCapacity(void);    // mAh
uint16_t BATTERY_GetFullChargeCapacity(void);   // mAh
int16_t  BATTERY_GetTemperature_0_1K(void);     // 0.1 K units
bool     BATTERY_IsQmaxLearned(void);           // CONTROL_STATUS bit 9
bool     BATTERY_IsResistanceLearned(void);     // CONTROL_STATUS bit 8
bool     BATTERY_IsVoltageOK(void);             // CONTROL_STATUS bit 1
bool     BATTERY_IsBatteryDetected(void);       // FLAG bit 3
bool     BATTERY_IsOverTemp(void);              // FLAG bit 15
bool     BATTERY_IsUnderTemp(void);             // FLAG bit 14
bool     BATTERY_IsOcvTaken(void);              // FLAG bit 7
bool     BATTERY_IsItpor(void);                 // FLAG bit 5

uint16_t BATTERY_GetDesignCapacity(void);       // mAh
uint16_t BATTERY_GetTerminateVoltage(void);     // mV
uint16_t BATTERY_GetTaperRate(void);            // 0.1 h units
uint16_t BATTERY_GetOpConfig(void);             // raw OpConfig (Subclass 64, off 0)
int16_t  BATTERY_GetAveragePower(void);         // mW, signed
int8_t   BATTERY_GetBoardOffset(void);
uint8_t  BATTERY_GetDeadband(void);             // mA (Subclass 107, off 1)

// Re-read the static gauge-config snapshot (DesignCap, TermVolt, Taper,
// OpConfig, Deadband, Subclass-104 calibration dump). Each call enters/exits
// CONFIG UPDATE — only invoke from BATTERY_Init() or after a reconfigure.
void     BATTERY_RefreshConfigCache(void);

// Subclass 104 (Calibration) raw dump, offsets 0..15. Layout per BQ27427 TRM:
//   0..3   CC Gain    (4-byte TI custom float, factory trim)
//   4..7   CC Delta   (4-byte TI custom float)
//   8..9   CC Offset  (int16)
//   10..11 candidate Board Offset (silicon-rev dependent)
//   12..15 spare/other
const uint8_t *BATTERY_GetCalibBytes(void);

uint8_t  BATTERY_GetInitFailStage(void);  // 0 = success; see battery.c for codes
uint8_t  BATTERY_GetInitCompleted(void);
uint8_t  BATTERY_GetPostResetFired(void);
uint16_t BATTERY_GetChemIdRead(void);

#endif // BATTERY_H
