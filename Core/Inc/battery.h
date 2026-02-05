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
