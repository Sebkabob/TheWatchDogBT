#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>
#include "bq25186_reg.h"  // Include the driver header

// Battery Management Functions
void Battery_Init(void);
void Battery_Update(void);

// Getters for battery status
float Battery_GetVoltage(void);
float Battery_GetCurrent(void);
uint8_t Battery_GetPercentage(void);
bool Battery_IsCharging(void);
bool Battery_IsFault(void);

// Wrapper functions - renamed to avoid conflicts with driver
void Battery_SetChargeCurrent(uint16_t current_mA);      // Changed from BQ25186_SetChargeCurrent
void Battery_EnableCharging(bool enable);
void Battery_SetBatteryVoltage(uint16_t voltage_mv);     // Changed from BQ25186_SetBatteryVoltage

// Get the handle for direct driver access if needed
BQ25186_Handle_t* Battery_GetHandle(void);

#endif // BATTERY_H
