#ifndef __BATTERY_H
#define __BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the BQ27427 fuel gauge
 * @return true if initialization successful, false otherwise
 */
bool BATTERY_Init(void);

/**
 * @brief Get the current State of Charge (SOC)
 * @return State of charge in percent (0-100), or 0 if read fails
 */
uint16_t BATTERY_SOC(void);

/**
 * @brief Get the instantaneous current draw
 * @return Current in mA (positive = charging, negative = discharging), or 0 if read fails
 */
int16_t BATTERY_Current(void);

/**
 * @brief Get the battery voltage
 * @return Voltage in mV, or 0 if read fails
 */
uint16_t BATTERY_Voltage(void);

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_H */
