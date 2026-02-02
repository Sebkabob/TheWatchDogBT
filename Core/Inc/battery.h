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
 * @return Current in mA (negative = charging, positive = discharging), or 0 if read fails
 */
int16_t BATTERY_Current(void);

/**
 * @brief Get the battery voltage
 * @return Voltage in mV, or 0 if read fails
 */
uint16_t BATTERY_Voltage(void);

/**
 * @brief Run BQ27427 self-test and print detailed status
 * @return true if gauge is operating normally, false if errors detected
 * @note This function uses printf for debugging output
 */
bool BATTERY_SelfTest(void);

/**
 * @brief Get quick status check (no printf, suitable for production)
 * @param voltage_mV Output: battery voltage in mV
 * @param soc_percent Output: state of charge in percent (estimated from voltage if gauge uncalibrated)
 * @param is_charging Output: true if battery is charging (negative current)
 * @return true if read successful, false otherwise
 * @note If gauge SOC is 0 (uncalibrated), SOC will be estimated from voltage
 */
bool BATTERY_GetStatus(uint16_t *voltage_mV, uint16_t *soc_percent, bool *is_charging);

/**
 * @brief Returns true if charging, false if not
 */
bool BATTERY_Charging(void);


/**
 * @brief Check if battery is critically low
 * @return true if battery is critically low (SOCF flag set), false otherwise
 */
bool BATTERY_IsCriticallyLow(void);

/**
 * @brief Check if battery is low
 * @return true if battery is low (SOC1 flag set), false otherwise
 */
bool BATTERY_IsLow(void);

/**
 * @brief Check if battery is fully charged
 * @return true if battery is full (FC flag set), false otherwise
 */
bool BATTERY_IsFull(void);

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_H */
