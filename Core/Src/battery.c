#include "battery.h"
#include "bq27427_reg.h"

// Battery configuration parameters
#define BATTERY_DESIGN_CAPACITY_MAH     300     // 300mAh battery
#define BATTERY_TERMINATE_VOLTAGE_MV    4200    // 4.2V (4200mV)
#define BATTERY_TAPER_CURRENT_MA        30      // 30mA (10% of capacity)

/**
 * @brief Initialize the BQ27427 fuel gauge
 * @return true if initialization successful, false otherwise
 */
bool BATTERY_Init(void)
{
    // Initialize the BQ27427 with battery parameters
    if (!bq27427_init(BATTERY_DESIGN_CAPACITY_MAH,
                      BATTERY_TERMINATE_VOLTAGE_MV,
                      BATTERY_TAPER_CURRENT_MA))
    {
        return false;
    }

    return true;
}

/**
 * @brief Get the current State of Charge (SOC)
 * @return State of charge in percent (0-100), or 0 if read fails
 */
uint16_t BATTERY_SOC(void)
{
    uint16_t soc = 0;

    if (!bq27427_readStateofCharge_percent(&soc))
    {
        return 0;
    }

    return soc;
}

/**
 * @brief Get the instantaneous current draw
 * @return Current in mA (positive = charging, negative = discharging), or 0 if read fails
 */
int16_t BATTERY_Current(void)
{
    int16_t current = 0;

    if (!bq27427_readAvgCurrent_mA(&current))
    {
        return 0;
    }

    return current;
}

/**
 * @brief Get the battery voltage
 * @return Voltage in mV, or 0 if read fails
 */
uint16_t BATTERY_Voltage(void)
{
    uint16_t voltage = 0;

    if (!bq27427_readVoltage_mV(&voltage))
    {
        return 0;
    }

    return voltage;
}
