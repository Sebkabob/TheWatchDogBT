/***************************************************************************
 * battery.c
 * created by Sebastian Forenza 2026
 *
 * Functions in charge of interfacing with the
 * BQ27427 Fuel Gauge IC and ____ Battery Charger IC
 ***************************************************************************/

#include "battery.h"
#include <stddef.h>
#include "stm32wb0x_hal.h"  // Adjust based on your HAL

// Global handle for the BQ25186
static BQ25186_Handle_t bq25186_handle;

// External variable for battery percentage
extern uint8_t deviceBattery;

// I2C wrapper functions for the driver
static int BQ25186_I2C_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    // Implement using your I2C peripheral (e.g., hi2c1)
    extern I2C_HandleTypeDef hi2c1;  // Declare extern or pass as parameter

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg_addr,
                                                  I2C_MEMADD_SIZE_8BIT, data, len, 100);
    return (status == HAL_OK) ? 0 : -1;
}

static int BQ25186_I2C_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    // Implement using your I2C peripheral
    extern I2C_HandleTypeDef hi2c1;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg_addr,
                                                 I2C_MEMADD_SIZE_8BIT, data, len, 100);
    return (status == HAL_OK) ? 0 : -1;
}

static void BQ25186_Delay_Ms(uint32_t ms) {
    HAL_Delay(ms);
}

void Battery_Init(void) {
    // Initialize the BQ25186 driver
    BQ25186_Status_t status = BQ25186_Init(&bq25186_handle,
                                            BQ25186_I2C_Write,
                                            BQ25186_I2C_Read,
                                            BQ25186_Delay_Ms);

    if (status != BQ25186_OK) {
        // Handle initialization error
        // You might want to set an error flag or retry
        return;
    }

    // Configure default charging parameters
    BQ25186_SetBatteryVoltage(&bq25186_handle, 4150);  // 4.15V
    BQ25186_SetChargeCurrent(&bq25186_handle, 300);     // 300mA
    BQ25186_SetInputCurrentLimit(&bq25186_handle, BQ25186_ILIM_300MA);
    BQ25186_SetWatchdog(&bq25186_handle, BQ25186_WATCHDOG_DISABLED); // No timeout for charging if no I2C activity
    BQ25186_SetChargeEnable(&bq25186_handle, true);

    // Set other parameters as needed
    BQ25186_SetTerminationCurrent(&bq25186_handle, BQ25186_ITERM_10_PERCENT);
    BQ25186_SetVINDPM(&bq25186_handle, BQ25186_VINDPM_4500MV);
    BQ25186_SetThermalRegulation(&bq25186_handle, BQ25186_TREG_100C);
}

void Battery_Update(void) {
    // Read status periodically
    BQ25186_Status_Regs_t status;
    BQ25186_GetStatus(&bq25186_handle, &status);

    // Clear any faults
    if (status.safety_timer_fault) {
        BQ25186_ClearFaults(&bq25186_handle);
    }

    // Reset watchdog timer
    BQ25186_ResetWatchdog(&bq25186_handle);
}

float Battery_GetCurrent(void) {
    uint16_t current_ma;
    BQ25186_Status_t status = BQ25186_GetChargeCurrent(&bq25186_handle, &current_ma);

    if (status == BQ25186_OK) {
        return current_ma / 1000.0f;  // Convert to amps
    }
    return 0.0f;
}

/**
 * @brief Convert battery voltage to State of Charge percentage
 * @param voltage_v Battery voltage in volts
 * @return SOC percentage (0-100)
 */

bool Battery_IsCharging(void) {
    BQ25186_Status_Regs_t status;
    BQ25186_GetStatus(&bq25186_handle, &status);

    return (status.chg_stat == BQ25186_CHG_STAT_CC_MODE ||
            status.chg_stat == BQ25186_CHG_STAT_CV_MODE);
}

bool Battery_IsFault(void) {
    BQ25186_Status_Regs_t status;
    BQ25186_GetStatus(&bq25186_handle, &status);

    return (status.vin_ovp || status.buvlo || status.safety_timer_fault);
}

void Battery_SetChargeCurrent(uint16_t current_mA) {
    BQ25186_SetChargeCurrent(&bq25186_handle, current_mA);
}

void Battery_EnableCharging(bool enable) {
    BQ25186_SetChargeEnable(&bq25186_handle, enable);
}

void Battery_SetBatteryVoltage(uint16_t voltage_mv) {
    BQ25186_SetBatteryVoltage(&bq25186_handle, voltage_mv);
}

BQ25186_Handle_t* Battery_GetHandle(void) {
    return &bq25186_handle;
}
