#include "battery.h"
#include "bq27427_reg.h"
#include <stdio.h>
#include "main.h"

// Battery configuration parameters
#define BATTERY_DESIGN_CAPACITY_MAH     300     // 300mAh battery
#define BATTERY_TERMINATE_VOLTAGE_MV    3400    // 3.40 (3400mV)
#define BATTERY_TAPER_CURRENT_MA        30      // 30mA (10% of capacity)

// Debug info structure
typedef struct {
    uint16_t device_type;
    uint16_t fw_version;
    uint16_t flags;
    uint16_t control_status;
    uint16_t voltage_mV;
    int16_t current_mA;
    uint16_t soc_percent;
    uint16_t design_capacity_mAh;
    uint16_t remaining_capacity_mAh;
} bq27427_debug_info_t;

bool BATTERY_TestCapacityRead(uint16_t *design_cap)
{
    // Try direct read from standard command
    if (!bq27427_i2c_command_read(BQ27427_DESIGN_CAP_LOW, design_cap)) {
        return false;
    }
    return true;
}

/**
 * @brief Initialize the BQ27427 fuel gauge
 * @return true if initialization successful, false otherwise
 */
bool BATTERY_Init(void)
{
    uint16_t device_type, design_cap, flags;

    if (!bq27427_readDeviceType(&device_type)) {
        return false;
    }

    if (device_type != 0x0427) {
        return false;
    }

    // Check if stuck in CONFIG UPDATE mode
    if (bq27427_readFlagsReg(&flags)) {
        if (flags & 0x0010) {
            bq27427_i2c_control_write(BQ27427_CONTROL_SOFT_RESET);
            HAL_Delay(2000);
            bq27427_readFlagsReg(&flags);
            if (flags & 0x0010) {
                return false;
            }
        }
    }

    // FORCE init to run at least once
    // (Remove this after confirming it works)
    if (!bq27427_init(BATTERY_DESIGN_CAPACITY_MAH,
                      BATTERY_TERMINATE_VOLTAGE_MV,
                      BATTERY_TAPER_CURRENT_MA))
    {
        return false;
    }

    // NOW try to read back
    HAL_Delay(500);  // Give it time to settle

    if (bq27427_readDesignCapacity_mAh(&design_cap)) {
        // Success - check if correct
        if (design_cap == BATTERY_DESIGN_CAPACITY_MAH) {
            return true;  // All good
        } else {
            return false;  // Written but wrong value
        }
    } else {
        return false;  // Still can't read
    }
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

/**
 * @brief Verify BQ27427 operation and read all status
 * @param info Pointer to debug info structure to populate
 * @return true if all reads successful, false otherwise
 */
bool BATTERY_VerifyOperation(bq27427_debug_info_t *info)
{
    // Read device identification
    if (!bq27427_readDeviceType(&info->device_type)) {
        return false;
    }

    if (!bq27427_readDeviceFWver(&info->fw_version)) {
        return false;
    }

    // Read status registers
    if (!bq27427_readFlagsReg(&info->flags)) {
        return false;
    }

    if (!bq27427_readControlReg(&info->control_status)) {
        return false;
    }

    // Read battery measurements
    if (!bq27427_readVoltage_mV(&info->voltage_mV)) {
        return false;
    }

    if (!bq27427_readAvgCurrent_mA(&info->current_mA)) {
        return false;
    }

    if (!bq27427_readStateofCharge_percent(&info->soc_percent)) {
        return false;
    }

    if (!bq27427_readDesignCapacity_mAh(&info->design_capacity_mAh)) {
        return false;
    }

    if (!bq27427_readRemainingCapacity_mAh(&info->remaining_capacity_mAh)) {
        return false;
    }

    return true;
}

/**
 * @brief Print detailed BQ27427 status (for debugging)
 * @param info Pointer to debug info structure
 */
void BATTERY_PrintStatus(bq27427_debug_info_t *info)
{
    printf("\n=== BQ27427 Status ===\n");
    printf("Device Type: 0x%04X (should be 0x0427)\n", info->device_type);
    printf("FW Version:  0x%04X\n", info->fw_version);

    printf("\nFlags Register: 0x%04X\n", info->flags);
    printf("  CFGUPMODE: %s\n", (info->flags & 0x0010) ? "SET (ERROR!)" : "Clear (OK)");
    printf("  ITPOR:     %s\n", (info->flags & 0x0004) ? "SET" : "Clear");
    printf("  BAT_DET:   %s\n", (info->flags & 0x0008) ? "Detected" : "Not Detected");
    printf("  FC:        %s\n", (info->flags & 0x0200) ? "Full" : "Not Full");
    printf("  DSG:       %s\n", (info->flags & 0x0001) ? "Discharging" : "Not Discharging");

    printf("\nControl Status: 0x%04X\n", info->control_status);
    printf("  INITCOMP:  %s\n", (info->control_status & 0x0080) ? "Complete (OK)" : "NOT Complete (ERROR!)");

    printf("\nBattery Measurements:\n");
    printf("  Voltage:            %u mV\n", info->voltage_mV);
    printf("  Current:            %d mA\n", info->current_mA);
    printf("  State of Charge:    %u %%\n", info->soc_percent);
    printf("  Design Capacity:    %u mAh\n", info->design_capacity_mAh);
    printf("  Remaining Capacity: %u mAh\n", info->remaining_capacity_mAh);

    // Overall health check
    printf("\n=== Health Check ===\n");
    bool healthy = true;

    if (info->device_type != 0x0427) {
        printf("ERROR: Wrong device type!\n");
        healthy = false;
    }

    if (info->flags & 0x0010) {
        printf("ERROR: Still in CONFIG UPDATE mode!\n");
        healthy = false;
    }

    if (!(info->control_status & 0x0080)) {
        printf("ERROR: Initialization not complete!\n");
        healthy = false;
    }

    if (!(info->flags & 0x0008)) {
        printf("WARNING: Battery not detected\n");
    }

    if (info->voltage_mV < 2500) {
        printf("WARNING: Battery voltage very low (< 2.5V)\n");
    }

    if (healthy) {
        printf("SUCCESS: BQ27427 operating normally!\n");
    } else {
        printf("ERROR: BQ27427 has errors - check above\n");
    }
    printf("\n");
}

/**
 * @brief Run BQ27427 self-test and print results
 * @return true if gauge is operating normally, false if errors detected
 * @note This function uses printf for debugging output
 */
bool BATTERY_SelfTest(void)
{
    bq27427_debug_info_t info;

    if (!BATTERY_VerifyOperation(&info)) {
        return false;
    }

    BATTERY_PrintStatus(&info);

    // Return true only if all critical checks pass
    bool healthy = true;

    if (info.device_type != 0x0427) {
        healthy = false;
    }

    if (info.flags & 0x0010) {  // CFGUPMODE still set
        healthy = false;
    }

    if (!(info.control_status & 0x0080)) {  // INITCOMP not set
        healthy = false;
    }

    return healthy;
}

/**
 * @brief Estimate SOC percentage from battery voltage (LiPo curve)
 * @param voltage_mV Battery voltage in millivolts
 * @return Estimated SOC in percent (0-100)
 * @note This is an approximation based on typical LiPo discharge curve
 */
static uint8_t BATTERY_EstimateSOC_FromVoltage(uint16_t voltage_mV)
{
    // LiPo voltage to SOC lookup table (approximate)
    // Based on typical single-cell LiPo discharge curve

    if (voltage_mV >= 4200) {
        return 100;  // 4.2V = 100% (fully charged)
    } else if (voltage_mV >= 4100) {
        return 90;   // 4.1V = ~90%
    } else if (voltage_mV >= 4000) {
        return 80;   // 4.0V = ~80%
    } else if (voltage_mV >= 3950) {
        return 75;   // 3.95V = ~75%
    } else if (voltage_mV >= 3900) {
        return 70;   // 3.9V = ~70%
    } else if (voltage_mV >= 3850) {
        return 65;   // 3.85V = ~65%
    } else if (voltage_mV >= 3800) {
        return 60;   // 3.8V = ~60%
    } else if (voltage_mV >= 3750) {
        return 55;   // 3.75V = ~55%
    } else if (voltage_mV >= 3700) {
        return 50;   // 3.7V = ~50% (nominal voltage)
    } else if (voltage_mV >= 3650) {
        return 40;   // 3.65V = ~40%
    } else if (voltage_mV >= 3600) {
        return 30;   // 3.6V = ~30%
    } else if (voltage_mV >= 3500) {
        return 20;   // 3.5V = ~20%
    } else if (voltage_mV >= 3400) {
        return 10;   // 3.4V = ~10%
    } else if (voltage_mV >= 3300) {
        return 5;    // 3.3V = ~5% (low battery warning)
    } else if (voltage_mV >= 3200) {
        return 2;    // 3.2V = ~2% (critical)
    } else {
        return 1;    // Below 3.2V = ~1% (cutoff imminent)
    }
}

/**
 * @brief Get quick status check (no printf)
 * @param voltage_mV Output: battery voltage in mV
 * @param soc_percent Output: state of charge in percent
 * @param is_charging Output: true if battery is charging
 * @return true if read successful, false otherwise
 */
bool BATTERY_GetStatus(uint16_t *voltage_mV, uint16_t *soc_percent, bool *is_charging)
{
    int16_t current_mA;

    if (!bq27427_readVoltage_mV(voltage_mV)) {
        return false;
    }

    if (!bq27427_readStateofCharge_percent(soc_percent)) {
        return false;
    }

    if (!bq27427_readAvgCurrent_mA(&current_mA)) {
        return false;
    }

    // If gauge is uncalibrated (SOC = 0), estimate from voltage
    if (*soc_percent == 0) {
        *soc_percent = BATTERY_EstimateSOC_FromVoltage(*voltage_mV);
    }

    // Negative current = charging (BQ27427 convention)
    *is_charging = (current_mA < 0);

    return true;
}

bool BATTERY_Charging(){
    int16_t current_mA;

    if (!bq27427_readAvgCurrent_mA(&current_mA)) {
        return false;
    }
    // Negative current = charging (BQ27427 convention)
     if (current_mA < 0){
    	 return true;
     } else if (current_mA >= 0){
    	 return false;
     }

}

/**
 * @brief Check if battery is critically low
 * @return true if battery is critically low, false otherwise
 */
bool BATTERY_IsCriticallyLow(void)
{
    uint16_t flags;

    if (!bq27427_readFlagsReg(&flags)) {
        return false;  // Can't read - assume not critical
    }

    // Check SOCF (State of Charge Final) bit - bit 1
    return (flags & 0x0002) != 0;
}

/**
 * @brief Check if battery is low
 * @return true if battery is low, false otherwise
 */
bool BATTERY_IsLow(void)
{
    uint16_t flags;

    if (!bq27427_readFlagsReg(&flags)) {
        return false;  // Can't read - assume not low
    }

    // Check SOC1 bit - bit 2
    return (flags & 0x0004) != 0;
}

/**
 * @brief Check if battery is fully charged
 * @return true if battery is full, false otherwise
 */
bool BATTERY_IsFull(void)
{
    uint16_t flags;

    if (!bq27427_readFlagsReg(&flags)) {
        return false;  // Can't read - assume not full
    }

    // Check FC (Full Charge) bit - bit 9
    return (flags & 0x0200) != 0;
}
