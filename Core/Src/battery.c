#include "battery.h"
#include "bq27427_reg.h"
#include <stdio.h>
#include "main.h"

// Debug info structure
typedef struct {
    uint16_t device_type;
    uint16_t flags;
    uint16_t control_status;
    uint16_t voltage_mV;
    int16_t current_mA;
    uint16_t soc_percent;
    uint16_t design_capacity_mAh;
    uint16_t remaining_capacity_mAh;
} bq27427_debug_info_t;

// Global battery state
typedef struct {
    uint16_t voltage_mV;
    int16_t current_mA;
    uint16_t soc_percent;
    bool is_charging;
    bool is_full;
    bool is_low;
    bool is_critical;
    uint32_t last_update;
} BatteryState_t;

static BatteryState_t battery_state = {0};

// Forward declaration for static function
static uint8_t BATTERY_EstimateSOC_FromVoltage(uint16_t voltage_mV);

bool BATTERY_TestCapacityRead(uint16_t *design_cap)
{
    *design_cap = bq27427_capacity(BQ27427_CAPACITY_DESIGN);
    return (*design_cap != 0);
}

/**
 * @brief Initialize the BQ27427 fuel gauge
 * @return true if initialization successful, false otherwise
 */
bool BATTERY_Init(void)
{
    if (!bq27427_init()) {
        return false;
    }

    uint16_t device_type = bq27427_device_type();
    if (device_type != 0x0427) {
        return false;
    }

    // Check if already configured correctly
    uint16_t current_capacity = bq27427_capacity(BQ27427_CAPACITY_DESIGN);
    uint16_t current_terminate_voltage = bq27427_terminate_voltage();
    uint16_t current_taper_rate = bq27427_taper_rate();

    bool needs_config = (current_capacity != 300) ||
                        (current_terminate_voltage != 3500) ||
                        (current_taper_rate != 100);

    if (needs_config) {
        // Reset to clear any bad state
        bq27427_reset();
        HAL_Delay(2000);

        // Configure battery - enter config once, set everything, exit
        if (!bq27427_enter_config(true)) {
            return false;
        }

        // Polarity bit = 1 means negative is charging
        // Polarity bit = 0 means positive is charging
        bq27427_set_current_polarity(0);

        bq27427_set_capacity(300);
        bq27427_set_terminate_voltage(3500);
        bq27427_set_taper_rate(100);  // (300mAh / 30mA) * 10 = 100, CUTS OFF AT 26mA CHARGING

        if (!bq27427_exit_config(true)) {
            return false;
        }

        HAL_Delay(500);
    }

    // Initialize battery state
    battery_state.last_update = 0;

    return true;
}

/**
 * @brief Update all battery parameters (call once per second max)
 * @return true if update successful
 */
bool BATTERY_UpdateState(void)
{
    uint32_t now = HAL_GetTick();

    // Rate limit to prevent I2C spam (minimum 1 second between updates)
    if ((now - battery_state.last_update) < 500) {
        return true;  // Use cached values
    }

    // Read all battery parameters in one burst
    battery_state.voltage_mV = bq27427_voltage();
    battery_state.current_mA = bq27427_current(BQ27427_CURRENT_AVG);
    battery_state.soc_percent = bq27427_soc(BQ27427_SOC_FILTERED);
    battery_state.is_charging = bq27427_chg_flag();
    battery_state.is_full = bq27427_fc_flag();
    battery_state.is_low = bq27427_soc_flag();
    battery_state.is_critical = bq27427_socf_flag();

    // If gauge uncalibrated (SOC = 0), estimate from voltage
    if (battery_state.soc_percent == 0 && battery_state.voltage_mV > 0) {
        //battery_state.soc_percent = BATTERY_EstimateSOC_FromVoltage(battery_state.voltage_mV);
        battery_state.soc_percent = 127;

    }

    battery_state.last_update = now;
    return true;
}

/**
 * @brief Get cached voltage (call BATTERY_UpdateState first)
 */
uint16_t BATTERY_GetVoltage(void)
{
    return battery_state.voltage_mV;
}

/**
 * @brief Get cached current (call BATTERY_UpdateState first)
 */
int16_t BATTERY_GetCurrent(void)
{
    return battery_state.current_mA;
}

/**
 * @brief Get cached SOC (call BATTERY_UpdateState first)
 */
uint16_t BATTERY_GetSOC(void)
{
    return battery_state.soc_percent;
}

/**
 * @brief Get cached charging status (call BATTERY_UpdateState first)
 */
bool BATTERY_IsCharging(void)
{
    return battery_state.is_charging;
}

/**
 * @brief Get cached full status (call BATTERY_UpdateState first)
 */
bool BATTERY_IsFullCached(void)
{
    return battery_state.is_full;
}

/**
 * @brief Get cached low battery status (call BATTERY_UpdateState first)
 */
bool BATTERY_IsLowCached(void)
{
    return battery_state.is_low;
}

/**
 * @brief Get cached critical battery status (call BATTERY_UpdateState first)
 */
bool BATTERY_IsCriticallyCached(void)
{
    return battery_state.is_critical;
}

/**
 * @brief Get the current State of Charge (SOC) - LEGACY, use BATTERY_GetSOC instead
 * @return State of charge in percent (0-100), or 0 if read fails
 */
uint16_t BATTERY_SOC(void)
{
    return bq27427_soc(BQ27427_SOC_FILTERED);
}

/**
 * @brief Get the instantaneous current draw - LEGACY, use BATTERY_GetCurrent instead
 * @return Current in mA (positive = charging, negative = discharging), or 0 if read fails
 */
int16_t BATTERY_Current(void)
{
    return bq27427_current(BQ27427_CURRENT_AVG);
}

/**
 * @brief Get the battery voltage - LEGACY, use BATTERY_GetVoltage instead
 * @return Voltage in mV, or 0 if read fails
 */
uint16_t BATTERY_Voltage(void)
{
    return bq27427_voltage();
}

/**
 * @brief Verify BQ27427 operation and read all status
 * @param info Pointer to debug info structure to populate
 * @return true if all reads successful, false otherwise
 */
bool BATTERY_VerifyOperation(bq27427_debug_info_t *info)
{
    // Read device identification
    info->device_type = bq27427_device_type();
    if (info->device_type == 0) {
        return false;
    }

    // Read status registers
    info->flags = bq27427_flags();
    info->control_status = bq27427_status();

    // Read battery measurements
    info->voltage_mV = bq27427_voltage();
    info->current_mA = bq27427_current(BQ27427_CURRENT_AVG);
    info->soc_percent = bq27427_soc(BQ27427_SOC_FILTERED);
    info->design_capacity_mAh = bq27427_capacity(BQ27427_CAPACITY_DESIGN);
    info->remaining_capacity_mAh = bq27427_capacity(BQ27427_CAPACITY_REMAIN);

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

    printf("\nFlags Register: 0x%04X\n", info->flags);
    printf("  CFGUPMODE: %s\n", (info->flags & BQ27427_FLAG_CFGUPMODE) ? "SET (ERROR!)" : "Clear (OK)");
    printf("  ITPOR:     %s\n", (info->flags & BQ27427_FLAG_ITPOR) ? "SET" : "Clear");
    printf("  BAT_DET:   %s\n", (info->flags & BQ27427_FLAG_BAT_DET) ? "Detected" : "Not Detected");
    printf("  FC:        %s\n", (info->flags & BQ27427_FLAG_FC) ? "Full" : "Not Full");
    printf("  DSG:       %s\n", (info->flags & BQ27427_FLAG_DSG) ? "Discharging" : "Not Discharging");

    printf("\nControl Status: 0x%04X\n", info->control_status);
    printf("  INITCOMP:  %s\n", (info->control_status & BQ27427_STATUS_INITCOMP) ? "Complete (OK)" : "NOT Complete (ERROR!)");

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

    if (info->flags & BQ27427_FLAG_CFGUPMODE) {
        printf("ERROR: Still in CONFIG UPDATE mode!\n");
        healthy = false;
    }

    if (!(info->control_status & BQ27427_STATUS_INITCOMP)) {
        printf("ERROR: Initialization not complete!\n");
        healthy = false;
    }

    if (!(info->flags & BQ27427_FLAG_BAT_DET)) {
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

    if (info.flags & BQ27427_FLAG_CFGUPMODE) {
        healthy = false;
    }

    if (!(info.control_status & BQ27427_STATUS_INITCOMP)) {
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
    if (voltage_mV >= 4200) {
        return 100;
    } else if (voltage_mV >= 4100) {
        return 90;
    } else if (voltage_mV >= 4000) {
        return 80;
    } else if (voltage_mV >= 3950) {
        return 75;
    } else if (voltage_mV >= 3900) {
        return 70;
    } else if (voltage_mV >= 3850) {
        return 65;
    } else if (voltage_mV >= 3800) {
        return 60;
    } else if (voltage_mV >= 3750) {
        return 55;
    } else if (voltage_mV >= 3700) {
        return 50;
    } else if (voltage_mV >= 3650) {
        return 40;
    } else if (voltage_mV >= 3600) {
        return 30;
    } else if (voltage_mV >= 3500) {
        return 20;
    } else if (voltage_mV >= 3400) {
        return 10;
    } else if (voltage_mV >= 3300) {
        return 5;
    } else if (voltage_mV >= 3200) {
        return 2;
    } else {
        return 1;
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
    *voltage_mV = bq27427_voltage();
    *soc_percent = bq27427_soc(BQ27427_SOC_FILTERED);
    *is_charging = bq27427_chg_flag();

    // If gauge is uncalibrated (SOC = 0), estimate from voltage
    if (*soc_percent == 0 && *voltage_mV > 0) {
        *soc_percent = BATTERY_EstimateSOC_FromVoltage(*voltage_mV);
    }

    return (*voltage_mV > 0);
}

/**
 * @brief Check if battery is charging - LEGACY, use BATTERY_IsCharging instead
 * @return true if charging, false otherwise
 */
bool BATTERY_Charging(void)
{
    return bq27427_chg_flag();
}

/**
 * @brief Check if battery is critically low - LEGACY, use BATTERY_IsCriticallyCached instead
 * @return true if battery is critically low, false otherwise
 */
bool BATTERY_IsCriticallyLow(void)
{
    return bq27427_socf_flag();
}

/**
 * @brief Check if battery is low - LEGACY, use BATTERY_IsLowCached instead
 * @return true if battery is low, false otherwise
 */
bool BATTERY_IsLow(void)
{
    return bq27427_soc_flag();
}

/**
 * @brief Check if battery is fully charged - LEGACY, use BATTERY_IsFullCached instead
 * @return true if battery is full, false otherwise
 */
bool BATTERY_IsFull(void)
{
    return bq27427_fc_flag();
}
