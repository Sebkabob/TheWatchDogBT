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

    // Diagnostic / learning telemetry
    uint8_t  soc_unfiltered;
    uint16_t flags_raw;
    uint16_t control_status_raw;
    uint16_t remaining_mAh;
    uint16_t full_charge_mAh;
    int16_t  temperature_0_1K;
    bool qmax_updated;       // CONTROL_STATUS bit 9
    bool res_updated;        // CONTROL_STATUS bit 8
    bool voltage_ok;         // CONTROL_STATUS bit 1 (VOK)
    bool over_temp;          // FLAG bit 15
    bool under_temp;         // FLAG bit 14
    bool ocv_taken;          // FLAG bit 7
    bool bat_detected;       // FLAG bit 3
    bool itpor;              // FLAG bit 5
} BatteryState_t;

static BatteryState_t battery_state = {0};
static uint16_t cached_design_capacity = 0;

// Forward declaration for static function
static uint8_t BATTERY_EstimateSOC_FromVoltage(uint16_t voltage_mV);

bool BATTERY_TestCapacityRead(uint16_t *design_cap)
{
    *design_cap = cached_design_capacity;
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

    // Wait for INITCOMP after power-on
    uint32_t init_start = HAL_GetTick();
    while (!(bq27427_status() & BQ27427_STATUS_INITCOMP)) {
        if ((HAL_GetTick() - init_start) >= 1500) {
            return false;
        }
        HAL_Delay(10);
    }

    // Notify gauge of battery presence if BAT_DET is clear
    if (!(bq27427_flags() & BQ27427_FLAG_BAT_DET)) {
        bq27427_execute_control_word(BQ27427_CONTROL_BAT_INSERT);
        HAL_Delay(100);
    }

    // Ensure correct chemistry (CHEM_B = 4.2V LiPo).
    // bq27427_set_chem_id() handles enter/exit_config internally.
    if (bq27427_chem_id() != BQ27427_CHEM_B) {
        if (!bq27427_set_chem_id(BQ27427_CHEM_B)) {
            return false;
        }
    }

    // Check if already configured correctly
    uint16_t current_capacity = bq27427_capacity(BQ27427_CAPACITY_DESIGN);
    uint16_t current_terminate_voltage = bq27427_terminate_voltage();
    uint16_t current_taper_rate = bq27427_taper_rate();

    bool needs_config = (current_capacity != 300) ||
                        (current_terminate_voltage != 3000) ||
                        (current_taper_rate != 100) ||
                        bq27427_itpor_flag();

    if (needs_config) {

        if (!bq27427_enter_config(true)) {
            return false;
        }

        if (!bq27427_set_current_polarity(0)) { // 0 = Positive current means battery is charging
            return false;
        }
        if (!bq27427_set_capacity(300)) {
            return false;
        }
        if (!bq27427_set_design_energy(1110)) {  // 300mAh * 3.7V
            return false;
        }
        if (!bq27427_set_terminate_voltage(3000)) {
            return false;
        }
        if (!bq27427_set_taper_rate(100)) {  // (300mAh / 30mA) * 10 = 100, CUTS OFF AT 26mA CHARGING
            return false;
        }

        if (!bq27427_exit_config(true)) {
            return false;
        }

        HAL_Delay(1000);
    }

    cached_design_capacity = bq27427_capacity(BQ27427_CAPACITY_DESIGN);

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

    if ((now - battery_state.last_update) < 1000) {
        return true;
    }

    uint16_t flags = bq27427_flags();

    // ITPOR set means gauge lost its config — reinitialize.
    if (flags & BQ27427_FLAG_ITPOR) {
        BATTERY_Init();
        battery_state.last_update = 0;
        return true;
    }

    uint16_t cs = bq27427_status();

    battery_state.voltage_mV = bq27427_voltage();
    battery_state.current_mA = bq27427_current(BQ27427_CURRENT_AVG);
    battery_state.soc_percent = bq27427_soc(BQ27427_SOC_FILTERED);
    battery_state.is_charging = (flags & BQ27427_FLAG_CHG) != 0;
    battery_state.is_full = (flags & BQ27427_FLAG_FC) != 0;
    battery_state.is_low = (flags & BQ27427_FLAG_SOC1) != 0;
    battery_state.is_critical = (flags & BQ27427_FLAG_SOCF) != 0;

    battery_state.soc_unfiltered = (uint8_t)(bq27427_soc(BQ27427_SOC_UNFILTERED) & 0xFF);
    battery_state.flags_raw = flags;
    battery_state.control_status_raw = cs;
    battery_state.remaining_mAh = bq27427_capacity(BQ27427_CAPACITY_REMAIN);
    battery_state.full_charge_mAh = bq27427_capacity(BQ27427_CAPACITY_FULL);
    battery_state.temperature_0_1K = (int16_t)bq27427_temperature();
    battery_state.qmax_updated = (cs & BQ27427_STATUS_QMAX_UP) != 0;
    battery_state.res_updated  = (cs & BQ27427_STATUS_RES_UP) != 0;
    battery_state.voltage_ok   = (cs & BQ27427_STATUS_VOK) != 0;
    battery_state.over_temp    = (flags & BQ27427_FLAG_OT) != 0;
    battery_state.under_temp   = (flags & BQ27427_FLAG_UT) != 0;
    battery_state.ocv_taken    = (flags & BQ27427_FLAG_OCVTAKEN) != 0;
    battery_state.bat_detected = (flags & BQ27427_FLAG_BAT_DET) != 0;
    battery_state.itpor        = (flags & BQ27427_FLAG_ITPOR) != 0;

    // If fuel gauge reports Full Charge (FC flag), ensure SOC shows 100%
    if (battery_state.is_full && battery_state.soc_percent < 100) {
        battery_state.soc_percent = 100;
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

uint8_t  BATTERY_GetSOC_Unfiltered(void)     { return battery_state.soc_unfiltered; }
uint16_t BATTERY_GetFlags(void)              { return battery_state.flags_raw; }
uint16_t BATTERY_GetControlStatus(void)      { return battery_state.control_status_raw; }
uint16_t BATTERY_GetRemainingCapacity(void)  { return battery_state.remaining_mAh; }
uint16_t BATTERY_GetFullChargeCapacity(void) { return battery_state.full_charge_mAh; }
int16_t  BATTERY_GetTemperature_0_1K(void)   { return battery_state.temperature_0_1K; }
bool     BATTERY_IsQmaxLearned(void)         { return battery_state.qmax_updated; }
bool     BATTERY_IsResistanceLearned(void)   { return battery_state.res_updated; }
bool     BATTERY_IsVoltageOK(void)           { return battery_state.voltage_ok; }
bool     BATTERY_IsBatteryDetected(void)     { return battery_state.bat_detected; }
bool     BATTERY_IsOverTemp(void)            { return battery_state.over_temp; }
bool     BATTERY_IsUnderTemp(void)           { return battery_state.under_temp; }
bool     BATTERY_IsOcvTaken(void)            { return battery_state.ocv_taken; }
bool     BATTERY_IsItpor(void)               { return battery_state.itpor; }

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
    info->design_capacity_mAh = cached_design_capacity;
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
