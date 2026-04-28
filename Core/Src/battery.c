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

    // v3 telemetry — config readback (refreshed only via BATTERY_RefreshConfigCache)
    uint16_t design_capacity_mAh;
    uint16_t terminate_voltage_mV;
    uint16_t taper_rate;
    uint16_t op_config_raw;
    int8_t   board_offset;
    uint8_t  deadband_mA;
    // dynamic — refreshed every BATTERY_UpdateState
    int16_t  average_power_mW;
} BatteryState_t;

static BatteryState_t battery_state = {0};
static uint16_t cached_design_capacity = 0;

// --- Diagnostic instrumentation (BatteryDiagnostic v4) ----------------------
static battery_refresh_diag_t s_refresh_diag = {0};
static uint16_t                s_test_design_cap = 0;
static uint8_t                 s_reconfig_count = 0;
static uint16_t                s_pre_init_design_cap = 0;

battery_refresh_diag_t BATTERY_GetRefreshDiag(void)  { return s_refresh_diag; }
uint16_t               BATTERY_GetTestDesignCap(void) { return s_test_design_cap; }
uint8_t                BATTERY_GetReconfigCount(void) { return s_reconfig_count; }
uint16_t               BATTERY_GetPreInitDesignCap(void) { return s_pre_init_design_cap; }

static battery_init_diag_t s_init_diag = {0};
battery_init_diag_t    BATTERY_GetInitDiag(void)    { return s_init_diag; }

static uint8_t s_test_design_cap_byte6 = 0;
static uint8_t s_test_design_cap_byte7 = 0;
uint8_t BATTERY_GetTestDesignCapByte6(void) { return s_test_design_cap_byte6; }
uint8_t BATTERY_GetTestDesignCapByte7(void) { return s_test_design_cap_byte7; }

// Incremented on first line of BATTERY_Init — proves the function is called.
static uint8_t s_init_call_count = 0;
uint8_t BATTERY_GetInitCallCount(void) { return s_init_call_count; }

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
    // First line: prove this function is reached at all (visible early in BLE diag).
    s_init_call_count++;

    // Reset init diag for this boot.
    s_init_diag = (battery_init_diag_t){0};

    if (!bq27427_init()) {
        s_init_diag.init_fail_stage = 1;
        return false;
    }

    uint16_t device_type = bq27427_device_type();
    if (device_type != 0x0427) {
        s_init_diag.init_fail_stage = 2;
        return false;
    }

    // Wait for INITCOMP after power-on
    uint32_t init_start = HAL_GetTick();
    while (!(bq27427_status() & BQ27427_STATUS_INITCOMP)) {
        if ((HAL_GetTick() - init_start) >= 1500) {
            s_init_diag.init_fail_stage = 3;
            return false;
        }
        HAL_Delay(10);
    }

    // Snapshot whether the gauge is sealed at boot — if unseal silently fails,
    // every subsequent SET_CFGUPDATE will fail too.
    s_init_diag.was_sealed = bq27427_is_user_config_active() ? 0 : 0;  // placeholder
    // Use status SS bit directly (CONTROL_STATUS bit 13).
    s_init_diag.was_sealed = (bq27427_status() & BQ27427_STATUS_SS) ? 1 : 0;

    // Extra settle window: data in the field shows CFGUPMODE never asserts when
    // SET_CFGUPDATE is issued too soon after INITCOMP. 250 ms is well below the
    // existing 1.5 s INITCOMP timeout and well below the 1 s post-config delay.
    HAL_Delay(250);

    // Notify gauge of battery presence if BAT_DET is clear
    if (!(bq27427_flags() & BQ27427_FLAG_BAT_DET)) {
        bq27427_execute_control_word(BQ27427_CONTROL_BAT_INSERT);
        HAL_Delay(100);
    }

    // DIAGNOSTIC OVERRIDE: chem_id read/write bypassed. We're trying to confirm
    // the reconfigure block can write to flash at all; chem_id has been the
    // suspected blocker. Re-enable once flash writes are proven working.
    // if (bq27427_chem_id() != BQ27427_CHEM_B) {
    //     if (!bq27427_set_chem_id(BQ27427_CHEM_B)) {
    //         s_init_diag.init_fail_stage = 4;
    //     }
    // }

    // Check if already configured correctly
    uint16_t current_capacity = bq27427_capacity(BQ27427_CAPACITY_DESIGN);
    // Snapshot the gauge's stored design capacity BEFORE we touch it — tells us
    // whether previous boots ever successfully wrote 300 mAh to flash.
    s_pre_init_design_cap = current_capacity;
    uint16_t current_terminate_voltage = bq27427_terminate_voltage();
    uint16_t current_taper_rate = bq27427_taper_rate();
    uint16_t current_opconfig = bq27427_op_config();
    bool sleep_enabled = (current_opconfig & BQ27427_OPCONFIG_SLEEP) != 0;

    s_init_diag.init_current_capacity    = current_capacity;
    s_init_diag.init_current_terminate_v = current_terminate_voltage;
    s_init_diag.init_current_taper_rate  = current_taper_rate;
    s_init_diag.init_current_opconfig    = current_opconfig;
    s_init_diag.init_sleep_enabled       = sleep_enabled ? 1 : 0;
    s_init_diag.init_itpor_flag          = bq27427_itpor_flag() ? 1 : 0;
    s_init_diag.chem_id_fail_stage       = bq27427_get_chem_id_fail_stage();

    // DIAGNOSTIC OVERRIDE: force the reconfigure path unconditionally so we can
    // observe whether enter_config / set_* / exit_config actually succeed,
    // independent of any "looks already configured" early-out.
    (void)current_terminate_voltage;
    (void)current_taper_rate;
    (void)sleep_enabled;
    bool needs_config = true;

    if (needs_config) {
        s_reconfig_count++;

        if (!bq27427_enter_config(true)) {
            s_init_diag.init_fail_stage = 5;
            return false;
        }

        if (!bq27427_set_current_polarity(0)) { // 0 = Positive current means battery is charging
            s_init_diag.init_fail_stage = 6;
            return false;
        }
        if (!bq27427_set_capacity(300)) {
            s_init_diag.init_fail_stage = 6;
            return false;
        }
        if (!bq27427_set_design_energy(1110)) {  // 300mAh * 3.7V
            s_init_diag.init_fail_stage = 6;
            return false;
        }
        if (!bq27427_set_terminate_voltage(3000)) {
            s_init_diag.init_fail_stage = 6;
            return false;
        }
        if (!bq27427_set_taper_rate(100)) {  // (300mAh / 30mA) * 10 = 100, CUTS OFF AT 26mA CHARGING
            s_init_diag.init_fail_stage = 6;
            return false;
        }
        // Force gauge to stay in NORMAL mode so AverageCurrent() reflects real load
        // (SLEEP mode filters readings to ~10 mA when load is below 30 mA wake threshold).
        if (!bq27427_disable_sleep()) {
            s_init_diag.init_fail_stage = 6;
            return false;
        }

        if (!bq27427_exit_config(true)) {
            s_init_diag.init_fail_stage = 7;
            return false;
        }

        HAL_Delay(1000);
    }

    // Give the gauge a brief settle window after the config-write exit_config
    // (above) before re-entering CFGUPMODE for the readback batch — back-to-back
    // sessions can race INITCOMP and cause enter_config to time out.
    HAL_Delay(200);

    // Snapshot all "static" gauge-config values exactly once.
    // After init, BATTERY_UpdateState reads only dynamic registers — entering
    // CONFIG UPDATE every second would suspend gauging and itself break current readings.
    BATTERY_RefreshConfigCache();

    // One-shot diagnostic: try to read design capacity with NO user-config session.
    // _user_config_control should be false here; read_extended_data manages its own
    // enter/exit. If this also returns 0 the bug is below session management
    // (I2C, addressing, BlockData dance, etc).
    {
        uint8_t b6 = bq27427_read_extended_data(BQ27427_ID_STATE, 6);
        uint8_t b7 = bq27427_read_extended_data(BQ27427_ID_STATE, 7);
        s_test_design_cap_byte6 = b6;
        s_test_design_cap_byte7 = b7;
        s_test_design_cap = ((uint16_t)b6 << 8) | b7;
    }

    // Initialize battery state
    battery_state.last_update = 0;

    // If init_fail_stage was set to 4 (chem_id soft-failure) we leave it as-is
    // for visibility, but mark init_completed=1 since we ran to the end. Any
    // hard fail above already returned early without setting init_completed.
    s_init_diag.init_completed = 1;

    return true;
}

/**
 * @brief Snapshot all "static" gauge-config values into the cache in a single
 *        user-controlled CONFIG UPDATE session (one enter/exit instead of six).
 */
void BATTERY_RefreshConfigCache(void)
{
    // Reset diag for this call.
    s_refresh_diag = (battery_refresh_diag_t){0};
    s_refresh_diag.user_ctrl_at_entry = bq27427_is_user_config_active() ? 1 : 0;

    // The gauge can need a moment between consecutive config sessions; retry
    // a few times before giving up so a transient INITCOMP race doesn't poison
    // the cache.
    bool entered = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (bq27427_enter_config(true)) {
            entered = true;
            s_refresh_diag.attempts_used = (uint8_t)(attempt + 1);
            break;
        }
        HAL_Delay(50);
    }
    s_refresh_diag.entered = entered ? 1 : 0;

    if (!entered) {
        // Mark cache as invalid so consumers (and the BLE diagnostic) can
        // distinguish "0 because read failed" from a legitimate value.
        battery_state.design_capacity_mAh   = 0;
        battery_state.terminate_voltage_mV  = 0;
        battery_state.taper_rate            = 0;
        battery_state.op_config_raw         = 0;
        battery_state.board_offset          = 0;
        battery_state.deadband_mA           = 0;
        cached_design_capacity              = 0;
        return;
    }

    // Confirm CFGUPMODE bit (0x10) is actually set after a "successful" entry.
    s_refresh_diag.flags_in_cfgmode = bq27427_flags();

    battery_state.design_capacity_mAh   = bq27427_capacity(BQ27427_CAPACITY_DESIGN);
    s_refresh_diag.design_cap_raw       = battery_state.design_capacity_mAh;
    battery_state.terminate_voltage_mV  = bq27427_terminate_voltage();
    battery_state.taper_rate            = bq27427_taper_rate();
    battery_state.op_config_raw         = bq27427_op_config();
    battery_state.board_offset          = (int8_t)bq27427_read_extended_data(BQ27427_ID_CALIB_DATA, 0);
    battery_state.deadband_mA           = bq27427_read_extended_data(BQ27427_ID_CURRENT, 1);

    bool exit_ok = bq27427_exit_config(true);
    s_refresh_diag.exited = exit_ok ? 1 : 0;

    cached_design_capacity = battery_state.design_capacity_mAh;
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

    // ITPOR set means gauge lost its config. Recording it as a flag here
    // (instead of recursing into BATTERY_Init) avoids re-init storms that
    // mask diagnostics; the main loop / state machine can decide when to
    // re-init based on this status.
    if (flags & BQ27427_FLAG_ITPOR) {
        battery_state.itpor = true;
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

    // AveragePower() at 0x18 — standard command, no config-mode penalty.
    battery_state.average_power_mW = bq27427_power();

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

uint16_t BATTERY_GetDesignCapacity(void)     { return battery_state.design_capacity_mAh; }
uint16_t BATTERY_GetTerminateVoltage(void)   { return battery_state.terminate_voltage_mV; }
uint16_t BATTERY_GetTaperRate(void)          { return battery_state.taper_rate; }
uint16_t BATTERY_GetOpConfig(void)           { return battery_state.op_config_raw; }
int16_t  BATTERY_GetAveragePower(void)       { return battery_state.average_power_mW; }
int8_t   BATTERY_GetBoardOffset(void)        { return battery_state.board_offset; }
uint8_t  BATTERY_GetDeadband(void)           { return battery_state.deadband_mA; }

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
