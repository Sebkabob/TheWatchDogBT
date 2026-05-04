/***************************************************************************
 * battery.c
 * created by Sebastian Forenza 2026
 *
 * BQ27427 fuel-gauge wrapper. Cached state is updated once per second
 * via BATTERY_UpdateState(); accessors return that cache so the state
 * machine never blocks on I2C. BATTERY_Init() handles a self-healing
 * CC-Gain restore + one-shot reconfigure of design capacity, terminate
 * voltage, taper rate, and SLEEP-disable.
 ***************************************************************************/

#include "battery.h"
#include "bq27427_reg.h"
#include "main.h"

typedef struct {
    uint16_t voltage_mV;
    int16_t current_mA;
    uint16_t soc_percent;
    bool is_charging;
    bool is_full;
    bool is_low;
    bool is_critical;
    uint32_t last_update;

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

    // Refreshed only via BATTERY_RefreshConfigCache (CFGUPMODE entry).
    uint16_t design_capacity_mAh;
    uint16_t terminate_voltage_mV;
    uint16_t taper_rate;
    uint16_t op_config_raw;
    int8_t   board_offset;
    uint8_t  deadband_mA;
    int16_t  average_power_mW;
} BatteryState_t;

static BatteryState_t battery_state = {0};
static uint16_t cached_design_capacity = 0;

// Subclass 104 (Calibration) raw dump — diagnostic surface for CC Gain trim.
static uint8_t s_calib_bytes[16] = {0};
const uint8_t *BATTERY_GetCalibBytes(void) { return s_calib_bytes; }

// init_fail_stage codes:
//   0  success / not yet
//   1  bq27427_init() failed
//   2  device_type wrong
//   3  initial INITCOMP timeout
//   4  post-RESET INITCOMP timeout
//   5  enter_config failed
//   6  set_current_polarity failed
//   7  set_capacity failed
//   8  set_design_energy failed
//   9  set_terminate_voltage failed
//   10 set_taper_rate failed
//   11 disable_sleep failed
//   12 exit_config failed
static uint8_t s_init_fail_stage = 0;
static uint8_t s_init_completed  = 0;
static uint8_t s_post_reset_fired = 0;
static uint16_t s_chem_id_read = 0;
uint8_t  BATTERY_GetInitFailStage(void)  { return s_init_fail_stage; }
uint8_t  BATTERY_GetInitCompleted(void)  { return s_init_completed; }
uint8_t  BATTERY_GetPostResetFired(void) { return s_post_reset_fired; }
uint16_t BATTERY_GetChemIdRead(void)     { return s_chem_id_read; }

/***************************************************************************
 * BATTERY_Init — bring up gauge, self-heal CC-Gain, write static config
 *   Idempotent: PowerMgmt_RestoreAll() and the BLE-connect path both call
 *   this. The full reconfigure is skipped when ITPOR is clear and we've
 *   already succeeded once this boot.
 *   Returns true on success, false on any failure (s_init_fail_stage tells
 *   exactly which step bailed).
 ***************************************************************************/
bool BATTERY_Init(void)
{
    static bool s_initialized = false;
    if (s_initialized && !bq27427_itpor_flag()) {
        return true;
    }

    if (!bq27427_init()) {
        s_init_fail_stage = 1;
        return false;
    }

    if (bq27427_device_type() != 0x0427) {
        s_init_fail_stage = 2;
        return false;
    }

    uint32_t init_start = HAL_GetTick();
    while (!(bq27427_status() & BQ27427_STATUS_INITCOMP)) {
        if ((HAL_GetTick() - init_start) >= 1500) {
            s_init_fail_stage = 3;
            return false;
        }
        HAL_Delay(10);
    }

    // CFGUPMODE never asserts when SET_CFGUPDATE is issued too soon after
    // INITCOMP — observed in field units.
    HAL_Delay(250);

    // Self-heal: if CC Gain (Subclass 104, offsets 0..3) reads all zero, the
    // factory current-scaling trim has been clobbered (Current() reads ~10x
    // low). CONTROL_RESET restores data flash to ROM defaults including the
    // factory CC Gain. Self-limiting — once non-zero, this branch is skipped.
    {
        uint8_t g0 = bq27427_read_extended_data(BQ27427_ID_CALIB_DATA, 0);
        uint8_t g1 = bq27427_read_extended_data(BQ27427_ID_CALIB_DATA, 1);
        uint8_t g2 = bq27427_read_extended_data(BQ27427_ID_CALIB_DATA, 2);
        uint8_t g3 = bq27427_read_extended_data(BQ27427_ID_CALIB_DATA, 3);
        if ((g0 | g1 | g2 | g3) == 0) {
            s_post_reset_fired = 1;
            // Issue CONTROL_RESET directly without CFGUPMODE wrapping —
            // wrapping leaves the chip in a state where INITCOMP never
            // re-asserts after RESET.
            bq27427_execute_control_word(BQ27427_CONTROL_RESET);
            HAL_Delay(500);
            uint32_t reset_start = HAL_GetTick();
            while (!(bq27427_status() & BQ27427_STATUS_INITCOMP)) {
                if ((HAL_GetTick() - reset_start) >= 5000) {
                    s_init_fail_stage = 4;
                    return false;
                }
                HAL_Delay(10);
            }
            HAL_Delay(250);
        }
    }

    if (!(bq27427_flags() & BQ27427_FLAG_BAT_DET)) {
        bq27427_execute_control_word(BQ27427_CONTROL_BAT_INSERT);
        HAL_Delay(100);
    }

    s_chem_id_read = (uint16_t)bq27427_chem_id();
    if (s_chem_id_read != BQ27427_CHEM_B) {
        bq27427_set_chem_id(BQ27427_CHEM_B);
        s_chem_id_read = (uint16_t)bq27427_chem_id();
    }

    uint16_t current_capacity = bq27427_capacity(BQ27427_CAPACITY_DESIGN);
    uint16_t current_terminate_voltage = bq27427_terminate_voltage();
    uint16_t current_taper_rate = bq27427_taper_rate();
    uint16_t current_opconfig = bq27427_op_config();
    bool sleep_disabled = (current_opconfig & BQ27427_OPCONFIG_SLEEP) == 0;

    bool needs_config = (current_capacity != 300) ||
                        (current_terminate_voltage != 3000) ||
                        (current_taper_rate != 100) ||
                        sleep_disabled ||
                        bq27427_itpor_flag();

    if (needs_config) {
        if (!bq27427_enter_config(true))      { s_init_fail_stage = 5;  return false; }
        if (!bq27427_set_current_polarity(0)) { s_init_fail_stage = 6;  return false; }
        if (!bq27427_set_capacity(300))       { s_init_fail_stage = 7;  return false; }
        if (!bq27427_set_design_energy(1110)) { s_init_fail_stage = 8;  return false; }
        if (!bq27427_set_terminate_voltage(3000)) { s_init_fail_stage = 9;  return false; }
        if (!bq27427_set_taper_rate(100))     { s_init_fail_stage = 10; return false; }
        if (!bq27427_enable_sleep())          { s_init_fail_stage = 11; return false; }
        if (!bq27427_exit_config(true))       { s_init_fail_stage = 12; return false; }

        HAL_Delay(1000);
    }

    // Settle window: back-to-back CFGUPMODE sessions following a flash write
    // race INITCOMP and cause subclass reads to return 0x0000.
    HAL_Delay(500);

    BATTERY_RefreshConfigCache();

    battery_state.last_update = 0;
    s_initialized = true;
    s_init_completed = 1;

    return true;
}

/***************************************************************************
 * BATTERY_RefreshConfigCache — snapshot static config in one CFGUPMODE pass
 *   One enter/exit covers Design Capacity / Terminate Voltage / Taper /
 *   OpConfig / Deadband / Subclass-104 dump. Retries CFGUPMODE entry up to
 *   three times to ride out transient INITCOMP races.
 ***************************************************************************/
void BATTERY_RefreshConfigCache(void)
{
    bool entered = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (bq27427_enter_config(true)) {
            entered = true;
            break;
        }
        HAL_Delay(50);
    }

    if (!entered) {
        battery_state.design_capacity_mAh   = 0;
        battery_state.terminate_voltage_mV  = 0;
        battery_state.taper_rate            = 0;
        battery_state.op_config_raw         = 0;
        battery_state.board_offset          = 0;
        battery_state.deadband_mA           = 0;
        cached_design_capacity              = 0;
        return;
    }

    battery_state.design_capacity_mAh   = bq27427_capacity(BQ27427_CAPACITY_DESIGN);
    battery_state.terminate_voltage_mV  = bq27427_terminate_voltage();
    battery_state.taper_rate            = bq27427_taper_rate();
    battery_state.op_config_raw         = bq27427_op_config();
    // Board Offset read intentionally skipped: the BlockData/checksum dance
    // around Subclass 104 / offset 0 (which is CC Gain byte 0) clobbered CC
    // Gain to all-zero on at least one unit. The value was never used.
    battery_state.board_offset          = 0;
    battery_state.deadband_mA           = bq27427_read_extended_data(BQ27427_ID_CURRENT, 1);

    for (uint8_t i = 0; i < 16; i++) {
        s_calib_bytes[i] = bq27427_read_extended_data(BQ27427_ID_CALIB_DATA, i);
    }

    bq27427_exit_config(true);

    cached_design_capacity = battery_state.design_capacity_mAh;
}

/***************************************************************************
 * BATTERY_UpdateState — refresh cache (rate-limited to once per second)
 *   ITPOR is recorded as a flag here rather than triggering a re-init —
 *   that decision is left to the state machine so re-init storms don't
 *   mask diagnostics.
 ***************************************************************************/
bool BATTERY_UpdateState(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - battery_state.last_update) < 1000) {
        return true;
    }

    uint16_t flags = bq27427_flags();

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

    battery_state.average_power_mW = bq27427_power();

    // FC asserted but SOC < 100 — pin to 100 so the UI doesn't show 99%.
    if (battery_state.is_full && battery_state.soc_percent < 100) {
        battery_state.soc_percent = 100;
    }

    battery_state.last_update = now;
    return true;
}

uint16_t BATTERY_GetVoltage(void)         { return battery_state.voltage_mV; }
int16_t  BATTERY_GetCurrent(void)         { return battery_state.current_mA; }
uint16_t BATTERY_GetSOC(void)             { return battery_state.soc_percent; }
bool     BATTERY_IsCharging(void)         { return battery_state.is_charging; }
bool     BATTERY_IsFullCached(void)       { return battery_state.is_full; }
bool     BATTERY_IsLowCached(void)        { return battery_state.is_low; }
bool     BATTERY_IsCriticallyCached(void) { return battery_state.is_critical; }

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
