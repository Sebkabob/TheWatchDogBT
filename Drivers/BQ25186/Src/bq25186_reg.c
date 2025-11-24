/**
 * @file bq25186.c
 * @brief BQ25186 1-Cell, 1A I2C Linear Battery Charger Driver Implementation
 */

#include <stddef.h>
#include "bq25186_reg.h"

/* ========================================================================== */
/*                           Private Functions                                 */
/* ========================================================================== */

/**
 * @brief Calculate ICHG register code from current in mA
 *
 * For ICHG <= 35mA: code = (current_ma - 5)
 * For ICHG > 35mA: code = 31 + ((current_ma - 40) / 10)
 */
static uint8_t BQ25186_CalculateIchgCode(uint16_t current_ma)
{
    if (current_ma < 5) {
        return 0;  // Minimum 5mA
    } else if (current_ma <= 35) {
        return (uint8_t)(current_ma - 5);
    } else if (current_ma <= 1000) {
        return (uint8_t)(31 + ((current_ma - 40) / 10));
    } else {
        return 127;  // Maximum code (1000mA)
    }
}

/**
 * @brief Calculate current in mA from ICHG register code
 */
static uint16_t BQ25186_CalculateCurrentFromCode(uint8_t code)
{
    if (code <= 30) {
        return (uint16_t)(code + 5);
    } else {
        return (uint16_t)(40 + ((code - 31) * 10));
    }
}

/**
 * @brief Calculate VBATREG register code from voltage in mV
 *
 * VBATREG = 3500mV + (code * 10mV), max 4650mV
 */
static uint8_t BQ25186_CalculateVbatRegCode(uint16_t voltage_mv)
{
    if (voltage_mv < 3500) {
        return 0;  // Minimum 3.5V
    } else if (voltage_mv <= 4650) {
        return (uint8_t)((voltage_mv - 3500) / 10);
    } else {
        return 115;  // Maximum code (4.65V)
    }
}

/**
 * @brief Calculate voltage in mV from VBATREG register code
 */
static uint16_t BQ25186_CalculateVoltageFromCode(uint8_t code)
{
    return (uint16_t)(3500 + (code * 10));
}

/* ========================================================================== */
/*                           Public Functions                                  */
/* ========================================================================== */

BQ25186_Status_t BQ25186_Init(BQ25186_Handle_t *handle,
                               BQ25186_I2C_Write_t i2c_write,
                               BQ25186_I2C_Read_t i2c_read,
                               BQ25186_Delay_Ms_t delay_ms)
{
    if (handle == NULL || i2c_write == NULL ||
        i2c_read == NULL || delay_ms == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    // Initialize handle
    handle->i2c_write = i2c_write;
    handle->i2c_read = i2c_read;
    handle->delay_ms = delay_ms;
    handle->device_id = 0;

    // Small delay after power-up
    delay_ms(10);

    // Read and verify device ID
    uint8_t device_id;
    BQ25186_Status_t status = BQ25186_GetDeviceID(handle, &device_id);
    if (status != BQ25186_OK) {
        return status;
    }

    // Expected device ID is 0x01 (from datasheet)
    if (device_id != 0x01) {
        return BQ25186_ERROR_DEVICE_ID;
    }

    handle->device_id = device_id;

    return BQ25186_OK;
}

BQ25186_Status_t BQ25186_GetDeviceID(BQ25186_Handle_t *handle, uint8_t *device_id)
{
    if (handle == NULL || device_id == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t reg_val;
    BQ25186_Status_t status = BQ25186_ReadRegister(handle,
                                                     BQ25186_REG_MASK_ID,
                                                     &reg_val);
    if (status != BQ25186_OK) {
        return status;
    }

    *device_id = reg_val & BQ25186_MASK_ID_DEVICE_ID_MASK;
    return BQ25186_OK;
}

BQ25186_Status_t BQ25186_SoftwareReset(BQ25186_Handle_t *handle)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    // Set REG_RST bit
    BQ25186_Status_t status = BQ25186_ModifyRegister(handle,
                                                       BQ25186_REG_SHIP_RST,
                                                       BQ25186_SHIP_RST_REG_RST,
                                                       BQ25186_SHIP_RST_REG_RST);
    if (status != BQ25186_OK) {
        return status;
    }

    // Wait for reset to complete
    handle->delay_ms(100);

    return BQ25186_OK;
}

BQ25186_Status_t BQ25186_GetStatus(BQ25186_Handle_t *handle,
                                    BQ25186_Status_Regs_t *status)
{
    if (handle == NULL || status == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t stat0, stat1;
    BQ25186_Status_t ret;

    // Read STAT0
    ret = BQ25186_ReadRegister(handle, BQ25186_REG_STAT0, &stat0);
    if (ret != BQ25186_OK) {
        return ret;
    }

    // Read STAT1
    ret = BQ25186_ReadRegister(handle, BQ25186_REG_STAT1, &stat1);
    if (ret != BQ25186_OK) {
        return ret;
    }

    // Parse STAT0
    status->ts_open = (stat0 & BQ25186_STAT0_TS_OPEN_STAT) ? true : false;
    status->chg_stat = (BQ25186_ChgStat_t)((stat0 & BQ25186_STAT0_CHG_STAT_MASK) >>
                                            BQ25186_STAT0_CHG_STAT_SHIFT);
    status->ilim_active = (stat0 & BQ25186_STAT0_ILIM_ACTIVE_STAT) ? true : false;
    status->vdppm_active = (stat0 & BQ25186_STAT0_VDPPM_ACTIVE_STAT) ? true : false;
    status->vindpm_active = (stat0 & BQ25186_STAT0_VINDPM_ACTIVE_STAT) ? true : false;
    status->thermreg_active = (stat0 & BQ25186_STAT0_THERMREG_ACTIVE_STAT) ? true : false;
    status->vin_pgood = (stat0 & BQ25186_STAT0_VIN_PGOOD_STAT) ? true : false;

    // Parse STAT1
    status->vin_ovp = (stat1 & BQ25186_STAT1_VIN_OVP_STAT) ? true : false;
    status->buvlo = (stat1 & BQ25186_STAT1_BUVLO_STAT) ? true : false;
    status->ts_stat = (BQ25186_TsStat_t)((stat1 & BQ25186_STAT1_TS_STAT_MASK) >>
                                          BQ25186_STAT1_TS_STAT_SHIFT);
    status->safety_timer_fault = (stat1 & BQ25186_STAT1_SAFETY_TMR_FAULT) ? true : false;
    status->wake1_flag = (stat1 & BQ25186_STAT1_WAKE1_FLAG) ? true : false;
    status->wake2_flag = (stat1 & BQ25186_STAT1_WAKE2_FLAG) ? true : false;

    return BQ25186_OK;
}

BQ25186_Status_t BQ25186_GetFaults(BQ25186_Handle_t *handle,
                                    BQ25186_Faults_t *faults)
{
    if (handle == NULL || faults == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t flag0;
    BQ25186_Status_t ret = BQ25186_ReadRegister(handle,
                                                  BQ25186_REG_FLAG0,
                                                  &flag0);
    if (ret != BQ25186_OK) {
        return ret;
    }

    // Parse FLAG0
    faults->ts_fault = (flag0 & BQ25186_FLAG0_TS_FAULT) ? true : false;
    faults->ilim_active = (flag0 & BQ25186_FLAG0_ILIM_ACTIVE_FLAG) ? true : false;
    faults->vdppm_active = (flag0 & BQ25186_FLAG0_VDPPM_ACTIVE_FLAG) ? true : false;
    faults->vindpm_active = (flag0 & BQ25186_FLAG0_VINDPM_ACTIVE_FLAG) ? true : false;
    faults->thermreg_active = (flag0 & BQ25186_FLAG0_THERMREG_ACTIVE_FLAG) ? true : false;
    faults->vin_ovp_fault = (flag0 & BQ25186_FLAG0_VIN_OVP_FAULT_FLAG) ? true : false;
    faults->buvlo_fault = (flag0 & BQ25186_FLAG0_BUVLO_FAULT_FLAG) ? true : false;
    faults->bat_ocp_fault = (flag0 & BQ25186_FLAG0_BAT_OCP_FAULT) ? true : false;

    return BQ25186_OK;
}

BQ25186_Status_t BQ25186_ClearFaults(BQ25186_Handle_t *handle)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    // Reading FLAG0 clears the flags (read-to-clear)
    uint8_t dummy;
    return BQ25186_ReadRegister(handle, BQ25186_REG_FLAG0, &dummy);
}

BQ25186_Status_t BQ25186_SetBatteryVoltage(BQ25186_Handle_t *handle,
                                            uint16_t voltage_mv)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    if (voltage_mv < 3500 || voltage_mv > 4650) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t code = BQ25186_CalculateVbatRegCode(voltage_mv);

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_VBAT_CTRL,
                                   BQ25186_VBAT_CTRL_VBATREG_MASK,
                                   code);
}

BQ25186_Status_t BQ25186_GetBatteryVoltage(BQ25186_Handle_t *handle,
                                            uint16_t *voltage_mv)
{
    if (handle == NULL || voltage_mv == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t reg_val;
    BQ25186_Status_t status = BQ25186_ReadRegister(handle,
                                                     BQ25186_REG_VBAT_CTRL,
                                                     &reg_val);
    if (status != BQ25186_OK) {
        return status;
    }

    uint8_t code = reg_val & BQ25186_VBAT_CTRL_VBATREG_MASK;
    *voltage_mv = BQ25186_CalculateVoltageFromCode(code);

    return BQ25186_OK;
}

BQ25186_Status_t BQ25186_SetChargeCurrent(BQ25186_Handle_t *handle,
                                           uint16_t current_ma)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    if (current_ma < 5 || current_ma > 1000) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t code = BQ25186_CalculateIchgCode(current_ma);

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_ICHG_CTRL,
                                   BQ25186_ICHG_CTRL_ICHG_MASK,
                                   code);
}

BQ25186_Status_t BQ25186_GetChargeCurrent(BQ25186_Handle_t *handle,
                                           uint16_t *current_ma)
{
    if (handle == NULL || current_ma == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t reg_val;
    BQ25186_Status_t status = BQ25186_ReadRegister(handle,
                                                     BQ25186_REG_ICHG_CTRL,
                                                     &reg_val);
    if (status != BQ25186_OK) {
        return status;
    }

    uint8_t code = reg_val & BQ25186_ICHG_CTRL_ICHG_MASK;
    *current_ma = BQ25186_CalculateCurrentFromCode(code);

    return BQ25186_OK;
}

BQ25186_Status_t BQ25186_SetChargeEnable(BQ25186_Handle_t *handle, bool enable)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    // CHG_DIS bit: 1 = disable, 0 = enable (inverse logic)
    uint8_t value = enable ? 0 : BQ25186_ICHG_CTRL_CHG_DIS;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_ICHG_CTRL,
                                   BQ25186_ICHG_CTRL_CHG_DIS,
                                   value);
}

BQ25186_Status_t BQ25186_GetChargeEnable(BQ25186_Handle_t *handle, bool *enabled)
{
    if (handle == NULL || enabled == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t reg_val;
    BQ25186_Status_t status = BQ25186_ReadRegister(handle,
                                                     BQ25186_REG_ICHG_CTRL,
                                                     &reg_val);
    if (status != BQ25186_OK) {
        return status;
    }

    // CHG_DIS bit: 1 = disabled, 0 = enabled (inverse logic)
    *enabled = (reg_val & BQ25186_ICHG_CTRL_CHG_DIS) ? false : true;

    return BQ25186_OK;
}

BQ25186_Status_t BQ25186_SetTerminationCurrent(BQ25186_Handle_t *handle,
                                                BQ25186_Iterm_t iterm)
{
    if (handle == NULL || iterm > BQ25186_ITERM_20_PERCENT) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)iterm << BQ25186_CHARGECTRL0_ITERM_SHIFT) &
                     BQ25186_CHARGECTRL0_ITERM_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_CHARGECTRL0,
                                   BQ25186_CHARGECTRL0_ITERM_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_SetVINDPM(BQ25186_Handle_t *handle,
                                    BQ25186_Vindpm_t vindpm)
{
    if (handle == NULL || vindpm > BQ25186_VINDPM_DISABLED) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)vindpm << BQ25186_CHARGECTRL0_VINDPM_SHIFT) &
                     BQ25186_CHARGECTRL0_VINDPM_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_CHARGECTRL0,
                                   BQ25186_CHARGECTRL0_VINDPM_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_SetThermalRegulation(BQ25186_Handle_t *handle,
                                               BQ25186_ThermalReg_t thermal_reg)
{
    if (handle == NULL || thermal_reg > BQ25186_TREG_DISABLED) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)thermal_reg << BQ25186_CHARGECTRL0_THERM_REG_SHIFT) &
                     BQ25186_CHARGECTRL0_THERM_REG_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_CHARGECTRL0,
                                   BQ25186_CHARGECTRL0_THERM_REG_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_SetBUVLO(BQ25186_Handle_t *handle,
                                   BQ25186_Buvlo_t buvlo)
{
    if (handle == NULL || buvlo > BQ25186_BUVLO_2000MV) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)buvlo << BQ25186_CHARGECTRL1_BUVLO_SHIFT) &
                     BQ25186_CHARGECTRL1_BUVLO_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_CHARGECTRL1,
                                   BQ25186_CHARGECTRL1_BUVLO_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_SetBatteryOCP(BQ25186_Handle_t *handle,
                                        BQ25186_BatOcp_t bat_ocp)
{
    if (handle == NULL || bat_ocp > BQ25186_BATOCP_3000MA) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)bat_ocp << BQ25186_CHARGECTRL1_IBAT_OCP_SHIFT) &
                     BQ25186_CHARGECTRL1_IBAT_OCP_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_CHARGECTRL1,
                                   BQ25186_CHARGECTRL1_IBAT_OCP_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_SetPrechargeVoltage(BQ25186_Handle_t *handle,
                                              BQ25186_Vlowv_t vlowv)
{
    if (handle == NULL || vlowv > BQ25186_VLOWV_2800MV) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = vlowv ? BQ25186_IC_CTRL_VLOWV_SEL : 0;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_IC_CTRL,
                                   BQ25186_IC_CTRL_VLOWV_SEL,
                                   value);
}

BQ25186_Status_t BQ25186_SetRechargeThreshold(BQ25186_Handle_t *handle,
                                               BQ25186_Vrch_t vrch)
{
    if (handle == NULL || vrch > BQ25186_VRCH_200MV) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = vrch ? BQ25186_IC_CTRL_VRCH_0 : 0;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_IC_CTRL,
                                   BQ25186_IC_CTRL_VRCH_0,
                                   value);
}

BQ25186_Status_t BQ25186_SetSafetyTimer(BQ25186_Handle_t *handle,
                                        BQ25186_SafetyTimer_t timer)
{
    if (handle == NULL || timer > BQ25186_SAFETY_TIMER_DISABLED) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)timer << BQ25186_IC_CTRL_SAFETY_TIMER_SHIFT) &
                     BQ25186_IC_CTRL_SAFETY_TIMER_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_IC_CTRL,
                                   BQ25186_IC_CTRL_SAFETY_TIMER_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_Set2xTimerEnable(BQ25186_Handle_t *handle, bool enable)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = enable ? BQ25186_IC_CTRL_2XTMR_EN : 0;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_IC_CTRL,
                                   BQ25186_IC_CTRL_2XTMR_EN,
                                   value);
}

BQ25186_Status_t BQ25186_SetWatchdog(BQ25186_Handle_t *handle,
                                      BQ25186_Watchdog_t watchdog)
{
    if (handle == NULL || watchdog > BQ25186_WATCHDOG_DISABLED) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)watchdog << BQ25186_IC_CTRL_WATCHDOG_SEL_SHIFT) &
                     BQ25186_IC_CTRL_WATCHDOG_SEL_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_IC_CTRL,
                                   BQ25186_IC_CTRL_WATCHDOG_SEL_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_ResetWatchdog(BQ25186_Handle_t *handle)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    // Any I2C transaction resets the watchdog
    // Reading a register is sufficient
    uint8_t dummy;
    return BQ25186_ReadRegister(handle, BQ25186_REG_STAT0, &dummy);
}

BQ25186_Status_t BQ25186_SetInputCurrentLimit(BQ25186_Handle_t *handle,
                                               BQ25186_Ilim_t ilim)
{
    if (handle == NULL || ilim > BQ25186_ILIM_1050MA) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)ilim << BQ25186_TMR_ILIM_ILIM_SHIFT) &
                     BQ25186_TMR_ILIM_ILIM_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_TMR_ILIM,
                                   BQ25186_TMR_ILIM_ILIM_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_SetLongPressTimer(BQ25186_Handle_t *handle,
                                            BQ25186_MrLongPress_t timer)
{
    if (handle == NULL || timer > BQ25186_MR_LPRESS_20S) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)timer << BQ25186_TMR_ILIM_MR_LPRESS_SHIFT) &
                     BQ25186_TMR_ILIM_MR_LPRESS_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_TMR_ILIM,
                                   BQ25186_TMR_ILIM_MR_LPRESS_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_SetAutoWakeTimer(BQ25186_Handle_t *handle,
                                           BQ25186_AutoWake_t timer)
{
    if (handle == NULL || timer > BQ25186_AUTOWAKE_4S) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)timer << BQ25186_TMR_ILIM_AUTOWAKE_SHIFT) &
                     BQ25186_TMR_ILIM_AUTOWAKE_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_TMR_ILIM,
                                   BQ25186_TMR_ILIM_AUTOWAKE_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_SetPowerMode(BQ25186_Handle_t *handle,
                                       BQ25186_EnRstShip_t mode)
{
    if (handle == NULL || mode > BQ25186_EN_RST_SHIP_HW_RESET) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)mode << BQ25186_SHIP_RST_EN_RST_SHIP_SHIFT) &
                     BQ25186_SHIP_RST_EN_RST_SHIP_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_SHIP_RST,
                                   BQ25186_SHIP_RST_EN_RST_SHIP_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_SetLongPressAction(BQ25186_Handle_t *handle,
                                             BQ25186_PbLpressAction_t action)
{
    if (handle == NULL || action > BQ25186_PB_LPRESS_SHUTDOWN) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)action << BQ25186_SHIP_RST_PB_LPRESS_ACTION_SHIFT) &
                     BQ25186_SHIP_RST_PB_LPRESS_ACTION_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_SHIP_RST,
                                   BQ25186_SHIP_RST_PB_LPRESS_ACTION_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_SetPushButtonEnable(BQ25186_Handle_t *handle,
                                              bool enable)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = enable ? BQ25186_SHIP_RST_EN_PUSH : 0;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_SHIP_RST,
                                   BQ25186_SHIP_RST_EN_PUSH,
                                   value);
}

BQ25186_Status_t BQ25186_SetSysRegulation(BQ25186_Handle_t *handle,
                                           BQ25186_SysReg_t sys_reg)
{
    if (handle == NULL || sys_reg > BQ25186_SYS_REG_PASSTHROUGH) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)sys_reg << BQ25186_SYS_REG_SYS_REG_CTRL_SHIFT) &
                     BQ25186_SYS_REG_SYS_REG_CTRL_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_SYS_REG,
                                   BQ25186_SYS_REG_SYS_REG_CTRL_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_SetSysMode(BQ25186_Handle_t *handle,
                                     BQ25186_SysMode_t mode)
{
    if (handle == NULL || mode > BQ25186_SYS_MODE_OFF_PULLDOWN) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)mode << BQ25186_SYS_REG_SYS_MODE_SHIFT) &
                     BQ25186_SYS_REG_SYS_MODE_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_SYS_REG,
                                   BQ25186_SYS_REG_SYS_MODE_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_Set15sWatchdogEnable(BQ25186_Handle_t *handle,
                                               bool enable)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = enable ? BQ25186_SYS_REG_WATCHDOG_15S_ENABLE : 0;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_SYS_REG,
                                   BQ25186_SYS_REG_WATCHDOG_15S_ENABLE,
                                   value);
}

BQ25186_Status_t BQ25186_SetVDPPMEnable(BQ25186_Handle_t *handle, bool enable)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    // VDPPM_DIS bit: 1 = disable, 0 = enable (inverse logic)
    uint8_t value = enable ? 0 : BQ25186_SYS_REG_VDPPM_DIS;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_SYS_REG,
                                   BQ25186_SYS_REG_VDPPM_DIS,
                                   value);
}

BQ25186_Status_t BQ25186_SetTSEnable(BQ25186_Handle_t *handle, bool enable)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = enable ? BQ25186_IC_CTRL_TS_EN : 0;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_IC_CTRL,
                                   BQ25186_IC_CTRL_TS_EN,
                                   value);
}

BQ25186_Status_t BQ25186_SetTSHotThreshold(BQ25186_Handle_t *handle,
                                            BQ25186_TsHot_t hot_threshold)
{
    if (handle == NULL || hot_threshold > BQ25186_TS_HOT_45C) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)hot_threshold << BQ25186_TS_CONTROL_TS_HOT_SHIFT) &
                     BQ25186_TS_CONTROL_TS_HOT_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_TS_CONTROL,
                                   BQ25186_TS_CONTROL_TS_HOT_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_SetTSColdThreshold(BQ25186_Handle_t *handle,
                                             BQ25186_TsCold_t cold_threshold)
{
    if (handle == NULL || cold_threshold > BQ25186_TS_COLD_NEG3C) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)cold_threshold << BQ25186_TS_CONTROL_TS_COLD_SHIFT) &
                     BQ25186_TS_CONTROL_TS_COLD_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_TS_CONTROL,
                                   BQ25186_TS_CONTROL_TS_COLD_MASK,
                                   value);
}

BQ25186_Status_t BQ25186_SetTSWarmEnable(BQ25186_Handle_t *handle, bool enable)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    // TS_WARM bit: 1 = disabled, 0 = enabled (inverse logic for this bit)
    uint8_t value = enable ? 0 : BQ25186_TS_CONTROL_TS_WARM;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_TS_CONTROL,
                                   BQ25186_TS_CONTROL_TS_WARM,
                                   value);
}

BQ25186_Status_t BQ25186_SetTSCoolEnable(BQ25186_Handle_t *handle, bool enable)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    // TS_COOL bit: 1 = disabled, 0 = enabled (inverse logic for this bit)
    uint8_t value = enable ? 0 : BQ25186_TS_CONTROL_TS_COOL;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_TS_CONTROL,
                                   BQ25186_TS_CONTROL_TS_COOL,
                                   value);
}

BQ25186_Status_t BQ25186_SetTSCurrentReduction(BQ25186_Handle_t *handle,
                                                bool reduce_to_20_percent)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = reduce_to_20_percent ? BQ25186_TS_CONTROL_TS_ICHG : 0;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_TS_CONTROL,
                                   BQ25186_TS_CONTROL_TS_ICHG,
                                   value);
}

BQ25186_Status_t BQ25186_SetTSVoltageReduction(BQ25186_Handle_t *handle,
                                                bool reduce_200mv)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = reduce_200mv ? BQ25186_TS_CONTROL_TS_VRCG : 0;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_TS_CONTROL,
                                   BQ25186_TS_CONTROL_TS_VRCG,
                                   value);
}

BQ25186_Status_t BQ25186_SetPGMode(BQ25186_Handle_t *handle, bool gpo_mode)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = gpo_mode ? BQ25186_VBAT_CTRL_PG_MODE : 0;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_VBAT_CTRL,
                                   BQ25186_VBAT_CTRL_PG_MODE,
                                   value);
}

BQ25186_Status_t BQ25186_SetGPOState(BQ25186_Handle_t *handle, bool state)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    uint8_t value = state ? BQ25186_SYS_REG_PG_GPO : 0;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_SYS_REG,
                                   BQ25186_SYS_REG_PG_GPO,
                                   value);
}

BQ25186_Status_t BQ25186_SetInterruptMasks(BQ25186_Handle_t *handle,
                                            bool mask_ts, bool mask_treg,
                                            bool mask_bat, bool mask_pg,
                                            bool mask_ilim, bool mask_vindpm,
                                            bool mask_chg_status)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    BQ25186_Status_t status;

    // Set MASK_ID register
    uint8_t mask_id_val = 0;
    if (mask_ts) mask_id_val |= BQ25186_MASK_ID_TS_INT_MASK;
    if (mask_treg) mask_id_val |= BQ25186_MASK_ID_TREG_INT_MASK;
    if (mask_bat) mask_id_val |= BQ25186_MASK_ID_BAT_INT_MASK;
    if (mask_pg) mask_id_val |= BQ25186_MASK_ID_PG_INT_MASK;

    uint8_t mask_id_mask = BQ25186_MASK_ID_TS_INT_MASK |
                           BQ25186_MASK_ID_TREG_INT_MASK |
                           BQ25186_MASK_ID_BAT_INT_MASK |
                           BQ25186_MASK_ID_PG_INT_MASK;

    status = BQ25186_ModifyRegister(handle,
                                     BQ25186_REG_MASK_ID,
                                     mask_id_mask,
                                     mask_id_val);
    if (status != BQ25186_OK) {
        return status;
    }

    // Set CHARGECTRL1 register
    uint8_t ctrl1_val = 0;
    if (mask_chg_status) ctrl1_val |= BQ25186_CHARGECTRL1_CHG_STATUS_INT_MASK;
    if (mask_ilim) ctrl1_val |= BQ25186_CHARGECTRL1_ILIM_INT_MASK;
    if (mask_vindpm) ctrl1_val |= BQ25186_CHARGECTRL1_VINDPM_INT_MASK;

    uint8_t ctrl1_mask = BQ25186_CHARGECTRL1_CHG_STATUS_INT_MASK |
                         BQ25186_CHARGECTRL1_ILIM_INT_MASK |
                         BQ25186_CHARGECTRL1_VINDPM_INT_MASK;

    return BQ25186_ModifyRegister(handle,
                                   BQ25186_REG_CHARGECTRL1,
                                   ctrl1_mask,
                                   ctrl1_val);
}

BQ25186_Status_t BQ25186_ApplyChargeConfig(BQ25186_Handle_t *handle,
                                            const BQ25186_ChargeConfig_t *config)
{
    if (handle == NULL || config == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    BQ25186_Status_t status;

    // Set battery regulation voltage
    status = BQ25186_SetBatteryVoltage(handle, config->vbat_reg_mv);
    if (status != BQ25186_OK) return status;

    // Set charge current
    status = BQ25186_SetChargeCurrent(handle, config->ichg_ma);
    if (status != BQ25186_OK) return status;

    // Set termination current
    status = BQ25186_SetTerminationCurrent(handle, config->iterm);
    if (status != BQ25186_OK) return status;

    // Set precharge current (1x or 2x termination current)
    uint8_t iprechg_val = config->iprechg_1x ? BQ25186_CHARGECTRL0_IPRECHG : 0;
    status = BQ25186_ModifyRegister(handle,
                                     BQ25186_REG_CHARGECTRL0,
                                     BQ25186_CHARGECTRL0_IPRECHG,
                                     iprechg_val);
    if (status != BQ25186_OK) return status;

    // Set VINDPM
    status = BQ25186_SetVINDPM(handle, config->vindpm);
    if (status != BQ25186_OK) return status;

    // Set thermal regulation
    status = BQ25186_SetThermalRegulation(handle, config->thermal_reg);
    if (status != BQ25186_OK) return status;

    // Set BUVLO
    status = BQ25186_SetBUVLO(handle, config->buvlo);
    if (status != BQ25186_OK) return status;

    // Set VLOWV (precharge voltage threshold)
    status = BQ25186_SetPrechargeVoltage(handle, config->vlowv);
    if (status != BQ25186_OK) return status;

    // Set recharge threshold
    status = BQ25186_SetRechargeThreshold(handle, config->vrch);
    if (status != BQ25186_OK) return status;

    // Set safety timer
    status = BQ25186_SetSafetyTimer(handle, config->safety_timer);
    if (status != BQ25186_OK) return status;

    // Set 2x timer enable
    status = BQ25186_Set2xTimerEnable(handle, config->timer_2x_en);
    if (status != BQ25186_OK) return status;

    // Set watchdog timer
    status = BQ25186_SetWatchdog(handle, config->watchdog);
    if (status != BQ25186_OK) return status;

    // Set input current limit
    status = BQ25186_SetInputCurrentLimit(handle, config->ilim);
    if (status != BQ25186_OK) return status;

    // Set TS enable
    status = BQ25186_SetTSEnable(handle, config->ts_enable);
    if (status != BQ25186_OK) return status;

    // Set charge enable (do this last)
    status = BQ25186_SetChargeEnable(handle, config->charge_enable);
    if (status != BQ25186_OK) return status;

    return BQ25186_OK;
}

BQ25186_Status_t BQ25186_ReadRegister(BQ25186_Handle_t *handle,
                                       uint8_t reg_addr, uint8_t *data)
{
    if (handle == NULL || data == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    if (handle->i2c_read == NULL) {
        return BQ25186_ERROR_I2C;
    }

    int result = handle->i2c_read(BQ25186_I2C_ADDR, reg_addr, data, 1);

    return (result == 0) ? BQ25186_OK : BQ25186_ERROR_I2C;
}

BQ25186_Status_t BQ25186_WriteRegister(BQ25186_Handle_t *handle,
                                        uint8_t reg_addr, uint8_t data)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    if (handle->i2c_write == NULL) {
        return BQ25186_ERROR_I2C;
    }

    int result = handle->i2c_write(BQ25186_I2C_ADDR, reg_addr, &data, 1);

    return (result == 0) ? BQ25186_OK : BQ25186_ERROR_I2C;
}

BQ25186_Status_t BQ25186_ModifyRegister(BQ25186_Handle_t *handle,
                                         uint8_t reg_addr,
                                         uint8_t mask, uint8_t value)
{
    if (handle == NULL) {
        return BQ25186_ERROR_INVALID_PARAM;
    }

    // Read current register value
    uint8_t reg_val;
    BQ25186_Status_t status = BQ25186_ReadRegister(handle, reg_addr, &reg_val);
    if (status != BQ25186_OK) {
        return status;
    }

    // Modify the register value
    reg_val = (reg_val & ~mask) | (value & mask);

    // Write back the modified value
    return BQ25186_WriteRegister(handle, reg_addr, reg_val);
}
