/******************************************************************************
 * bq27427.c
 * BQ27427 Battery Fuel Gauge STM32 HAL Driver
 *
 * Adapted from SparkFun BQ27441 Arduino Library
 * Original Author: Jim Lindblom @ SparkFun Electronics
 * Original Date: May 9, 2016
 * Original Repo: https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library
 *
 * Arduino Adaptation by: Edrean Ernst
 * Arduino Adaptation Date: June 2025
 * Arduino Repository: https://github.com/edreanernst/BQ27427_Arduino_Library
 *
 * STM32 HAL Adaptation by: Claude
 * STM32 HAL Adaptation Date: 2025
 *
 * This library implements all features of the BQ27427 Battery Fuel Gauge
 * for STM32 microcontrollers using the HAL library.
 *
 * Original License: MIT License
 ******************************************************************************/

#include "bq27427_reg.h"

/******************************************************************************
 * Private Variables
 ******************************************************************************/

static I2C_HandleTypeDef *_hi2c = NULL;
static bool _seal_flag = false;
static bool _user_config_control = false;

/******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static bool bq27427_sealed(void);
static bool bq27427_seal(void);
static bool bq27427_unseal(void);
static uint16_t bq27427_op_config(void);
static bool bq27427_write_op_config(uint16_t value);
static bool bq27427_soft_reset(void);

static uint16_t bq27427_read_word(uint16_t sub_address);
static uint16_t bq27427_read_control_word(uint16_t function);
static bool bq27427_execute_control_word(uint16_t function);

static bool bq27427_block_data_control(void);
static bool bq27427_block_data_class(uint8_t id);
static bool bq27427_block_data_offset(uint8_t offset);
static uint8_t bq27427_block_data_checksum(void);
static uint8_t bq27427_read_block_data(uint8_t offset);
static bool bq27427_write_block_data(uint8_t offset, uint8_t data);
static uint8_t bq27427_compute_block_checksum(void);
static bool bq27427_write_block_checksum(uint8_t csum);
static uint8_t bq27427_read_extended_data(uint8_t class_id, uint8_t offset);
static bool bq27427_write_extended_data(uint8_t class_id, uint8_t offset, uint8_t *data, uint8_t len);

static bool bq27427_i2c_read_bytes(uint8_t sub_address, uint8_t *dest, uint8_t count);
static bool bq27427_i2c_write_bytes(uint8_t sub_address, uint8_t *src, uint8_t count);

static uint8_t constrain_uint8(uint8_t value, uint8_t min, uint8_t max);

/******************************************************************************
 * Initialization Functions
 ******************************************************************************/

bool bq27427_init(void)
{
    extern I2C_HandleTypeDef hi2c1;
    _hi2c = &hi2c1;

    uint16_t device_id = bq27427_device_type();

    if (device_id == BQ27427_DEVICE_ID) {
        return true;
    }

    return false;
}

bool bq27427_set_capacity(uint16_t capacity)
{
    uint8_t cap_msb = capacity >> 8;
    uint8_t cap_lsb = capacity & 0x00FF;
    uint8_t capacity_data[2] = {cap_msb, cap_lsb};
    return bq27427_write_extended_data(BQ27427_ID_STATE, 6, capacity_data, 2);
}

uint16_t bq27427_design_energy(void)
{
    return (bq27427_read_extended_data(BQ27427_ID_STATE, 8) << 8) |
            bq27427_read_extended_data(BQ27427_ID_STATE, 9);
}

bool bq27427_set_design_energy(uint16_t energy)
{
    uint8_t en_msb = energy >> 8;
    uint8_t en_lsb = energy & 0x00FF;
    uint8_t energy_data[2] = {en_msb, en_lsb};
    return bq27427_write_extended_data(BQ27427_ID_STATE, 8, energy_data, 2);
}

uint16_t bq27427_terminate_voltage(void)
{
    return (bq27427_read_extended_data(BQ27427_ID_STATE, 10) << 8) |
            bq27427_read_extended_data(BQ27427_ID_STATE, 11);
}

bool bq27427_set_terminate_voltage(uint16_t voltage)
{
    if (voltage < 2500) voltage = 2500;
    if (voltage > 3700) voltage = 3700;

    uint8_t tv_msb = voltage >> 8;
    uint8_t tv_lsb = voltage & 0x00FF;
    uint8_t tv_data[2] = {tv_msb, tv_lsb};
    return bq27427_write_extended_data(BQ27427_ID_STATE, 10, tv_data, 2);
}

uint16_t bq27427_discharge_current_threshold(void)
{
    return (bq27427_read_extended_data(BQ27427_ID_CURRENT_THRESH, 0) << 8) |
            bq27427_read_extended_data(BQ27427_ID_CURRENT_THRESH, 1);
}

bool bq27427_set_discharge_current_threshold(uint16_t value)
{
    if (value > 2000) value = 2000;

    uint8_t dct_msb = value >> 8;
    uint8_t dct_lsb = value & 0x00FF;
    uint8_t dct_data[2] = {dct_msb, dct_lsb};
    return bq27427_write_extended_data(BQ27427_ID_CURRENT_THRESH, 0, dct_data, 2);
}

uint16_t bq27427_taper_voltage(void)
{
    return (bq27427_read_extended_data(BQ27427_ID_CHEM_DATA, 8) << 8) |
            bq27427_read_extended_data(BQ27427_ID_CHEM_DATA, 9);
}

bool bq27427_set_taper_voltage(uint16_t voltage)
{
    if (voltage > 5000) voltage = 5000;

    uint8_t tv_msb = voltage >> 8;
    uint8_t tv_lsb = voltage & 0x00FF;
    uint8_t tv_data[2] = {tv_msb, tv_lsb};
    return bq27427_write_extended_data(BQ27427_ID_CHEM_DATA, 8, tv_data, 2);
}

uint16_t bq27427_taper_rate(void)
{
    return (bq27427_read_extended_data(BQ27427_ID_STATE, 21) << 8) |
            bq27427_read_extended_data(BQ27427_ID_STATE, 22);
}

bool bq27427_set_taper_rate(uint16_t rate)
{
    if (rate > 2000) rate = 2000;

    uint8_t tr_msb = rate >> 8;
    uint8_t tr_lsb = rate & 0x00FF;
    uint8_t tr_data[2] = {tr_msb, tr_lsb};
    return bq27427_write_extended_data(BQ27427_ID_STATE, 21, tr_data, 2);
}

bool bq27427_current_polarity(void)
{
    uint8_t cal_bit0 = bq27427_read_extended_data(BQ27427_ID_CC_CAL, 5);
    return (cal_bit0 & 0x80) != 0;
}

bool bq27427_change_current_polarity(void)
{
    uint8_t cal_bit0 = bq27427_read_extended_data(BQ27427_ID_CC_CAL, 5);
    cal_bit0 ^= 0x80;
    return bq27427_write_extended_data(BQ27427_ID_CC_CAL, 5, &cal_bit0, 1);
}

bool bq27427_set_current_polarity(int bit)
{
    uint8_t cal_bit0 = bq27427_read_extended_data(BQ27427_ID_CC_CAL, 5);
    if (bit)
        cal_bit0 |= 0x80;
    else
        cal_bit0 &= ~0x80;
    return bq27427_write_extended_data(BQ27427_ID_CC_CAL, 5, &cal_bit0, 1);
}

/******************************************************************************
 * Battery Characteristics Functions
 ******************************************************************************/

uint16_t bq27427_voltage(void)
{
    return bq27427_read_word(BQ27427_COMMAND_VOLTAGE);
}

int16_t bq27427_current(bq27427_current_measure_t type)
{
    int16_t current = 0;

    switch (type) {
        case BQ27427_CURRENT_AVG:
            current = (int16_t)bq27427_read_word(BQ27427_COMMAND_AVG_CURRENT);
            break;
        case BQ27427_CURRENT_STBY:
            current = (int16_t)bq27427_read_word(BQ27427_COMMAND_STDBY_CURRENT);
            break;
        case BQ27427_CURRENT_MAX:
            current = (int16_t)bq27427_read_word(BQ27427_COMMAND_MAX_CURRENT);
            break;
    }

    return current;
}

uint16_t bq27427_capacity(bq27427_capacity_measure_t type)
{
    uint16_t capacity = 0;

    switch (type) {
        case BQ27427_CAPACITY_REMAIN:
            capacity = bq27427_read_word(BQ27427_COMMAND_REM_CAPACITY);
            break;
        case BQ27427_CAPACITY_FULL:
            capacity = bq27427_read_word(BQ27427_COMMAND_FULL_CAPACITY);
            break;
        case BQ27427_CAPACITY_AVAIL:
            capacity = bq27427_read_word(BQ27427_COMMAND_NOM_CAPACITY);
            break;
        case BQ27427_CAPACITY_AVAIL_FULL:
            capacity = bq27427_read_word(BQ27427_COMMAND_AVAIL_CAPACITY);
            break;
        case BQ27427_CAPACITY_REMAIN_F:
            capacity = bq27427_read_word(BQ27427_COMMAND_REM_CAP_FIL);
            break;
        case BQ27427_CAPACITY_REMAIN_UF:
            capacity = bq27427_read_word(BQ27427_COMMAND_REM_CAP_UNFL);
            break;
        case BQ27427_CAPACITY_FULL_F:
            capacity = bq27427_read_word(BQ27427_COMMAND_FULL_CAP_FIL);
            break;
        case BQ27427_CAPACITY_FULL_UF:
            capacity = bq27427_read_word(BQ27427_COMMAND_FULL_CAP_UNFL);
            break;
        case BQ27427_CAPACITY_DESIGN:
            capacity = (bq27427_read_extended_data(BQ27427_ID_STATE, 6) << 8) |
                        bq27427_read_extended_data(BQ27427_ID_STATE, 7);
            break;
    }

    return capacity;
}

int16_t bq27427_power(void)
{
    return (int16_t)bq27427_read_word(BQ27427_COMMAND_AVG_POWER);
}

uint16_t bq27427_soc(bq27427_soc_measure_t type)
{
    uint16_t soc_ret = 0;

    switch (type) {
        case BQ27427_SOC_FILTERED:
            soc_ret = bq27427_read_word(BQ27427_COMMAND_SOC);
            break;
        case BQ27427_SOC_UNFILTERED:
            soc_ret = bq27427_read_word(BQ27427_COMMAND_SOC_UNFL);
            break;
    }

    return soc_ret;
}

uint8_t bq27427_soh(bq27427_soh_measure_t type)
{
    uint16_t soh_raw = bq27427_read_word(BQ27427_COMMAND_SOH);
    uint8_t soh_status = soh_raw >> 8;
    uint8_t soh_percent = soh_raw & 0x00FF;

    if (type == BQ27427_SOH_PERCENT) {
        return soh_percent;
    } else {
        return soh_status;
    }
}

uint16_t bq27427_temperature(void)
{
    return bq27427_read_word(BQ27427_COMMAND_INT_TEMP);
}

/******************************************************************************
 * GPOUT Control Functions
 ******************************************************************************/

bool bq27427_gpout_polarity(void)
{
    uint16_t op_config_register = bq27427_op_config();
    return (op_config_register & BQ27427_OPCONFIG_GPIOPOL) != 0;
}

bool bq27427_set_gpout_polarity(bool active_high)
{
    uint16_t old_op_config = bq27427_op_config();

    if ((active_high && (old_op_config & BQ27427_OPCONFIG_GPIOPOL)) ||
        (!active_high && !(old_op_config & BQ27427_OPCONFIG_GPIOPOL))) {
        return true;
    }

    uint16_t new_op_config = old_op_config;
    if (active_high) {
        new_op_config |= BQ27427_OPCONFIG_GPIOPOL;
    } else {
        new_op_config &= ~(BQ27427_OPCONFIG_GPIOPOL);
    }

    return bq27427_write_op_config(new_op_config);
}

bool bq27427_gpout_function(void)
{
    uint16_t op_config_register = bq27427_op_config();
    return (op_config_register & BQ27427_OPCONFIG_BATLOWEN) != 0;
}

bool bq27427_set_gpout_function(bq27427_gpout_function_t function)
{
    uint16_t old_op_config = bq27427_op_config();

    if ((function && (old_op_config & BQ27427_OPCONFIG_BATLOWEN)) ||
        (!function && !(old_op_config & BQ27427_OPCONFIG_BATLOWEN))) {
        return true;
    }

    uint16_t new_op_config = old_op_config;
    if (function) {
        new_op_config |= BQ27427_OPCONFIG_BATLOWEN;
    } else {
        new_op_config &= ~(BQ27427_OPCONFIG_BATLOWEN);
    }

    return bq27427_write_op_config(new_op_config);
}

uint8_t bq27427_soc1_set_threshold(void)
{
    return bq27427_read_extended_data(BQ27427_ID_DISCHARGE, 0);
}

uint8_t bq27427_soc1_clear_threshold(void)
{
    return bq27427_read_extended_data(BQ27427_ID_DISCHARGE, 1);
}

bool bq27427_set_soc1_thresholds(uint8_t set, uint8_t clear)
{
    uint8_t thresholds[2];
    thresholds[0] = constrain_uint8(set, 0, 100);
    thresholds[1] = constrain_uint8(clear, 0, 100);
    return bq27427_write_extended_data(BQ27427_ID_DISCHARGE, 0, thresholds, 2);
}

uint8_t bq27427_socf_set_threshold(void)
{
    return bq27427_read_extended_data(BQ27427_ID_DISCHARGE, 2);
}

uint8_t bq27427_socf_clear_threshold(void)
{
    return bq27427_read_extended_data(BQ27427_ID_DISCHARGE, 3);
}

bool bq27427_set_socf_thresholds(uint8_t set, uint8_t clear)
{
    uint8_t thresholds[2];
    thresholds[0] = constrain_uint8(set, 0, 100);
    thresholds[1] = constrain_uint8(clear, 0, 100);
    return bq27427_write_extended_data(BQ27427_ID_DISCHARGE, 2, thresholds, 2);
}

bool bq27427_soc_flag(void)
{
    uint16_t flag_state = bq27427_flags();
    return (flag_state & BQ27427_FLAG_SOC1) != 0;
}

bool bq27427_socf_flag(void)
{
    uint16_t flag_state = bq27427_flags();
    return (flag_state & BQ27427_FLAG_SOCF) != 0;
}

bool bq27427_itpor_flag(void)
{
    uint16_t flag_state = bq27427_flags();
    return (flag_state & BQ27427_FLAG_ITPOR) != 0;
}

bool bq27427_fc_flag(void)
{
    uint16_t flag_state = bq27427_flags();
    return (flag_state & BQ27427_FLAG_FC) != 0;
}

bool bq27427_chg_flag(void)
{
    uint16_t flag_state = bq27427_flags();
    return (flag_state & BQ27427_FLAG_CHG) != 0;
}

bool bq27427_dsg_flag(void)
{
    uint16_t flag_state = bq27427_flags();
    return (flag_state & BQ27427_FLAG_DSG) != 0;
}

uint8_t bq27427_soci_delta(void)
{
    return bq27427_read_extended_data(BQ27427_ID_STATE, 26);
}

bool bq27427_set_soci_delta(uint8_t delta)
{
    uint8_t soci = constrain_uint8(delta, 0, 100);
    return bq27427_write_extended_data(BQ27427_ID_STATE, 26, &soci, 1);
}

bool bq27427_pulse_gpout(void)
{
    return bq27427_execute_control_word(BQ27427_CONTROL_PULSE_SOC_INT);
}

void bq27427_gpout_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clock */
    if (BQ27427_GPOUT_PORT == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (BQ27427_GPOUT_PORT == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    /* Configure GPIO pin as input with interrupt */
    GPIO_InitStruct.Pin = BQ27427_GPOUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BQ27427_GPOUT_PORT, &GPIO_InitStruct);
}

GPIO_PinState bq27427_gpout_read(void)
{
    return HAL_GPIO_ReadPin(BQ27427_GPOUT_PORT, BQ27427_GPOUT_PIN);
}

/******************************************************************************
 * Control Sub-commands Functions
 ******************************************************************************/

uint16_t bq27427_device_type(void)
{
    return bq27427_read_control_word(BQ27427_CONTROL_DEVICE_TYPE);
}

bool bq27427_set_chem_id(bq27427_chemistry_t chem_id)
{
    if (bq27427_sealed()) {
        _seal_flag = true;
        bq27427_unseal();
    }

    uint16_t old_chem_id = bq27427_read_control_word(BQ27427_CONTROL_CHEM_ID);

    if (bq27427_execute_control_word(BQ27427_CONTROL_SET_CFGUPDATE)) {
        int16_t timeout = BQ27427_I2C_TIMEOUT;
        while ((timeout--) && (!(bq27427_flags() & BQ27427_FLAG_CFGUPMODE))) {
            HAL_Delay(1);
        }

        if (timeout > 0) {
            if (bq27427_execute_control_word(chem_id)) {
                HAL_Delay(100);
                if (bq27427_soft_reset()) {
                    timeout = BQ27427_I2C_TIMEOUT;
                    while ((timeout--) && ((bq27427_flags() & BQ27427_FLAG_CFGUPMODE))) {
                        HAL_Delay(1);
                    }
                    if (timeout > 0) {
                        uint16_t new_chem_id = bq27427_read_control_word(BQ27427_CONTROL_CHEM_ID);
                        if (new_chem_id != old_chem_id) {
                            if (_seal_flag) bq27427_seal();
                            return true;
                        }
                        return false;
                    }
                }
                return false;
            } else {
                return false;
            }
        }
    }
    return false;
}

bq27427_chemistry_t bq27427_chem_id(void)
{
    uint16_t chem_id = bq27427_read_control_word(BQ27427_CONTROL_CHEM_ID);
    return (bq27427_chemistry_t)chem_id;
}

bool bq27427_enter_config(bool user_control)
{
    if (user_control) _user_config_control = true;

    if (bq27427_sealed()) {
        _seal_flag = true;
        bq27427_unseal();
    }

    if (bq27427_execute_control_word(BQ27427_CONTROL_SET_CFGUPDATE)) {
        int16_t timeout = BQ27427_I2C_TIMEOUT;
        while ((timeout--) && (!(bq27427_flags() & BQ27427_FLAG_CFGUPMODE))) {
            HAL_Delay(1);
        }

        if (timeout > 0) {
            return true;
        }
    }

    return false;
}

bool bq27427_exit_config(bool user_control)
{
    if (user_control) _user_config_control = false;

    if (bq27427_soft_reset()) {
        int16_t timeout = BQ27427_I2C_TIMEOUT;
        while ((timeout--) && ((bq27427_flags() & BQ27427_FLAG_CFGUPMODE))) {
            HAL_Delay(1);
        }
        if (timeout > 0) {
            if (_seal_flag) bq27427_seal();
            return true;
        }
    }
    return false;
}

uint16_t bq27427_flags(void)
{
    return bq27427_read_word(BQ27427_COMMAND_FLAGS);
}

uint16_t bq27427_status(void)
{
    return bq27427_read_control_word(BQ27427_CONTROL_STATUS);
}

bool bq27427_reset(void)
{
    if (!_user_config_control) bq27427_enter_config(false);

    if (bq27427_execute_control_word(BQ27427_CONTROL_RESET)) {
        if (!_user_config_control) bq27427_exit_config(false);
        return true;
    } else {
        return false;
    }
}

/******************************************************************************
 * Private Functions
 ******************************************************************************/

static bool bq27427_sealed(void)
{
    uint16_t stat = bq27427_status();
    return (stat & BQ27427_STATUS_SS) != 0;
}

static bool bq27427_seal(void)
{
    return bq27427_read_control_word(BQ27427_CONTROL_SEALED) != 0;
}

static bool bq27427_unseal(void)
{
    if (bq27427_read_control_word(BQ27427_UNSEAL_KEY)) {
        return bq27427_read_control_word(BQ27427_UNSEAL_KEY) != 0;
    }
    return false;
}

static uint16_t bq27427_op_config(void)
{
    return bq27427_read_extended_data(BQ27427_ID_REGISTERS, 0);
}

static bool bq27427_write_op_config(uint16_t value)
{
    uint8_t op_config_msb = value >> 8;
    uint8_t op_config_lsb = value & 0x00FF;
    uint8_t op_config_data[2] = {op_config_msb, op_config_lsb};

    return bq27427_write_extended_data(BQ27427_ID_REGISTERS, 0, op_config_data, 2);
}

static bool bq27427_soft_reset(void)
{
    return bq27427_execute_control_word(BQ27427_CONTROL_SOFT_RESET);
}

static uint16_t bq27427_read_word(uint16_t sub_address)
{
    uint8_t data[2];
    bq27427_i2c_read_bytes(sub_address, data, 2);
    return ((uint16_t)data[1] << 8) | data[0];
}

static uint16_t bq27427_read_control_word(uint16_t function)
{
    uint8_t sub_command_msb = (function >> 8);
    uint8_t sub_command_lsb = (function & 0x00FF);
    uint8_t command[2] = {sub_command_lsb, sub_command_msb};
    uint8_t data[2] = {0, 0};

    bq27427_i2c_write_bytes(0, command, 2);

    if (bq27427_i2c_read_bytes(0, data, 2)) {
        return ((uint16_t)data[1] << 8) | data[0];
    }

    return 0;
}

static bool bq27427_execute_control_word(uint16_t function)
{
    uint8_t sub_command_msb = (function >> 8);
    uint8_t sub_command_lsb = (function & 0x00FF);
    uint8_t command[2] = {sub_command_lsb, sub_command_msb};

    return bq27427_i2c_write_bytes(0, command, 2);
}

/******************************************************************************
 * Extended Data Commands
 ******************************************************************************/

static bool bq27427_block_data_control(void)
{
    uint8_t enable_byte = 0x00;
    return bq27427_i2c_write_bytes(BQ27427_EXTENDED_CONTROL, &enable_byte, 1);
}

static bool bq27427_block_data_class(uint8_t id)
{
    return bq27427_i2c_write_bytes(BQ27427_EXTENDED_DATACLASS, &id, 1);
}

static bool bq27427_block_data_offset(uint8_t offset)
{
    return bq27427_i2c_write_bytes(BQ27427_EXTENDED_DATABLOCK, &offset, 1);
}

static uint8_t bq27427_block_data_checksum(void)
{
    uint8_t csum;
    bq27427_i2c_read_bytes(BQ27427_EXTENDED_CHECKSUM, &csum, 1);
    return csum;
}

static uint8_t bq27427_read_block_data(uint8_t offset)
{
    uint8_t ret;
    uint8_t address = offset + BQ27427_EXTENDED_BLOCKDATA;
    bq27427_i2c_read_bytes(address, &ret, 1);
    return ret;
}

static bool bq27427_write_block_data(uint8_t offset, uint8_t data)
{
    uint8_t address = offset + BQ27427_EXTENDED_BLOCKDATA;
    return bq27427_i2c_write_bytes(address, &data, 1);
}

static uint8_t bq27427_compute_block_checksum(void)
{
    uint8_t data[32];
    bq27427_i2c_read_bytes(BQ27427_EXTENDED_BLOCKDATA, data, 32);

    uint8_t csum = 0;
    for (int i = 0; i < 32; i++) {
        csum += data[i];
    }
    csum = 255 - csum;

    return csum;
}

static bool bq27427_write_block_checksum(uint8_t csum)
{
    return bq27427_i2c_write_bytes(BQ27427_EXTENDED_CHECKSUM, &csum, 1);
}

static uint8_t bq27427_read_extended_data(uint8_t class_id, uint8_t offset)
{
    uint8_t ret_data = 0;

    if (!_user_config_control) bq27427_enter_config(false);

    if (!bq27427_block_data_control()) {
        return 0;
    }
    if (!bq27427_block_data_class(class_id)) {
        return 0;
    }

    bq27427_block_data_offset(offset / 32);

    bq27427_compute_block_checksum();
    bq27427_block_data_checksum();

    ret_data = bq27427_read_block_data(offset % 32);

    if (!_user_config_control) bq27427_exit_config(false);

    return ret_data;
}

static bool bq27427_write_extended_data(uint8_t class_id, uint8_t offset, uint8_t *data, uint8_t len)
{
    if (len > 32) {
        return false;
    }

    if (!_user_config_control) {
        if (!bq27427_enter_config(false)) {
            return false;
        }
    }

    if (!bq27427_block_data_control()) {
        return false;
    }
    if (!bq27427_block_data_class(class_id)) {
        return false;
    }

    bq27427_block_data_offset(offset / 32);
    bq27427_compute_block_checksum();
    bq27427_block_data_checksum();

    for (int i = 0; i < len; i++) {
        if (!bq27427_write_block_data((offset % 32) + i, data[i])) {
            return false;
        }
    }

    uint8_t new_csum = bq27427_compute_block_checksum();
    if (!bq27427_write_block_checksum(new_csum)) {
        return false;
    }

    if (!_user_config_control) {
        HAL_Delay(10);
        if (!bq27427_exit_config(false)) {
            return false;
        }
    }

    return true;
}

/******************************************************************************
 * I2C Read and Write Routines
 ******************************************************************************/

static bool bq27427_i2c_read_bytes(uint8_t sub_address, uint8_t *dest, uint8_t count)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(_hi2c, BQ27427_I2C_ADDRESS_WRITE, sub_address,
                              I2C_MEMADD_SIZE_8BIT, dest, count, BQ27427_I2C_TIMEOUT);

    return (status == HAL_OK);
}

static bool bq27427_i2c_write_bytes(uint8_t sub_address, uint8_t *src, uint8_t count)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(_hi2c, BQ27427_I2C_ADDRESS_WRITE, sub_address,
                               I2C_MEMADD_SIZE_8BIT, src, count, BQ27427_I2C_TIMEOUT);

    return (status == HAL_OK);
}

/******************************************************************************
 * Utility Functions
 ******************************************************************************/

static uint8_t constrain_uint8(uint8_t value, uint8_t min, uint8_t max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
