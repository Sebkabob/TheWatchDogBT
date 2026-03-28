/***************************************************************************
 * lis2dux12_app.c
 *
 * MLC asset tracking module for LIS2DUX12.
 * Loads the pre-built UCF configuration that programs the sensor's
 * Machine Learning Core and Finite State Machine to classify:
 *   - Stationary (upright / not upright)
 *   - In motion
 *   - Shaken
 *   - Impact (FSM1)
 *   - Free-fall (FSM2)
 *
 * Sensor config after UCF load: +/-16g, 25 Hz, low-power mode.
 * INT1: pulsed on MLC state change. INT2: pulsed on FSM events.
 ***************************************************************************/

#include "lis2dux12_app.h"
#include "lis2dux12_asset_tracking.h"
#include <string.h>

/***************************************************************************
 * I2C ADDRESS
 *
 * LIS2DUX12_I2C_ADD_H = 0x33 from the PID driver.
 * These defines encode (7-bit addr << 1) | 1, i.e. the read address.
 * STM32 HAL I2C functions accept the device address in this shifted
 * format and handle the R/W bit internally, so we use the define as-is
 * (same convention as accelerometer.c).
 ***************************************************************************/

/***************************************************************************
 * PLATFORM I/O (wraps STM32 HAL I2C)
 ***************************************************************************/
static I2C_HandleTypeDef *app_hi2c;

static int32_t app_platform_write(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
    if (HAL_I2C_Mem_Write(handle, LIS2DUX12_I2C_ADD_H, reg,
                          I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp,
                          len, 1000) != HAL_OK) {
        return -1;
    }
    return 0;
}

static int32_t app_platform_read(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
    if (HAL_I2C_Mem_Read(handle, LIS2DUX12_I2C_ADD_H, reg,
                         I2C_MEMADD_SIZE_8BIT, bufp,
                         len, 1000) != HAL_OK) {
        return -1;
    }
    return 0;
}

static void app_platform_delay(uint32_t millisec)
{
    HAL_Delay(millisec);
}

/***************************************************************************
 * INITIALIZATION
 ***************************************************************************/
int lis2dux12_app_init(I2C_HandleTypeDef *hi2c)
{
    app_hi2c = hi2c;

    /* Set up driver context (reuses the global dev_ctx) */
    dev_ctx.write_reg = app_platform_write;
    dev_ctx.read_reg  = app_platform_read;
    dev_ctx.mdelay    = app_platform_delay;
    dev_ctx.handle    = app_hi2c;

    /* Boot time */
    HAL_Delay(20);

    /* Verify WHO_AM_I */
    uint8_t whoami = 0;
    if (lis2dux12_device_id_get(&dev_ctx, &whoami) != 0) {
        return -1;
    }
    if (whoami != LIS2DUX12_ID) {
        return -1;
    }

    /* Software reset */
    if (lis2dux12_init_set(&dev_ctx, LIS2DUX12_RESET) != 0) {
        return -2;
    }

    /* Wait for reset to complete (poll BOOT bit, timeout ~100ms) */
    lis2dux12_status_t status;
    uint32_t timeout = HAL_GetTick() + 100;
    do {
        lis2dux12_status_get(&dev_ctx, &status);
        if (HAL_GetTick() > timeout) {
            return -2;
        }
    } while (status.sw_reset);

    HAL_Delay(10);

    /* Load the entire UCF configuration (MLC + FSM + sensor setup).
     * The v2.0 format uses mems_conf_op structs with typed operations:
     *   WRITE — write data to register address
     *   DELAY — wait data milliseconds
     *   POLL_RESET — poll register until masked bits are 0        */
    const struct mems_conf_op *conf = lis2dux12_asset_tracking_conf_0;
    uint32_t conf_len = (uint32_t)MEMS_CONF_ARRAY_LEN(lis2dux12_asset_tracking_conf_0);

    for (uint32_t i = 0; i < conf_len; i++) {
        switch (conf[i].type) {
            case MEMS_CONF_OP_TYPE_WRITE: {
                uint8_t val = conf[i].data;
                int32_t ret = lis2dux12_write_reg(&dev_ctx, conf[i].address, &val, 1);
                if (ret != 0) return -3;
                break;
            }
            case MEMS_CONF_OP_TYPE_DELAY:
                HAL_Delay(conf[i].data);
                break;
            case MEMS_CONF_OP_TYPE_READ: {
                uint8_t dummy;
                lis2dux12_read_reg(&dev_ctx, conf[i].address, &dummy, 1);
                break;
            }
            case MEMS_CONF_OP_TYPE_POLL_RESET: {
                uint8_t reg_val;
                uint32_t poll_timeout = HAL_GetTick() + 200;
                do {
                    lis2dux12_read_reg(&dev_ctx, conf[i].address, &reg_val, 1);
                    if (HAL_GetTick() > poll_timeout) return -3;
                } while (reg_val & conf[i].data);
                break;
            }
            case MEMS_CONF_OP_TYPE_POLL_SET: {
                uint8_t reg_val;
                uint32_t poll_timeout = HAL_GetTick() + 200;
                do {
                    lis2dux12_read_reg(&dev_ctx, conf[i].address, &reg_val, 1);
                    if (HAL_GetTick() > poll_timeout) return -3;
                } while ((reg_val & conf[i].data) != conf[i].data);
                break;
            }
            default:
                break;
        }
    }

    return 0;
}

/***************************************************************************
 * MLC OUTPUT
 ***************************************************************************/
int lis2dux12_app_get_mlc_output(uint8_t *mlc_out)
{
    /* lis2dux12_mlc_out_get reads 4 bytes (MLC1-MLC4) with automatic
     * embedded functions memory bank switching. We only need MLC1. */
    uint8_t mlc_buf[4];
    int32_t ret = lis2dux12_mlc_out_get(&dev_ctx, mlc_buf);
    if (ret != 0) {
        return ret;
    }
    *mlc_out = mlc_buf[0]; /* MLC1_SRC */
    return 0;
}

const char* lis2dux12_app_decode_mlc(uint8_t mlc_val)
{
    switch (mlc_val) {
        case MLC_STATE_STATIONARY_UPRIGHT:
            return "Stationary - Upright";
        case MLC_STATE_STATIONARY_NOT_UPRIGHT:
            return "Stationary - Not Upright";
        case MLC_STATE_IN_MOTION:
            return "In Motion";
        case MLC_STATE_SHAKEN:
            return "Shaken";
        default:
            return "Unknown";
    }
}

int lis2dux12_app_mlc_status_changed(void)
{
    lis2dux12_mlc_status_mainpage_t mlc_status;
    if (lis2dux12_mlc_status_get(&dev_ctx, &mlc_status) != 0) {
        return 0;
    }
    return mlc_status.is_mlc1;
}

/***************************************************************************
 * CACHED MLC STATE
 * Avoids I2C reads in the 1-second BLE status notification path.
 * Updated by the state machine after every MLC interrupt read.
 ***************************************************************************/
static uint8_t cached_mlc_state = 0xFF; /* 0xFF = unknown / not yet read */
static uint8_t tilt_detected = 0;       /* Set by state machine when tilt > 15° */

void lis2dux12_app_update_cached_state(uint8_t mlc_out)
{
    switch (mlc_out) {
        case MLC_STATE_STATIONARY_UPRIGHT:     cached_mlc_state = 0; break;
        case MLC_STATE_STATIONARY_NOT_UPRIGHT: cached_mlc_state = 1; break;
        case MLC_STATE_IN_MOTION:              cached_mlc_state = 2; break;
        case MLC_STATE_SHAKEN:                 cached_mlc_state = 3; break;
        default:                               cached_mlc_state = 0xFF; break;
    }
}

uint8_t lis2dux12_app_get_cached_mlc_state(void)
{
    /* Override stationary states with "Tilted" when tilt is detected */
    if (tilt_detected && (cached_mlc_state == 0 || cached_mlc_state == 1)) {
        return 1; /* Tilted (>15 deg from armed position) */
    }
    return cached_mlc_state;
}

void lis2dux12_app_set_tilt_detected(uint8_t tilted)
{
    tilt_detected = tilted;
}

/***************************************************************************
 * FSM EVENTS
 ***************************************************************************/
int lis2dux12_app_check_fsm_events(uint8_t *impact, uint8_t *freefall)
{
    *impact = 0;
    *freefall = 0;

    /* Read FSM status from the main page register (no bank switch needed) */
    lis2dux12_fsm_status_mainpage_t fsm_status;
    int32_t ret = lis2dux12_fsm_status_get(&dev_ctx, &fsm_status);
    if (ret != 0) {
        return ret;
    }

    *impact   = fsm_status.is_fsm1;
    *freefall = fsm_status.is_fsm2;

    return 0;
}
