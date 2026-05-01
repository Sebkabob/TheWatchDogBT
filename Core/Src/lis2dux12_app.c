/***************************************************************************
 * lis2dux12_app.c
 * created by Sebastian Forenza 2026
 *
 * MLC asset-tracking module for the LIS2DUX12. Loads ST's pre-built UCF
 * to program the on-chip Machine Learning Core (motion classification)
 * and Finite State Machine (impact / free-fall detection).
 *
 * MLC classes: Stationary Upright / Stationary Not-Upright / In Motion / Shaken
 * FSM events:  FSM1 = Impact, FSM2 = Free-fall
 *
 * Sensor config after UCF load: ±16 g, 25 Hz, low-power. INT1 pulses on
 * MLC class change; INT2 pulses on FSM events.
 ***************************************************************************/

#include "lis2dux12_app.h"
#include "lis2dux12_asset_tracking.h"
#include <string.h>

// LIS2DUX12_I2C_ADD_H = 0x33 from the PID driver, encodes (7-bit addr << 1) | 1.
// HAL I2C accepts the shifted form and overrides the R/W bit internally.

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
 * lis2dux12_app_init — verify WHO_AM_I, software-reset, replay UCF
 *   The UCF is a v2.0 mems_conf_op stream — WRITE / DELAY / READ /
 *   POLL_RESET / POLL_SET. UCF errors return -3.
 ***************************************************************************/
int lis2dux12_app_init(I2C_HandleTypeDef *hi2c)
{
    app_hi2c = hi2c;

    dev_ctx.write_reg = app_platform_write;
    dev_ctx.read_reg  = app_platform_read;
    dev_ctx.mdelay    = app_platform_delay;
    dev_ctx.handle    = app_hi2c;

    HAL_Delay(20);

    uint8_t whoami = 0;
    if (lis2dux12_device_id_get(&dev_ctx, &whoami) != 0) {
        return -1;
    }
    if (whoami != LIS2DUX12_ID) {
        return -1;
    }

    if (lis2dux12_init_set(&dev_ctx, LIS2DUX12_RESET) != 0) {
        return -2;
    }

    lis2dux12_status_t status;
    uint32_t timeout = HAL_GetTick() + 100;
    do {
        lis2dux12_status_get(&dev_ctx, &status);
        if (HAL_GetTick() > timeout) {
            return -2;
        }
    } while (status.sw_reset);

    HAL_Delay(10);

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
 * lis2dux12_app_get_mlc_output — read MLC1_SRC (motion classification)
 *   The PID helper reads MLC1..MLC4 in one shot with embedded-page bank
 *   switching; only MLC1 is exposed here.
 ***************************************************************************/
int lis2dux12_app_get_mlc_output(uint8_t *mlc_out)
{
    uint8_t mlc_buf[4];
    int32_t ret = lis2dux12_mlc_out_get(&dev_ctx, mlc_buf);
    if (ret != 0) {
        return ret;
    }
    *mlc_out = mlc_buf[0];
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

// Avoids I2C reads in the 1-second BLE status notification path.
static uint8_t cached_mlc_state = 0xFF;
static uint8_t stabilizing_override = 0;

void lis2dux12_app_update_cached_state(uint8_t mlc_out)
{
    switch (mlc_out) {
        case MLC_STATE_STATIONARY_UPRIGHT:     cached_mlc_state = 0; break;
        case MLC_STATE_STATIONARY_NOT_UPRIGHT: cached_mlc_state = 0; break;
        case MLC_STATE_IN_MOTION:              cached_mlc_state = 2; break;
        case MLC_STATE_SHAKEN:                 cached_mlc_state = 3; break;
        default:                               cached_mlc_state = 0xFF; break;
    }
}

uint8_t lis2dux12_app_get_cached_mlc_state(void)
{
    if (stabilizing_override) return CACHED_STATE_STABILIZING;
    return cached_mlc_state;
}

void lis2dux12_app_set_stabilizing(uint8_t on)
{
    stabilizing_override = on ? 1 : 0;
}

/***************************************************************************
 * lis2dux12_app_read_accel_mg — instantaneous X/Y/Z in milli-g
 *   FS/ODR are read back from the device so this works regardless of the
 *   UCF baked into the chip.
 ***************************************************************************/
int lis2dux12_app_read_accel_mg(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg)
{
    lis2dux12_md_t md;
    if (lis2dux12_mode_get(&dev_ctx, &md) != 0) return -1;

    lis2dux12_xl_data_t xl;
    if (lis2dux12_xl_data_get(&dev_ctx, &md, &xl) != 0) return -1;

    *x_mg = (int16_t)xl.mg[0];
    *y_mg = (int16_t)xl.mg[1];
    *z_mg = (int16_t)xl.mg[2];
    return 0;
}

/***************************************************************************
 * lis2dux12_app_check_fsm_events — read FSM status (impact / free-fall)
 ***************************************************************************/
int lis2dux12_app_check_fsm_events(uint8_t *impact, uint8_t *freefall)
{
    *impact = 0;
    *freefall = 0;

    lis2dux12_fsm_status_mainpage_t fsm_status;
    int32_t ret = lis2dux12_fsm_status_get(&dev_ctx, &fsm_status);
    if (ret != 0) {
        return ret;
    }

    *impact   = fsm_status.is_fsm1;
    *freefall = fsm_status.is_fsm2;

    return 0;
}
