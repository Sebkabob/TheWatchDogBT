/***************************************************************************
 * accelerometer.c
 * created by Sebastian Forenza 2026
 *
 * LIS2DUX12 accelerometer driver — MLC asset tracking mode.
 *
 *   - INT1 (PB15, EXTI rising) fires on MLC state change; only a flag is
 *     set in the ISR — all I2C reads happen in the main loop.
 *   - On DEEPSTOP wake the PWR controller invokes HAL_PWR_WKUPx_Callback
 *     instead of HAL_GPIO_EXTI_Callback.
 ***************************************************************************/

#include "main.h"
#include "state_machine.h"
#include "accelerometer.h"
#include "lis2dux12_app.h"
#include "lis2dux12_reg.h"
#include <string.h>

static volatile uint8_t motion_detected_flag = 0;

/***************************************************************************
 * HAL_GPIO_EXTI_Callback — sets motion flag on PB15 EXTI rising edge
 ***************************************************************************/
void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    if (GPIOx == ACCEL_INT_GPIO_Port && GPIO_Pin == ACCEL_INT_Pin) {
        motion_detected_flag = 1;
        __HAL_GPIO_EXTI_CLEAR_IT(ACCEL_INT_GPIO_Port, ACCEL_INT_Pin);
    }
}

/***************************************************************************
 * HAL_PWR_WKUPx_Callback — DEEPSTOP wakeup dispatcher
 *   On STM32WB0x, DEEPSTOP wakeup goes through PWR (not EXTI), so this
 *   weak override is the only place where PB15 / PB4 wakeups land.
 ***************************************************************************/
void HAL_PWR_WKUPx_Callback(uint32_t WakeupIOs) {
    if (WakeupIOs & PWR_WAKEUP_PB15) {
        motion_detected_flag = 1;
    }
    if (WakeupIOs & PWR_WAKEUP_PB4) {
        CablePlug_IRQCallback();
    }
}

void LIS2DUX12_ClearMotion(void) {
    motion_detected_flag = 0;
}

uint8_t LIS2DUX12_IsMotionDetected(void) {
    if (motion_detected_flag) {
        motion_detected_flag = 0;
        return 1;
    }
    return 0;
}

uint8_t LIS2DUX12_PeekMotionStatus(void) {
    return motion_detected_flag;
}

/***************************************************************************
 * LIS2DUX12_Init — load MLC asset-tracking UCF and prime cached state
 ***************************************************************************/
int32_t LIS2DUX12_Init(void) {
    int ret = lis2dux12_app_init(&hi2c1);
    if (ret != 0) {
        return (int32_t)ret;
    }

    uint8_t mlc_out;
    if (lis2dux12_app_get_mlc_output(&mlc_out) == 0) {
        lis2dux12_app_update_cached_state(mlc_out);
    }

    return 0;
}

/***************************************************************************
 * LIS2DUX12_ConfigureWakeup — arm PB15 as PWR wakeup pin (HIGH polarity)
 ***************************************************************************/
void LIS2DUX12_ConfigureWakeup(void) {
    LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PB15);
    LL_PWR_SetWakeUpPinPolarityHigh(LL_PWR_WAKEUP_PB15);

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF19);   // WUF19 = PB15
    __HAL_GPIO_EXTI_CLEAR_IT(ACCEL_INT_GPIO_Port, ACCEL_INT_Pin);
}

/***************************************************************************
 * LIS2DUX12_ResetAndPowerDown — software-reset then ODR=0
 *   Without the reset, MLC/FSM DSP blocks keep drawing >100 µA even at
 *   ODR=0. The reset clears all register state for a clean ~0.4 µA idle.
 ***************************************************************************/
int32_t LIS2DUX12_ResetAndPowerDown(void)
{
    int32_t ret;

    ret = lis2dux12_init_set(&dev_ctx, LIS2DUX12_RESET);
    if (ret != 0) return ret;

    lis2dux12_status_t status;
    uint32_t timeout = HAL_GetTick() + 100;
    do {
        lis2dux12_status_get(&dev_ctx, &status);
        if (HAL_GetTick() > timeout) return -1;
    } while (status.sw_reset);

    HAL_Delay(5);

    lis2dux12_md_t mode = {
        .odr = LIS2DUX12_OFF,
        .fs  = LIS2DUX12_4g,
        .bw  = LIS2DUX12_ODR_div_2,
    };
    ret = lis2dux12_mode_set(&dev_ctx, &mode);
    if (ret != 0) return ret;

    motion_detected_flag = 0;
    return 0;
}

/***************************************************************************
 * lis2dux12_enter_ulp_wakeup — internal: ULP wake-only mode at <odr>
 *   Software-resets the chip (wipes MLC/FSM), enables the wakeup engine
 *   on INT1 with ~31 mg threshold, latched. Use the public wrappers below.
 ***************************************************************************/
static int32_t lis2dux12_enter_ulp_wakeup(lis2dux12_odr_t odr,
                                          lis2dux12_inact_odr_t inact_odr)
{
    int32_t ret;

    ret = lis2dux12_init_set(&dev_ctx, LIS2DUX12_RESET);
    if (ret != 0) return ret;

    lis2dux12_status_t status;
    uint32_t timeout = HAL_GetTick() + 100;
    do {
        lis2dux12_status_get(&dev_ctx, &status);
        if (HAL_GetTick() > timeout) return -1;
    } while (status.sw_reset);

    HAL_Delay(5);

    lis2dux12_md_t mode = {
        .odr = odr,
        .fs  = LIS2DUX12_2g,
        .bw  = LIS2DUX12_ODR_div_2,
    };
    ret = lis2dux12_mode_set(&dev_ctx, &mode);
    if (ret != 0) return ret;

    // SLEEP_ON drops the chip to inact_odr once motion stops
    lis2dux12_wakeup_config_t wkup_cfg = {0};
    wkup_cfg.wake_ths        = 1;       // ~31 mg threshold (lowest non-zero)
    wkup_cfg.wake_ths_weight = 0;
    wkup_cfg.wake_dur        = LIS2DUX12_1_ODR;
    wkup_cfg.sleep_dur       = 1;
    wkup_cfg.wake_enable     = LIS2DUX12_SLEEP_ON;
    wkup_cfg.inact_odr       = inact_odr;
    ret = lis2dux12_wakeup_config_set(&dev_ctx, wkup_cfg);
    if (ret != 0) return ret;

    lis2dux12_pin_int_route_t int1_route = {0};
    int1_route.wake_up = 1;
    ret = lis2dux12_pin_int1_route_set(&dev_ctx, &int1_route);
    if (ret != 0) return ret;

    // Latched INT keeps INT1 HIGH until the MCU reads sources
    lis2dux12_int_config_t int_cfg = {0};
    int_cfg.int_cfg = LIS2DUX12_INT_LATCHED;
    int_cfg.sleep_status_on_int = 0;
    int_cfg.dis_rst_lir_all_int = 0;
    ret = lis2dux12_int_config_set(&dev_ctx, &int_cfg);
    if (ret != 0) return ret;

    LIS2DUX12_ConfigureWakeup();

    lis2dux12_all_sources_t all_src;
    lis2dux12_all_sources_get(&dev_ctx, &all_src);
    motion_detected_flag = 0;

    return 0;
}

int32_t LIS2DUX12_EnterUltraLowPowerWakeup(void)
{
    return lis2dux12_enter_ulp_wakeup(LIS2DUX12_1Hz6_ULP, LIS2DUX12_ODR_1_6_HZ);
}

int32_t LIS2DUX12_EnterMediumLowPowerWakeup(void)
{
    return lis2dux12_enter_ulp_wakeup(LIS2DUX12_3Hz_ULP, LIS2DUX12_ODR_3_HZ);
}

/***************************************************************************
 * LIS2DUX12_ConfigArmedSleep — arm DEEPSTOP without losing MLC config
 *   Adds the wakeup engine on top of the running UCF so the chip keeps
 *   classifying through MCU sleep at 25 Hz LP. On wake the MLC output
 *   is immediately valid — no UCF reload, no accumulation window.
 ***************************************************************************/
int32_t LIS2DUX12_ConfigArmedSleep(void)
{
    int32_t ret;

    // SLEEP_OFF so MLC keeps running across wake events
    lis2dux12_wakeup_config_t wkup_cfg = {0};
    wkup_cfg.wake_ths        = 1;
    wkup_cfg.wake_ths_weight = 0;
    wkup_cfg.wake_dur        = LIS2DUX12_1_ODR;
    wkup_cfg.sleep_dur       = 0;
    wkup_cfg.wake_enable     = LIS2DUX12_SLEEP_OFF;
    wkup_cfg.inact_odr       = LIS2DUX12_ODR_NO_CHANGE;
    ret = lis2dux12_wakeup_config_set(&dev_ctx, wkup_cfg);
    if (ret != 0) return ret;

    // Preserve existing MLC/FSM routing on INT1, add wake-up
    lis2dux12_pin_int_route_t int1_route;
    ret = lis2dux12_pin_int1_route_get(&dev_ctx, &int1_route);
    if (ret != 0) return ret;
    int1_route.wake_up = 1;
    ret = lis2dux12_pin_int1_route_set(&dev_ctx, &int1_route);
    if (ret != 0) return ret;

    // Latched INT — brief MLC pulses must reach PWR even in DEEPSTOP
    lis2dux12_int_config_t int_cfg = {0};
    int_cfg.int_cfg = LIS2DUX12_INT_LATCHED;
    int_cfg.sleep_status_on_int = 0;
    int_cfg.dis_rst_lir_all_int = 0;
    ret = lis2dux12_int_config_set(&dev_ctx, &int_cfg);
    if (ret != 0) return ret;

    LIS2DUX12_ConfigureWakeup();

    lis2dux12_all_sources_t all_src;
    lis2dux12_all_sources_get(&dev_ctx, &all_src);
    motion_detected_flag = 0;

    return 0;
}

void LIS2DUX12_ReadAcceleration(int16_t accel[3]) {
    uint8_t data[6];
    lis2dux12_read_reg(&dev_ctx, 0x28, data, 6);
    accel[0] = (int16_t)((data[1] << 8) | data[0]);
    accel[1] = (int16_t)((data[3] << 8) | data[2]);
    accel[2] = (int16_t)((data[5] << 8) | data[4]);
}
