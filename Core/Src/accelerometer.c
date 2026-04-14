/***************************************************************************
 * accelerometer.c
 * created by Sebastian Forenza 2026
 *
 * LIS2DUX12 accelerometer driver — MLC asset tracking mode.
 * Loads the pre-built UCF configuration via lis2dux12_app module.
 * INT1 (PB15) fires on MLC state changes; MLC/FSM data is read
 * from the main loop (no I2C inside the ISR).
 ***************************************************************************/

#include "main.h"
#include "state_machine.h"
#include "accelerometer.h"
#include "lis2dux12_app.h"
#include "lis2dux12_reg.h"
#include <string.h>

/***************************************************************************
 * PRIVATE VARIABLES
 ***************************************************************************/
static volatile uint8_t motion_detected_flag = 0;

/* Reference gravity vector captured when device is armed */
static int16_t ref_accel[3] = {0, 0, 0};
static uint8_t ref_valid = 0;
static uint8_t tilt_state = 0;  /* hysteresis state: 0 = not tilted, 1 = tilted */

/***************************************************************************
 * INTERRUPT HANDLER
 * PB15 = ACCEL_INT — fires on MLC state change (pulsed, 40 ms).
 * We only set a flag here; all I2C reads happen in the main loop.
 ***************************************************************************/
void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    if (GPIOx == ACCEL_INT_GPIO_Port && GPIO_Pin == ACCEL_INT_Pin) {
        motion_detected_flag = 1;
        __HAL_GPIO_EXTI_CLEAR_IT(ACCEL_INT_GPIO_Port, ACCEL_INT_Pin);
    }
}

/***************************************************************************
 * DEEPSTOP WAKEUP CALLBACK
 * On STM32WB0x, DEEPSTOP wakeup goes through the PWR controller —
 * NOT through EXTI.  So HAL_GPIO_EXTI_Callback never fires.
 * The HAL calls this weak-override after context restore to let us
 * set the flags that the main loop checks.
 ***************************************************************************/
void HAL_PWR_WKUPx_Callback(uint32_t WakeupIOs) {
    if (WakeupIOs & PWR_WAKEUP_PB15) {
        motion_detected_flag = 1;
    }
    if (WakeupIOs & PWR_WAKEUP_PB4) {
        CablePlug_IRQCallback();
    }
}

/***************************************************************************
 * PUBLIC API — Motion Detection
 ***************************************************************************/

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

void LIS2DUX12_ClearMotionFlag(void) {
    motion_detected_flag = 0;
}

/***************************************************************************
 * INITIALIZATION — MLC asset tracking via UCF
 ***************************************************************************/
int32_t LIS2DUX12_Init(void) {
    int ret = lis2dux12_app_init(&hi2c1);
    if (ret != 0) {
        return (int32_t)ret;
    }

    /* Do an initial MLC read so the cached state is valid */
    uint8_t mlc_out;
    if (lis2dux12_app_get_mlc_output(&mlc_out) == 0) {
        lis2dux12_app_update_cached_state(mlc_out);
    }

    return 0;
}

/***************************************************************************
 * POWER MANAGEMENT
 * Configure PB15 as a wake-up source from sleep
 ***************************************************************************/
void LIS2DUX12_ConfigureWakeup(void) {
    LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PB15);
    LL_PWR_SetWakeUpPinPolarityHigh(LL_PWR_WAKEUP_PB15);

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF19);   /* WUF19 = PB15 (was WUF0 = PB0) */
    __HAL_GPIO_EXTI_CLEAR_IT(ACCEL_INT_GPIO_Port, ACCEL_INT_Pin);
}

/**
 * @brief  Power down the LIS2DUX12 completely (ODR=0, ~0.4 µA).
 *         No interrupts will fire. Call BEFORE gating I2C.
 * @return 0 on success, non-zero on I2C error
 */
int32_t LIS2DUX12_PowerDown(void)
{
    lis2dux12_md_t mode = {
        .odr = LIS2DUX12_OFF,
        .fs  = LIS2DUX12_4g,
        .bw  = LIS2DUX12_ODR_div_2,
    };
    return lis2dux12_mode_set(&dev_ctx, &mode);
}

/**
 * @brief  Software-reset the LIS2DUX12 then power down (ODR=0, ~0.4 µA).
 *
 * Unlike plain LIS2DUX12_PowerDown(), this first performs a software
 * reset to clear all MLC/FSM configuration.  With MLC/FSM still loaded,
 * internal DSP blocks can draw >100 µA even at ODR=0.  The reset
 * guarantees the sensor is in a clean, minimal-current state.
 *
 * No wake-up interrupts are configured — use this for idle (no motion
 * detection needed).  Call BEFORE gating I2C.
 *
 * @return 0 on success, non-zero on I2C error
 */
int32_t LIS2DUX12_ResetAndPowerDown(void)
{
    int32_t ret;

    /* 1. Software reset — clears MLC/FSM and all register config */
    ret = lis2dux12_init_set(&dev_ctx, LIS2DUX12_RESET);
    if (ret != 0) return ret;

    lis2dux12_status_t status;
    uint32_t timeout = HAL_GetTick() + 100;
    do {
        lis2dux12_status_get(&dev_ctx, &status);
        if (HAL_GetTick() > timeout) return -1;
    } while (status.sw_reset);

    HAL_Delay(5);

    /* 2. Set ODR=0 (power-down) — sensor draws ~0.4 µA */
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

/**
 * @brief  Reconfigure LIS2DUX12 into ultra-low-power wake-up-only mode.
 *
 * Replaces the full MLC/FSM configuration with a minimal setup:
 *   - 1.6 Hz ultra-low-power ODR (~1.5 µA)
 *   - Hardware wake-up interrupt on INT1 (any significant motion)
 *   - No MLC, no FSM
 *
 * Call BEFORE gating I2C. After waking, call LIS2DUX12_Init() to
 * reload the full MLC asset-tracking configuration.
 *
 * @return 0 on success, non-zero on I2C error
 */
int32_t LIS2DUX12_EnterUltraLowPowerWakeup(void)
{
    int32_t ret;

    /* 1. Software reset to clear MLC/FSM configuration */
    ret = lis2dux12_init_set(&dev_ctx, LIS2DUX12_RESET);
    if (ret != 0) return ret;

    lis2dux12_status_t status;
    uint32_t timeout = HAL_GetTick() + 100;
    do {
        lis2dux12_status_get(&dev_ctx, &status);
        if (HAL_GetTick() > timeout) return -1;
    } while (status.sw_reset);

    HAL_Delay(5);

    /* 2. Set sensor mode: 1.6 Hz ULP, +/-4g (ample for wake detection) */
    lis2dux12_md_t mode = {
        .odr = LIS2DUX12_1Hz6_ULP,
        .fs  = LIS2DUX12_4g,
        .bw  = LIS2DUX12_ODR_div_2,
    };
    ret = lis2dux12_mode_set(&dev_ctx, &mode);
    if (ret != 0) return ret;

    /* 3. Configure wake-up detection:
     *    - threshold ~62.5 mg (wake_ths=1, weight=0 → 1 LSB = FS/64 = 62.5mg)
     *    - wake duration = 1 ODR sample
     *    - sleep enabled so sensor stays in low-current idle until motion */
    lis2dux12_wakeup_config_t wkup_cfg = {0};
    wkup_cfg.wake_ths        = 1;                     /* ~62.5 mg — detect slightest motion */
    wkup_cfg.wake_ths_weight = 0;                     /* coarse: FS/64 per LSB */
    wkup_cfg.wake_dur        = LIS2DUX12_1_ODR;
    wkup_cfg.sleep_dur       = 1;                     /* 512 ODR cycles to re-enter sleep */
    wkup_cfg.wake_enable     = LIS2DUX12_SLEEP_ON;
    wkup_cfg.inact_odr       = LIS2DUX12_ODR_1_6_HZ; /* 1.6 Hz during inactivity */
    ret = lis2dux12_wakeup_config_set(&dev_ctx, wkup_cfg);
    if (ret != 0) return ret;

    /* 4. Route wake-up interrupt to INT1 (PB15) */
    lis2dux12_pin_int_route_t int1_route = {0};
    int1_route.wake_up = 1;
    ret = lis2dux12_pin_int1_route_set(&dev_ctx, &int1_route);
    if (ret != 0) return ret;

    /* 5. Enable interrupts, latched mode.
     *    Latched keeps INT1 HIGH until status is read via I2C,
     *    ensuring the MCU reliably wakes from DEEPSTOP even for
     *    brief motion events.  The interrupt is cleared by
     *    lis2dux12_all_sources_get() before we gate I2C. */
    lis2dux12_int_config_t int_cfg = {0};
    int_cfg.int_cfg = LIS2DUX12_INT_LATCHED;
    int_cfg.sleep_status_on_int = 0;
    int_cfg.dis_rst_lir_all_int = 0;
    ret = lis2dux12_int_config_set(&dev_ctx, &int_cfg);
    if (ret != 0) return ret;

    /* 6. Configure PB15 as DEEPSTOP wakeup source */
    LIS2DUX12_ConfigureWakeup();

    /* Clear any pending interrupt */
    lis2dux12_all_sources_t all_src;
    lis2dux12_all_sources_get(&dev_ctx, &all_src);
    motion_detected_flag = 0;

    return 0;
}

/***************************************************************************
 * UTILITY FUNCTIONS
 ***************************************************************************/
void LIS2DUX12_ClearAllInterrupts(void) {
    /* Read main-page status registers to clear any latched flags */
    lis2dux12_all_sources_t all_sources;
    lis2dux12_all_sources_get(&dev_ctx, &all_sources);
}

void LIS2DUX12_ReadAcceleration(int16_t accel[3]) {
    uint8_t data[6];

    // Read 6 bytes starting from OUT_X_L (0x28)
    // This fills data[0]=0x28, data[1]=0x29, data[2]=0x2A, etc.
    lis2dux12_read_reg(&dev_ctx, 0x28, data, 6);

    // Combine (High << 8) | Low
    accel[0] = (int16_t)((data[1] << 8) | data[0]); // X-axis
    accel[1] = (int16_t)((data[3] << 8) | data[2]); // Y-axis
    accel[2] = (int16_t)((data[5] << 8) | data[4]); // Z-axis
}

/***************************************************************************
 * TILT DETECTION
 * Compares current gravity vector to the reference captured at arming.
 * Uses integer math only (no FPU on Cortex-M0+).
 *
 * Math: |cur - ref|^2 = 2 * |g|^2 * (1 - cos(theta))
 *       For theta > 15 deg:  2*(1 - cos(15)) = 0.0681
 *       So tilt > 15 deg when: diff_sq * 15 > ref_sq
 *       (because 1/0.0681 ~ 14.68, rounded to 15)
 ***************************************************************************/
void LIS2DUX12_CaptureReference(void) {
    LIS2DUX12_ReadAcceleration(ref_accel);
    ref_valid = 1;
    tilt_state = 0;
}

uint8_t LIS2DUX12_CheckTilt(void) {
    if (!ref_valid) return 0;

    int16_t cur[3];
    LIS2DUX12_ReadAcceleration(cur);

    int32_t dx = (int32_t)cur[0] - ref_accel[0];
    int32_t dy = (int32_t)cur[1] - ref_accel[1];
    int32_t dz = (int32_t)cur[2] - ref_accel[2];

    int32_t diff_sq = dx * dx + dy * dy + dz * dz;
    int32_t ref_sq  = (int32_t)ref_accel[0] * ref_accel[0]
                    + (int32_t)ref_accel[1] * ref_accel[1]
                    + (int32_t)ref_accel[2] * ref_accel[2];

    /* Avoid div-by-zero if reference is all zeros */
    if (ref_sq == 0) return 0;

    /* Hysteresis: 15° to enter tilt, 10° to exit.
     *   15°: 2*(1-cos(15)) = 0.0681 → multiplier 15  (1/0.0681 ≈ 14.68)
     *   10°: 2*(1-cos(10)) = 0.0304 → multiplier 33  (1/0.0304 ≈ 32.9)  */
    if (tilt_state == 0) {
        if (diff_sq * 15 > ref_sq) tilt_state = 1;  /* enter at 15° */
    } else {
        if (diff_sq * 33 <= ref_sq) tilt_state = 0;  /* exit at 10° */
    }

    return tilt_state;
}

void LIS2DUX12_I2CScan(void) {
    for (uint8_t i = 0; i < 128; i++) {
        uint16_t address = (uint16_t)(i << 1);
        if (HAL_I2C_IsDeviceReady(&hi2c1, address, 3, 5) == HAL_OK) {
            // Device found at address
        }
    }
}
