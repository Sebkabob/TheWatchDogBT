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

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF0);
    __HAL_GPIO_EXTI_CLEAR_IT(ACCEL_INT_GPIO_Port, ACCEL_INT_Pin);
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
