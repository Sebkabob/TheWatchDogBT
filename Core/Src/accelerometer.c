/***************************************************************************
 * accelerometer.c
 * created by Sebastian Forenza 2026
 *
 * Functions in charge of interfacing with the
 * LIS2DUX accelerometer IC
 ***************************************************************************/

#include "main.h"
#include "state_machine.h"
#include "accelerometer.h"
#include "lis2dux12_reg.h"
#include "sound.h"
#include "lights.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/***************************************************************************
 * PRIVATE DEFINES
 ***************************************************************************/
#define MOTION_THRESHOLD_LOW      2
#define MOTION_THRESHOLD_MEDIUM   8
#define MOTION_THRESHOLD_HIGH     20

#define LIS2DUX_ADDRESS 0x19
#define LIS2DUX12_I2C_ADDRESS_HIGH  0x19   // When SA0/SDO = VDD

/***************************************************************************
 * PRIVATE VARIABLES
 ***************************************************************************/
static volatile uint8_t motion_detected_flag = 0;

/***************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 ***************************************************************************/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

/***************************************************************************
 * PLATFORM INTERFACE FUNCTIONS
 ***************************************************************************/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    HAL_I2C_Mem_Write(handle, LIS2DUX12_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000);
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    HAL_I2C_Mem_Read(handle, LIS2DUX12_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    return 0;
}

/***************************************************************************
 * INTERRUPT HANDLER
 * PB15 = ACCEL_INT (new PCB)
 ***************************************************************************/
void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    if (GPIOx == ACCEL_INT_GPIO_Port && GPIO_Pin == ACCEL_INT_Pin) {
        /* Read wake-up source to clear the latched interrupt */
        lis2dux12_all_sources_t all_sources;
        lis2dux12_all_sources_get(&dev_ctx, &all_sources);

        if (all_sources.wake_up) {
            motion_detected_flag = 1;
        } else {
            motion_detected_flag = 1; // TEMP: flag any interrupt as motion
        }

        /* Clear EXTI after reading sensor */
        __HAL_GPIO_EXTI_CLEAR_IT(ACCEL_INT_GPIO_Port, ACCEL_INT_Pin);
    }
}

/***************************************************************************
 * PUBLIC API - Motion Detection
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
 * INITIALIZATION
 ***************************************************************************/
int32_t LIS2DUX12_Init(void) {
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &hi2c1;

    HAL_Delay(50);

    uint8_t whoami;
    int32_t ret = lis2dux12_device_id_get(&dev_ctx, &whoami);
    if (ret != 0 || whoami != 0x47) {
        return -1;
    }

    ret = lis2dux12_init_set(&dev_ctx, LIS2DUX12_RESET);
    if (ret != 0) {
        return -1;
    }

    HAL_Delay(50);

    lis2dux12_md_t md = {
        .odr = LIS2DUX12_25Hz_LP,
        .fs = LIS2DUX12_2g,
        .bw = LIS2DUX12_ODR_div_4
    };
    ret = lis2dux12_mode_set(&dev_ctx, &md);
    if (ret != 0) {
        return -1;
    }

    lis2dux12_wakeup_config_t wake_cfg = {
        .wake_enable = LIS2DUX12_SLEEP_ON,
        .wake_ths = MOTION_THRESHOLD_LOW,
        .wake_ths_weight = 0,
        .wake_dur = LIS2DUX12_1_ODR,
        .sleep_dur = 1,
        .inact_odr = LIS2DUX12_ODR_NO_CHANGE
    };
    ret = lis2dux12_wakeup_config_set(&dev_ctx, wake_cfg);
    if (ret != 0) {
        return -1;
    }

    lis2dux12_pin_int_route_t int_route = {0};
    int_route.wake_up = PROPERTY_ENABLE;
    ret = lis2dux12_pin_int1_route_set(&dev_ctx, &int_route);
    if (ret != 0) {
        return -1;
    }

    lis2dux12_int_config_t int_cfg = {
        .int_cfg = LIS2DUX12_DRDY_PULSED,
        .dis_rst_lir_all_int = 0,
        .sleep_status_on_int = 0
    };
    ret = lis2dux12_int_config_set(&dev_ctx, &int_cfg);
    if (ret != 0) {
        return -1;
    }

    return 0;
}

void LIS2DUX12_QuickReinit(void) {
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &hi2c1;

    uint8_t whoami = 0;
    lis2dux12_device_id_get(&dev_ctx, &whoami);

    if (whoami != 0x47) {
        LIS2DUX12_Init();
    } else {
        lis2dux12_int_config_t int_cfg = {
            .int_cfg = LIS2DUX12_DRDY_PULSED,
            .dis_rst_lir_all_int = 0,
            .sleep_status_on_int = 0
        };
        lis2dux12_int_config_set(&dev_ctx, &int_cfg);
    }
}

/***************************************************************************
 * CONFIGURATION FUNCTIONS
 ***************************************************************************/
int32_t LIS2DUX12_SetMotionThreshold(uint8_t threshold) {
    lis2dux12_wakeup_config_t wake_cfg = {
        .wake_enable = LIS2DUX12_SLEEP_ON,
        .wake_ths = threshold,
        .wake_ths_weight = 0,
        .wake_dur = LIS2DUX12_1_ODR,
        .sleep_dur = 1,
        .inact_odr = LIS2DUX12_ODR_NO_CHANGE
    };
    return lis2dux12_wakeup_config_set(&dev_ctx, wake_cfg);
}

/***************************************************************************
 * POWER MANAGEMENT
 * Configure PB15 as a wake-up source from sleep
 ***************************************************************************/
void LIS2DUX12_ConfigureWakeup(void) {
    /* Enable PB15 as wakeup pin with rising edge polarity */
    LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PB15);
    LL_PWR_SetWakeUpPinPolarityHigh(LL_PWR_WAKEUP_PB15);

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF0);
    __HAL_GPIO_EXTI_CLEAR_IT(ACCEL_INT_GPIO_Port, ACCEL_INT_Pin);
}

/***************************************************************************
 * UTILITY FUNCTIONS
 ***************************************************************************/
void LIS2DUX12_ClearAllInterrupts(void) {
    lis2dux12_wake_up_src_t wake_src;
    lis2dux12_read_reg(&dev_ctx, LIS2DUX12_WAKE_UP_SRC, (uint8_t*)&wake_src, 1);

    lis2dux12_all_int_src_t all_int_src;
    lis2dux12_read_reg(&dev_ctx, LIS2DUX12_ALL_INT_SRC, (uint8_t*)&all_int_src, 1);

    lis2dux12_tap_src_t tap_src;
    lis2dux12_read_reg(&dev_ctx, LIS2DUX12_TAP_SRC, (uint8_t*)&tap_src, 1);

    lis2dux12_sixd_src_t sixd_src;
    lis2dux12_read_reg(&dev_ctx, LIS2DUX12_SIXD_SRC, (uint8_t*)&sixd_src, 1);
}

void LIS2DUX12_ReadAcceleration(int16_t accel[3]) {
    uint8_t data[6];
    lis2dux12_read_reg(&dev_ctx, 0x25, data, 6);

    accel[0] = (int16_t)((data[1] << 8) | data[0]);
    accel[1] = (int16_t)((data[3] << 8) | data[2]);
    accel[2] = (int16_t)((data[5] << 8) | data[4]);
}

void LIS2DUX12_I2CScan(void) {
    for (uint8_t i = 0; i < 128; i++) {
        uint16_t address = (uint16_t)(i << 1);
        if (HAL_I2C_IsDeviceReady(&hi2c1, address, 3, 5) == HAL_OK) {
            // Device found at address
        }
    }
}
