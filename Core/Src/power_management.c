/***************************************************************************
 * power_management.c
 * created by Sebastian Forenza 2026
 *
 * Peripheral gating for low-power advertising states.
 *
 * NEW PCB pin management:
 *   - I2C_POWER (PA10): Must be HIGH for any I2C (accel + fuel gauge).
 *                        Drive LOW to cut I2C bus power and save current.
 *   - EEPROM_POWER (PB0): Must be HIGH for EEPROM access.
 *                          Drive LOW when not in use.
 *   - ACCEL_INT (PB15): Accelerometer interrupt, EXTI rising edge.
 *
 * The BLE stack and radio are NOT touched here.
 ***************************************************************************/

#include "main.h"
#include "power_management.h"
#include "accelerometer.h"
#include "battery.h"
#include "lights.h"
#include "sound.h"
#include "motion_logger.h"

/* ---- External handles from main.c -------------------------------------- */
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim2;

/* ---- Private state ----------------------------------------------------- */
static volatile uint8_t peripherals_gated = 0;

/* ---- Reinit wrappers defined in main.c ---- */
extern void MX_I2C1_Reinit(void);
extern void MX_TIM2_Reinit(void);

/***************************************************************************
 * PRIVATE HELPERS
 ***************************************************************************/

/**
 * @brief Turn off I2C bus power (PA10 LOW), then de-init I2C peripheral.
 *        Sets SDA/SCL pins to analog to prevent leakage.
 */
static void Gate_I2C(void)
{
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_CLK_DISABLE();

    /* Cut power to I2C bus */
    HAL_GPIO_WritePin(I2C_POWER_GPIO_Port, I2C_POWER_Pin, GPIO_PIN_RESET);

    /* Set PA0 (SCL) and PA1 (SDA) to analog / no-pull */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/**
 * @brief Turn off EEPROM power (PB0 LOW).
 */
static void Gate_EEPROM(void)
{
    HAL_GPIO_WritePin(EEPROM_POWER_GPIO_Port, EEPROM_POWER_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Stop all PWM outputs and de-init timers to save power.
 *        This covers TIM2 (buzzer, red LED, green LED).
 *        Blue LED (soft PWM on PB1) is turned off via LED_Off().
 */
static void Gate_Timers(void)
{
    LED_Off();
    BUZZER_Stop();

    HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_Base_DeInit(&htim2);
    __HAL_RCC_TIM2_CLK_DISABLE();
}

/**
 * @brief Disable the accelerometer EXTI interrupt (PB15 -> analog).
 */
static void Gate_AccelInterrupt(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = ACCEL_INT_Pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ACCEL_INT_GPIO_Port, &gpio);
}

/**
 * @brief Keep PB15 as rising-edge EXTI so the accelerometer
 *        interrupt still fires (for armed mode).
 */
static void Keep_AccelInterrupt(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = ACCEL_INT_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ACCEL_INT_GPIO_Port, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(ACCEL_INT_GPIO_Port, ACCEL_INT_Pin);
    HAL_NVIC_EnableIRQ(GPIOB_IRQn);
}

/**
 * @brief Set misc GPIO outputs low / analog to prevent leakage.
 */
static void Gate_GPIO_Outputs(void)
{
    HAL_GPIO_WritePin(GPOUT_GPIO_Port, GPOUT_Pin, GPIO_PIN_RESET);

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    gpio.Pin = GPOUT_Pin;
    HAL_GPIO_Init(GPOUT_GPIO_Port, &gpio);
}

/**
 * @brief Restore misc GPIO outputs to push-pull.
 */
static void Restore_GPIO_Outputs(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = GPOUT_Pin;
    HAL_GPIO_Init(GPOUT_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(GPOUT_GPIO_Port, GPOUT_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Restore the accelerometer EXTI pin (PB15).
 */
static void Restore_AccelInterrupt(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = ACCEL_INT_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ACCEL_INT_GPIO_Port, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(ACCEL_INT_GPIO_Port, ACCEL_INT_Pin);
    HAL_NVIC_EnableIRQ(GPIOB_IRQn);
}

/***************************************************************************
 * PUBLIC API
 ***************************************************************************/

void PowerMgmt_EnterLowPower_Idle(void)
{
    if (peripherals_gated) return;

    Gate_Timers();
    Gate_I2C();
    Gate_EEPROM();
    Gate_AccelInterrupt();   /* no motion wake */
    Gate_GPIO_Outputs();

    peripherals_gated = 1;
}

void PowerMgmt_EnterLowPower_Armed(void)
{
    if (peripherals_gated) return;

    Gate_Timers();
    Gate_I2C();
    Gate_EEPROM();
    Keep_AccelInterrupt();   /* motion wake stays active */
    Gate_GPIO_Outputs();

    peripherals_gated = 1;
}

void PowerMgmt_RestoreAll(void)
{
    if (!peripherals_gated) return;

    /* --- Power up I2C bus FIRST (PA10 HIGH) --- */
    HAL_GPIO_WritePin(I2C_POWER_GPIO_Port, I2C_POWER_Pin, GPIO_PIN_SET);
    HAL_Delay(5);  /* let power rail stabilize */

    /* --- Re-enable clocks --- */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* --- Reinitialise peripherals --- */
    MX_I2C1_Reinit();
    MX_TIM2_Reinit();

    /* --- Restore GPIO --- */
    Restore_GPIO_Outputs();
    Restore_AccelInterrupt();

    /* --- Re-init drivers that depend on I2C --- */
    HAL_Delay(10);
    LIS2DUX12_QuickReinit();
    LIS2DUX12_ClearMotion();
    BATTERY_Init();

    peripherals_gated = 0;
}

uint8_t PowerMgmt_IsLowPower(void)
{
    return peripherals_gated;
}

/***************************************************************************
 * EEPROM POWER HELPERS (for future use)
 ***************************************************************************/

/**
 * @brief Turn on EEPROM power (PB0 HIGH). Call before EEPROM access.
 */
void PowerMgmt_EEPROM_PowerOn(void)
{
    HAL_GPIO_WritePin(EEPROM_POWER_GPIO_Port, EEPROM_POWER_Pin, GPIO_PIN_SET);
    HAL_Delay(2);  /* let EEPROM power up */
}

/**
 * @brief Turn off EEPROM power (PB0 LOW). Call after EEPROM access.
 */
void PowerMgmt_EEPROM_PowerOff(void)
{
    HAL_GPIO_WritePin(EEPROM_POWER_GPIO_Port, EEPROM_POWER_Pin, GPIO_PIN_RESET);
}
