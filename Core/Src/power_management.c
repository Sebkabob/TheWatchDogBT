/***************************************************************************
 * power_management.c
 * created by Sebastian Forenza 2026
 *
 * Peripheral gating for low-power advertising states.
 *
 * BUZZER FIX: TIM2 is LED-only. TIM16 is buzzer-only.
 *             Both are gated/restored independently.
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
extern TIM_HandleTypeDef  htim16;

/* ---- Private state ----------------------------------------------------- */
static volatile uint8_t peripherals_gated = 0;

/* ---- Reinit wrappers defined in main.c ---- */
extern void MX_I2C1_Reinit(void);
extern void MX_TIM2_Reinit(void);
extern void MX_TIM16_Reinit(void);

/***************************************************************************
 * PRIVATE HELPERS
 ***************************************************************************/

static void Gate_I2C(void)
{
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_CLK_DISABLE();

    HAL_GPIO_WritePin(I2C_POWER_GPIO_Port, I2C_POWER_Pin, GPIO_PIN_RESET);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void Gate_EEPROM(void)
{
    HAL_GPIO_WritePin(EEPROM_POWER_GPIO_Port, EEPROM_POWER_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Stop LED PWM (TIM2) and buzzer (TIM16), de-init both.
 */
static void Gate_Timers(void)
{
    /* Stop LEDs */
    LED_Off();

    /* Stop buzzer */
    BUZZER_Stop();

    /* De-init TIM2 (LEDs) */
    HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_Base_DeInit(&htim2);
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* De-init TIM16 (buzzer) */
    HAL_TIM_Base_Stop_IT(&htim16);
    HAL_TIM_Base_DeInit(&htim16);
    __HAL_RCC_TIM16_CLK_DISABLE();

    /* Ensure PB6 is LOW (MOSFET off) */
    HAL_GPIO_WritePin(BUZZ_1_GPIO_Port, BUZZ_1_Pin, GPIO_PIN_RESET);
}

static void Gate_AccelInterrupt(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = ACCEL_INT_Pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ACCEL_INT_GPIO_Port, &gpio);
}

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

static void Gate_GPIO_Outputs(void)
{
    HAL_GPIO_WritePin(GPOUT_GPIO_Port, GPOUT_Pin, GPIO_PIN_RESET);

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    gpio.Pin = GPOUT_Pin;
    HAL_GPIO_Init(GPOUT_GPIO_Port, &gpio);
}

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
    Gate_AccelInterrupt();
    Gate_GPIO_Outputs();

    peripherals_gated = 1;
}

void PowerMgmt_EnterLowPower_Armed(void)
{
    if (peripherals_gated) return;

    Gate_Timers();
    Gate_I2C();
    Gate_EEPROM();
    Keep_AccelInterrupt();
    Gate_GPIO_Outputs();

    peripherals_gated = 1;
}

void PowerMgmt_RestoreAll(void)
{
    if (!peripherals_gated) return;

    /* --- Power up I2C bus FIRST (PA10 HIGH) --- */
    HAL_GPIO_WritePin(I2C_POWER_GPIO_Port, I2C_POWER_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    /* --- Re-enable clocks --- */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* --- Reinitialise peripherals --- */
    MX_I2C1_Reinit();
    MX_TIM2_Reinit();
    MX_TIM16_Reinit();

    /* --- Re-init buzzer safe state --- */
    BUZZER_Init();

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
 * EEPROM POWER HELPERS
 ***************************************************************************/

void PowerMgmt_EEPROM_PowerOn(void)
{
    HAL_GPIO_WritePin(EEPROM_POWER_GPIO_Port, EEPROM_POWER_Pin, GPIO_PIN_SET);
    HAL_Delay(2);
}

void PowerMgmt_EEPROM_PowerOff(void)
{
    HAL_GPIO_WritePin(EEPROM_POWER_GPIO_Port, EEPROM_POWER_Pin, GPIO_PIN_RESET);
}
