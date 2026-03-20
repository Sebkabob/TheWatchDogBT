/***************************************************************************
 * power_management.c
 * created by Sebastian Forenza 2026
 *
 * Peripheral gating for low-power advertising states.
 *
 * BUZZER FIX: TIM2 is LED-only. TIM16 is buzzer-only.
 *             Both are gated/restored independently.
 *
 * LOW POWER FIX:
 *   - Also gates USART1 clock if it was left enabled
 *   - Sets PA9 (BQ251_STAT) and PB14 (USART1_RX) to analog during LP
 *   - Restores PA9 as input on wake
 *
 * CABLE PLUG WAKEUP:
 *   - PB4 (BQ251_PG) is kept as EXTI falling-edge in ALL low-power modes
 *   - Configured as PWR wakeup pin so it can wake from DEEPSTOP
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

/**
 * @brief Gate USART1 if its clock is still enabled.
 *        Set PA9 and PB14 to analog to prevent leakage.
 */
static void Gate_UART(void)
{
    /* Disable USART1 clock if enabled */
    if (__HAL_RCC_USART1_IS_CLK_ENABLED()) {
        __HAL_RCC_USART1_CLK_DISABLE();
    }

    /* PA9 (was USART1_TX or BQ251_STAT) -> analog in deep LP */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PB14 (was USART1_RX) -> analog */
    gpio.Pin  = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &gpio);
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

/**
 * @brief Keep PB4 (BQ251_PG) as EXTI falling-edge so it can wake from sleep.
 *        Also configure as PWR wakeup pin for DEEPSTOP wakeup.
 */
static void Keep_CablePlugInterrupt(void)
{
    /* PB4 stays as EXTI falling-edge with pull-up (already configured in MX_GPIO_Init).
     * Just make sure the EXTI line is clear and the NVIC is enabled. */
    __HAL_GPIO_EXTI_CLEAR_IT(BQ251_PG_GPIO_Port, BQ251_PG_Pin);

    /* Enable PB4 as a wakeup source from DEEPSTOP.
     * Polarity LOW = wake when pin goes LOW (cable plugged in).
     * Note: The exact LL_PWR_WAKEUP_PBx define depends on your HAL version.
     * On STM32WB05, PB4 maps to IO9 in the wakeup pin table. */
    LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PB4);
    LL_PWR_SetWakeUpPinPolarityLow(LL_PWR_WAKEUP_PB4);

    /* Make sure GPIOB NVIC is still enabled (shared with PB15) */
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

/**
 * @brief Restore PB4 as EXTI falling-edge after wake
 */
static void Restore_CablePlugInterrupt(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = BQ251_PG_Pin;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BQ251_PG_GPIO_Port, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(BQ251_PG_GPIO_Port, BQ251_PG_Pin);
    HAL_NVIC_EnableIRQ(GPIOB_IRQn);
}

/**
 * @brief Restore PA9 as BQ251_STAT input after low power
 */
static void Restore_UART_Pins(void)
{
    /* PA9 -> input with pull-up for BQ251_STAT (open-drain from BQ25186) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = BQ251_STAT_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BQ251_STAT_GPIO_Port, &gpio);

    /* PB14 -> analog (not used unless UART is explicitly enabled) */
    gpio.Pin  = GPIO_PIN_14;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);
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
    Gate_UART();
    Gate_AccelInterrupt();
    Gate_GPIO_Outputs();

    /* Keep PB4 (cable detect) active so plugging in wakes us */
    Keep_CablePlugInterrupt();

    peripherals_gated = 1;
}

void PowerMgmt_EnterLowPower_Armed(void)
{
    if (peripherals_gated) return;

    Gate_Timers();
    Gate_I2C();
    Gate_EEPROM();
    Gate_UART();
    Keep_AccelInterrupt();
    Gate_GPIO_Outputs();

    /* Keep PB4 (cable detect) active so plugging in wakes us */
    Keep_CablePlugInterrupt();

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
    /* Note: USART1 clock is NOT re-enabled here.
     * UART is debug-only. Call MX_USART1_UART_Init() explicitly if needed. */

    /* --- Reinitialise peripherals --- */
    MX_I2C1_Reinit();
    MX_TIM2_Reinit();
    MX_TIM16_Reinit();

    /* --- Re-init buzzer safe state --- */
    BUZZER_Init();

    /* --- Restore GPIO --- */
    Restore_GPIO_Outputs();
    Restore_AccelInterrupt();
    Restore_CablePlugInterrupt();
    Restore_UART_Pins();

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
