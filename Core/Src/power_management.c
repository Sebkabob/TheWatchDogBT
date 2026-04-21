/***************************************************************************
 * power_management.c
 * created by Sebastian Forenza 2026
 *
 * Peripheral gating for low-power advertising states.
 *
 * V2 PCB pin changes:
 *   - Buzzer: PB0, TIM16_CH1 HW PWM (was PB6 GPIO toggle)
 *   - LED3:   PB7, TIM2_CH2 HW PWM (was PB1 SW PWM)
 *   - EEPROM: PB6, GPIO output (was PB0)
 *   - STAT:   PA11, GPIO input (was PA9)
 *   - DEBUG:  PB5, EXTI rising, pulldown — holds device awake while HIGH
 *
 * CABLE PLUG WAKEUP:
 *   - PB4 (BQ251_PG) is kept as EXTI falling-edge in ALL low-power modes
 *   - Configured as PWR wakeup pin so it can wake from DEEPSTOP
 *
 * DEBUG GPIO WAKEUP:
 *   - PB5 (DEBUG_GPIO) is kept as EXTI rising-edge in ALL low-power modes
 *   - While PB5 is HIGH, device stays awake (for debugger attachment)
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

    /* Power off the I2C bus, then set the control pin to analog */
    HAL_GPIO_WritePin(I2C_POWER_GPIO_Port, I2C_POWER_Pin, GPIO_PIN_RESET);

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    /* I2C data lines to analog */
    gpio.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* I2C power control pin (PA10) to analog */
    gpio.Pin = I2C_POWER_Pin;
    HAL_GPIO_Init(I2C_POWER_GPIO_Port, &gpio);
}

/**
 * @brief Gate I2C peripheral but keep the I2C bus powered.
 *        Used in armed mode so the accelerometer retains its
 *        ULP wake-up configuration and can still fire INT1.
 */
static void Gate_I2C_KeepPower(void)
{
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_CLK_DISABLE();

    /* I2C data lines to analog — external pull-ups hold them HIGH,
     * no current flows since nothing is driving the bus. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Keep I2C_POWER_Pin (PA10) HIGH so the accelerometer stays powered */
}

static void Gate_EEPROM(void)
{
    HAL_GPIO_WritePin(EEPROM_POW_GPIO_Port, EEPROM_POW_Pin, GPIO_PIN_RESET);

    /* Set EEPROM power pin (PB6) to analog */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = EEPROM_POW_Pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(EEPROM_POW_GPIO_Port, &gpio);
}

/**
 * @brief Stop LED PWM (TIM2) and buzzer (TIM16), de-init both.
 *        Set all LED and buzzer pins to analog to prevent leakage.
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
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
    HAL_TIM_Base_DeInit(&htim16);
    __HAL_RCC_TIM16_CLK_DISABLE();

    /* Set LED pins (PB2, PB3, PB7) and buzzer pin (PB0) to analog.
     * With timer clocks off, AF pins would default LOW which turns on
     * the active-low LEDs — analog mode prevents this. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin  = LED1_Pin | LED2_Pin | LED3_Pin | BUZZ_Pin;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/**
 * @brief Gate USART1 and charger status pin for low power.
 *        PA9/PB14 to analog (UART disabled in production).
 *        PA11 (STAT) to analog — not needed during sleep.
 */
static void Gate_UART(void)
{
    /* Disable USART1 clock if enabled */
    if (__HAL_RCC_USART1_IS_CLK_ENABLED()) {
        __HAL_RCC_USART1_CLK_DISABLE();
    }

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    /* PA9 (USART1_TX) -> analog */
    gpio.Pin  = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PA11 (STAT / charge status) -> analog during sleep */
    gpio.Pin  = STAT_Pin;
    HAL_GPIO_Init(STAT_GPIO_Port, &gpio);

    /* PB14 (USART1_RX) -> analog */
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
     * Polarity LOW = wake when pin goes LOW (cable plugged in). */
    LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PB4);
    LL_PWR_SetWakeUpPinPolarityLow(LL_PWR_WAKEUP_PB4);

    /* Make sure GPIOB NVIC is still enabled (shared with PB15, PB5) */
    HAL_NVIC_EnableIRQ(GPIOB_IRQn);
}

/**
 * @brief Keep PB5 (DEBUG_GPIO) as EXTI rising-edge so debugger can wake device.
 *        Also configure as PWR wakeup pin for DEEPSTOP wakeup.
 */
static void Keep_DebugGPIOInterrupt(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = DEBUG_GPIO_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(DEBUG_GPIO_GPIO_Port, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(DEBUG_GPIO_GPIO_Port, DEBUG_GPIO_Pin);

    /* Enable PB5 as a wakeup source from DEEPSTOP.
     * Polarity HIGH = wake when pin goes HIGH (debugger attached). */
    LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PB5);
    LL_PWR_SetWakeUpPinPolarityHigh(LL_PWR_WAKEUP_PB5);

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
 * @brief Restore PA11 (STAT) as input and UART pins after low power
 */
static void Restore_UART_Pins(void)
{
    /* PA11 -> input for STAT (BQ25186 charge status, open-drain) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = STAT_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(STAT_GPIO_Port, &gpio);

    /* PA9 and PB14 stay analog (UART not used unless explicitly enabled) */
}

/**
 * @brief Restore PB5 (DEBUG_GPIO) as EXTI rising-edge with pulldown after wake
 */
static void Restore_DebugGPIO(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = DEBUG_GPIO_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(DEBUG_GPIO_GPIO_Port, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(DEBUG_GPIO_GPIO_Port, DEBUG_GPIO_Pin);
    HAL_NVIC_EnableIRQ(GPIOB_IRQn);
}

/***************************************************************************
 * PUBLIC API
 ***************************************************************************/

void PowerMgmt_EnterLowPower_Idle(void)
{
    if (peripherals_gated) return;

    /* If debug pin is held HIGH, don't enter low power */
    if (HAL_GPIO_ReadPin(DEBUG_GPIO_GPIO_Port, DEBUG_GPIO_Pin) == GPIO_PIN_SET)
        return;

    Gate_Timers();
    Gate_I2C();              /* kill I2C bus power — accel is off */
    Gate_EEPROM();
    Gate_UART();
    Gate_AccelInterrupt();   /* no motion detection in idle */
    Gate_GPIO_Outputs();
    Keep_CablePlugInterrupt();
    Keep_DebugGPIOInterrupt();

    peripherals_gated = 1;
}

void PowerMgmt_EnterLowPower_Armed(void)
{
    if (peripherals_gated) return;

    /* If debug pin is held HIGH, don't enter low power */
    if (HAL_GPIO_ReadPin(DEBUG_GPIO_GPIO_Port, DEBUG_GPIO_Pin) == GPIO_PIN_SET)
        return;

    Gate_Timers();

    /* Put accel into 1.6 Hz ULP wake-up mode BEFORE gating I2C.
     * IMPORTANT: Use Gate_I2C_KeepPower() — NOT Gate_I2C().
     * Gate_I2C() kills VDD to the accelerometer, destroying the ULP config. */
    LIS2DUX12_EnterUltraLowPowerWakeup();

    Gate_I2C_KeepPower();
    Gate_EEPROM();
    Gate_UART();
    Keep_AccelInterrupt();   /* keep PB15 active for wake-on-motion */
    Gate_GPIO_Outputs();

    /* Keep PB4 (cable detect) and PB5 (debug) active */
    Keep_CablePlugInterrupt();
    Keep_DebugGPIOInterrupt();

    peripherals_gated = 1;
}

void PowerMgmt_RestoreAll(void)
{
    if (!peripherals_gated) return;

    /* --- Restore I2C power pin (PA10) as output FIRST, then drive HIGH --- */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin   = I2C_POWER_Pin;
    HAL_GPIO_Init(I2C_POWER_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(I2C_POWER_GPIO_Port, I2C_POWER_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    /* --- Restore EEPROM power pin (PB6) as output, keep OFF --- */
    gpio.Pin = EEPROM_POW_Pin;
    HAL_GPIO_Init(EEPROM_POW_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(EEPROM_POW_GPIO_Port, EEPROM_POW_Pin, GPIO_PIN_RESET);

    /* --- Re-enable clocks --- */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* --- Reinitialise peripherals --- */
    MX_I2C1_Reinit();
    MX_TIM2_Reinit();    /* also calls HAL_TIM_MspPostInit -> restores PB2/PB3/PB7 AF */
    MX_TIM16_Reinit();   /* also calls HAL_TIM_MspPostInit -> restores PB0 AF */

    /* --- Re-init buzzer safe state --- */
    BUZZER_Init();

    /* --- Restore GPIO --- */
    Restore_GPIO_Outputs();
    Restore_AccelInterrupt();
    Restore_CablePlugInterrupt();
    Restore_UART_Pins();
    Restore_DebugGPIO();

    /* --- Re-init drivers that depend on I2C --- */
    HAL_Delay(10);
    LIS2DUX12_Init();
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
    HAL_GPIO_WritePin(EEPROM_POW_GPIO_Port, EEPROM_POW_Pin, GPIO_PIN_SET);
    HAL_Delay(2);
}

void PowerMgmt_EEPROM_PowerOff(void)
{
    HAL_GPIO_WritePin(EEPROM_POW_GPIO_Port, EEPROM_POW_Pin, GPIO_PIN_RESET);
}
