/***************************************************************************
 * power_management.c
 * created by Sebastian Forenza 2026
 *
 * Functions in charge of handling
 * sleep modes and wake-ups
 ***************************************************************************/

#include "main.h"
#include "battery.h"
#include "accelerometer.h"
#include "lis2dux12_reg.h"
#include "app_ble.h"

/* External declarations */
extern stmdev_ctx_t dev_ctx;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

/* Forward declarations of functions from main.c */
void SystemClock_Config(void);

/* Forward declarations of wrapper functions (to be added in main.c) */
void Reinitialize_Peripherals_After_Wakeup(void);

/**
 * @brief Configure all GPIO pins for lowest power consumption
 * IMPORTANT: PB0 is NOT changed - must remain as interrupt input for wakeup
 */
void Configure_GPIO_For_LowPower(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure all unused pins as analog to reduce power */

    // GPIOA - Configure unused pins as analog
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    // PA0, PA1 are I2C - keep as is (will be handled by I2C deinit)
    // PA8 is LED - set as analog to turn off
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure other unused GPIOA pins as analog
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
                          GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                          GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                          GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // GPIOB Configuration
    // PB0 is accelerometer interrupt - MUST NOT BE CHANGED (leave as EXTI rising edge)
    // PB3 is charge detect - configure as input with pullup
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PB6, PB7 are buzzers - set as analog to turn off
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure other unused GPIOB pins as analog
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 |
                          GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9 |
                          GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                          GPIO_PIN_13 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Disable internal pull-ups/pull-downs in deepstop for all pins */
    HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_A, 0xFFFF);
    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_A, 0xFFFF);
    HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_B, 0xFFFF);
    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_B, 0xFFFF);

    /* Re-enable only the pull-down on PB0 (accelerometer interrupt) */
    HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_0);
}

/**
 * @brief Put BQ25186 into lowest power mode
 */
void BQ25186_EnterLowPower(void)
{
    uint8_t reg_value;

    /* Read current VBAT_CTRL register */
    HAL_I2C_Mem_Read(&hi2c1, 0x6A << 1, 0x03,
                     I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);

    /* Ensure HIZ mode is not enabled (we want normal low-power operation) */
    HAL_I2C_Mem_Read(&hi2c1, 0x6A << 1, 0x04,
                     I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
    reg_value &= ~(0x80);  // Clear HIZ bit
    HAL_I2C_Mem_Write(&hi2c1, 0x6A << 1, 0x04,
                      I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
}

/**
 * @brief Put LIS2DUX12 into ultra-low power mode
 * Ensure pulsed interrupt mode is enabled for clean wakeup
 */
void LIS2DUX12_EnterLowPower(void)
{
    /* Already configured for low power in your LIS2DUX12_Init() */
    /* Current draw: ~1.2µA at 25Hz low power mode */

    /* CRITICAL: Ensure pulsed interrupt mode is set */
    lis2dux12_int_config_t int_cfg = {
        .int_cfg = LIS2DUX12_DRDY_PULSED,  // ← IMPORTANT: Pulsed mode for clean wakeup
        .dis_rst_lir_all_int = 0,
        .sleep_status_on_int = 0
    };
    lis2dux12_int_config_set(&dev_ctx, &int_cfg);
}

/**
 * @brief Disable all unnecessary peripherals before sleep
 */
void Disable_Peripherals_For_Sleep(void)
{
    /* Stop and deinitialize peripherals */
    HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

    /* Deinitialize I2C - will be reinitialized on wake */
    HAL_I2C_DeInit(&hi2c1);

    /* Disable peripheral clocks */
    __HAL_RCC_TIM2_CLK_DISABLE();
    __HAL_RCC_I2C1_CLK_DISABLE();
    __HAL_RCC_USART1_CLK_DISABLE();
    __HAL_RCC_RNG_CLK_DISABLE();
    __HAL_RCC_PKA_CLK_DISABLE();
}

/**
 * @brief Configure wakeup source (PB0 from accelerometer)
 * This is critical for waking from deep stop on interrupt
 */
void Configure_Wakeup_Optimized(void)
{
    /* Clear all pending wakeup flags */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF0);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF1);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF3);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF4);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF5);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF6);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF7);

    /* Clear EXTI pending bit for PB0 */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIOB, GPIO_PIN_0);

    /* Enable PB0 as wakeup pin with rising edge (active high from accelerometer) */
    LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PB0);
    LL_PWR_SetWakeUpPinPolarityHigh(LL_PWR_WAKEUP_PB0);
}

/**
 * @brief Enter ultra-low power DEEPSTOP mode
 * MODIFIED: Does NOT clear accelerometer interrupts - allows immediate wake on next interrupt
 */
void Enter_DeepStop_Mode(void)
{
    /* IMPORTANT: DO NOT clear sensor interrupts before sleeping!
     * This allows any pending or new interrupt to wake the system immediately
     * If we cleared them here, we'd miss the interrupt that should wake us up
     */
    // REMOVED: lis2dux12_wake_up_src_t wake_src;
    // REMOVED: lis2dux12_read_reg(&dev_ctx, LIS2DUX12_WAKE_UP_SRC, (uint8_t*)&wake_src, 1);
    // REMOVED: lis2dux12_all_int_src_t all_int_src;
    // REMOVED: lis2dux12_read_reg(&dev_ctx, LIS2DUX12_ALL_INT_SRC, (uint8_t*)&all_int_src, 1);

    /* Put peripherals in low power mode */
    LIS2DUX12_EnterLowPower();
    BQ25186_EnterLowPower();

    /* Configure GPIO for minimum power */
    Configure_GPIO_For_LowPower();

    /* Disable unnecessary peripherals */
    Disable_Peripherals_For_Sleep();

    /* Configure wakeup sources (PB0 rising edge) */
    Configure_Wakeup_Optimized();

    /* Configure DEEPSTOP mode with slowclock off for lowest power */
    PWR_DEEPSTOPTypeDef deepstop_config;
    deepstop_config.deepStopMode = PWR_DEEPSTOP_WITH_SLOW_CLOCK_OFF;
    HAL_PWR_ConfigDEEPSTOP(&deepstop_config);

    /* Enter DEEPSTOP mode - system will wake on PB0 rising edge */
    HAL_PWR_EnterDEEPSTOPMode();

    /* ---- Execution resumes here after wakeup ---- */
    /* System wakes up when PB0 goes high (accelerometer interrupt) */

    /* System will be reinitialized in main.c after wakeup */
}

/**
 * @brief Reinitialize system after wakeup from DEEPSTOP
 */
void Wakeup_System_Init(void)
{
    /* Reconfigure system clock */
    SystemClock_Config();

    /* CRITICAL: Give system time to stabilize after clock config */
    HAL_Delay(50);

    /* Reinitialize all peripherals using wrapper function in main.c */
    /* This also reinitializes I2C, RADIO, BLE, etc. */
    Reinitialize_Peripherals_After_Wakeup();

    /* IMPORTANT: Wait for I2C to stabilize after reinit */
    HAL_Delay(20);

    /* NOW it's safe to clear accelerometer interrupts */
    LIS2DUX12_ClearAllInterrupts();

    /* Reinitialize sensor context */
    LIS2DUX12_QuickReinit();

    /* Clear the EXTI line and motion flag */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIOB, GPIO_PIN_0);
    LIS2DUX12_ClearMotion();

    /* Re-enable EXTI interrupt */
    HAL_NVIC_EnableIRQ(GPIOB_IRQn);

    /* Play debug tone to confirm full reinitialization */
    HAL_Delay(100);
    extern TIM_HandleTypeDef htim2;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

/**
 * @brief Check if system should go to sleep
 * @return 1 if should sleep, 0 if should stay awake
 */
uint8_t Should_Enter_Sleep(void)
{
    /* Don't sleep if charging */
    if (HAL_GPIO_ReadPin(GPIOB, CHARGE_Pin) == GPIO_PIN_RESET) {
        return 0;  // Charging - stay awake to monitor
    }

    /* Don't sleep if BLE connected */
    if (APP_BLE_Get_Server_Connection_Status() == APP_BLE_CONNECTED_SERVER) {
        return 0;  // BLE connected - stay awake
    }

    /* Add other conditions as needed */

    return 1;  // OK to sleep
}
