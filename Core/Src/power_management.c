/***************************************************************************
 * power_management.c
 * created by Sebastian Forenza 2026
 *
 * Peripheral gating for low-power advertising states.
 *
 * The BLE stack and radio are NOT touched here — they are managed by
 * app_ble.c / the sequencer.  We only gate the "application" peripherals:
 *   - TIM2  (buzzer CH1/CH2, LED red CH3, LED blue CH4)
 *   - TIM16 (LED green CH1)
 *   - I2C1  (accelerometer + fuel gauge)
 *   - USART1 (debug UART)
 *   - GPIO outputs (EN_1, EN_2, GPOUT)
 *   - EXTI on PB2 (accelerometer INT1)
 *
 * Calling PowerMgmt_RestoreAll() re-initialises everything so the rest
 * of the firmware can carry on as if nothing happened.
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
extern UART_HandleTypeDef huart1;

/* ---- Private state ----------------------------------------------------- */
static volatile uint8_t peripherals_gated = 0;   /* 1 = low-power, 0 = running */

/* Forward declarations of init functions defined in main.c               */
/* (They are static there, so we re-declare the ones we need as extern.   *
 *  Alternatively you can remove the 'static' keyword in main.c.)         */
extern void MX_USART1_UART_Init(void);

/* We cannot call the static MX_*_Init() functions from main.c directly.
 * Instead we provide thin wrappers that main.c must expose.  See the
 * "WHAT TO CHANGE IN main.c" comment block at the bottom of this file.  */
extern void MX_I2C1_Reinit(void);
extern void MX_TIM2_Reinit(void);
extern void MX_TIM16_Reinit(void);

/***************************************************************************
 * PRIVATE HELPERS
 ***************************************************************************/

/**
 * @brief Stop all PWM outputs and de-init timers to save power.
 */
static void Gate_Timers(void)
{
    /* Make sure all PWM channels are off first */
    LED_Off();
    BUZZER_Stop();

    /* Stop base timers */
    HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_Base_Stop(&htim16);

    /* De-init to release GPIO back to analog */
    HAL_TIM_Base_DeInit(&htim2);
    HAL_TIM_Base_DeInit(&htim16);

    /* Disable peripheral clocks */
    __HAL_RCC_TIM2_CLK_DISABLE();
    __HAL_RCC_TIM16_CLK_DISABLE();
}

/**
 * @brief De-init I2C1 and set its pins to analog to cut leakage.
 */
static void Gate_I2C(void)
{
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_CLK_DISABLE();

    /* Set PA0 (SCL) and PA1 (SDA) to analog / no-pull */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/**
 * @brief Disable the accelerometer EXTI interrupt so motion
 *        events don't wake the CPU.
 */
static void Gate_AccelInterrupt(void)
{
    /* Reconfigure PB2 (INT1) as plain analog input */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = INT1_Pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(INT1_GPIO_Port, &gpio);
}

/**
 * @brief Keep PB2 (INT1) as rising-edge EXTI so the accelerometer
 *        interrupt still fires.
 */
static void Keep_AccelInterrupt(void)
{
    /* Make sure the EXTI config is correct (it was set in MX_GPIO_Init) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = INT1_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(INT1_GPIO_Port, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(INT1_GPIO_Port, INT1_Pin);
    HAL_NVIC_EnableIRQ(GPIOB_IRQn);
}

/**
 * @brief Set unused GPIO outputs low / analog to prevent leakage.
 */
static void Gate_GPIO_Outputs(void)
{
    /* Drive enable pins low, then switch to analog */
    HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_2_GPIO_Port, EN_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPOUT_GPIO_Port, GPOUT_Pin, GPIO_PIN_RESET);

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    gpio.Pin = EN_1_Pin;
    HAL_GPIO_Init(EN_1_GPIO_Port, &gpio);

    gpio.Pin = EN_2_Pin;
    HAL_GPIO_Init(EN_2_GPIO_Port, &gpio);

    gpio.Pin = GPOUT_Pin;
    HAL_GPIO_Init(GPOUT_GPIO_Port, &gpio);
}

/**
 * @brief Disable USART1 clock (debug UART).
 */
static void Gate_UART(void)
{
    HAL_UART_DeInit(&huart1);
    __HAL_RCC_USART1_CLK_DISABLE();

    /* Set TX/RX pins to analog */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    gpio.Pin = GPIO_PIN_9;          /* PA9  = USART1_TX */
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_14;         /* PB14 = USART1_RX */
    HAL_GPIO_Init(GPIOB, &gpio);
}

/**
 * @brief Restore GPIO outputs to their normal push-pull config.
 */
static void Restore_GPIO_Outputs(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = EN_2_Pin;
    HAL_GPIO_Init(EN_2_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(EN_2_GPIO_Port, EN_2_Pin, GPIO_PIN_RESET);

    gpio.Pin = EN_1_Pin | GPOUT_Pin;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, EN_1_Pin | GPOUT_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Restore the accelerometer EXTI pin.
 */
static void Restore_AccelInterrupt(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = INT1_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(INT1_GPIO_Port, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(INT1_GPIO_Port, INT1_Pin);
    HAL_NVIC_EnableIRQ(GPIOB_IRQn);
}

/***************************************************************************
 * PUBLIC API
 ***************************************************************************/

void PowerMgmt_EnterLowPower_Idle(void)
{
    if (peripherals_gated) return;           /* already gated */

    Gate_Timers();
    Gate_I2C();
    Gate_AccelInterrupt();                   /* no motion wake */
    Gate_GPIO_Outputs();
    Gate_UART();

    peripherals_gated = 1;
}

void PowerMgmt_EnterLowPower_Armed(void)
{
    if (peripherals_gated) return;

    Gate_Timers();
    Gate_I2C();
    Keep_AccelInterrupt();                   /* motion wake stays active */
    Gate_GPIO_Outputs();
    Gate_UART();

    peripherals_gated = 1;
}

void PowerMgmt_RestoreAll(void)
{
    if (!peripherals_gated) return;          /* nothing to do */

    /* --- Re-enable clocks first --- */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* --- Reinitialise peripherals via wrappers in main.c --- */
    MX_I2C1_Reinit();
    MX_TIM2_Reinit();
    MX_TIM16_Reinit();

    /* --- Restore GPIO --- */
    Restore_GPIO_Outputs();
    Restore_AccelInterrupt();

    /* --- Restore UART (optional – remove if you don't need debug) --- */
    MX_USART1_UART_Init();

    /* --- Re-init drivers that depend on I2C --- */
    HAL_Delay(10);                           /* let I2C bus settle */
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
 * WHAT TO CHANGE IN main.c
 * =========================================================================
 *
 * The static MX_*_Init() functions generated by CubeMX can't be called
 * from another .c file.  Add these thin public wrappers at the bottom of
 * main.c (inside a USER CODE BEGIN/END block so CubeMX doesn't delete
 * them):
 *
 *   // ---- In USER CODE BEGIN 4 section of main.c ----
 *
 *   void MX_I2C1_Reinit(void)  { MX_I2C1_Init();  }
 *   void MX_TIM2_Reinit(void)  { MX_TIM2_Init();   }
 *   void MX_TIM16_Reinit(void) { MX_TIM16_Init();  }
 *
 * That's it — everything else is handled here.
 *
 * =========================================================================
 * HOW TO USE IN state_machine.c
 * =========================================================================
 *
 * #include "power_management.h"
 *
 * void State_Disconnected_Idle_Loop() {
 *
 *     if (IS_CHARGING(HAL_GPIO_ReadPin(GPIOB, CHARGE_Pin))) {
 *         // Charging — need full peripherals
 *         if (PowerMgmt_IsLowPower()) PowerMgmt_RestoreAll();
 *         stayAwakeFlag = 1;
 *
 *         if (BATTERY_IsCharging())
 *             LED_Pulse(4000, 255, 100, 0, 60);
 *         else
 *             LED_Pulse(4000, 0, 255, 0, 60);
 *
 *     } else {
 *         stayAwakeFlag = 0;
 *
 *         if (GET_ARMED_BIT(deviceState)) {
 *             // Armed + disconnected: keep accel interrupt
 *             if (!PowerMgmt_IsLowPower())
 *                 PowerMgmt_EnterLowPower_Armed();
 *         } else {
 *             // Idle + disconnected: everything off
 *             if (!PowerMgmt_IsLowPower())
 *                 PowerMgmt_EnterLowPower_Idle();
 *         }
 *     }
 *
 *     if (connectionStatus) {
 *         PowerMgmt_RestoreAll();
 *         StateMachine_ChangeState(STATE_CONNECTED_IDLE);
 *     }
 * }
 *
 * Also guard the battery polling in main.c while(1):
 *
 *     if (HAL_GetTick() - last_battery_check > 1000) {
 *         last_battery_check = HAL_GetTick();
 *         if (!PowerMgmt_IsLowPower()) {
 *             BATTERY_UpdateState();
 *             LOCKSERVICE_SendStatusUpdate();
 *         }
 *     }
 *
 ***************************************************************************/
