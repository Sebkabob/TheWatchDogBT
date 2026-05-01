/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  * Sebastian Forenza
  * The WatchDogBT
  *
  * This code is firmware for the WatchDogBT
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "state_machine.h"
#include "lis2dux12_reg.h"
#include "battery.h"
#include "app_ble.h"
#include "accelerometer.h"
#include "power_management.h"
#include "sound.h"
#include "motion_logger.h"
#include "lights.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "lockservice_app.h"
#include "loyalty.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_BUS hi2c1

/* Set to 1 to force the code-defined CFG_PUBLIC_BD_ADDRESS to overwrite
 * whatever is stored in EEPROM. Leave 0 for normal boots — EEPROM wins.
 * Consumed by app_ble.c via the extern below. */
#define BD_ADDRESS_OVERRIDE 0
const uint8_t bd_address_override = BD_ADDRESS_OVERRIDE;

stmdev_ctx_t dev_ctx;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

PKA_HandleTypeDef hpka;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_RNG_Init(void);
static void MX_PKA_Init(void);
static void MX_RADIO_Init(void);
static void MX_RADIO_TIMER_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
static void MX_GPIO_LowPower_Unused(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Configure unused pins as analog to minimize leakage current.
 *        Call after MX_GPIO_Init() and before entering main loop.
 */
static void MX_GPIO_LowPower_Unused(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    /*
     * PA9 — USART1_TX. Set to analog since UART is disabled in production.
     * STAT (charge status) is now on PA11, configured by MX_GPIO_Init.
     */
    gpio.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &gpio);

    /*
     * PB14 — USART1_RX. Set to analog since UART is disabled.
     */
    gpio.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &gpio);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_RNG_Init();
  MX_PKA_Init();
  MX_RADIO_Init();
  MX_RADIO_TIMER_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  /*
   * DO NOT call MX_USART1_UART_Init() here in production!
   * UART init reconfigures PA9 as AF push-pull, wasting ~100+µA.
   * If you need debug UART, call MX_USART1_UART_Init() manually
   * only while cable is plugged in.
   */

  BUZZER_Init();

  /* === Power up I2C bus BEFORE I2C init === */
  HAL_GPIO_WritePin(I2C_POWER_GPIO_Port, I2C_POWER_Pin, GPIO_PIN_SET);
  HAL_Delay(5); /* let power rail stabilize */

  /* === Keep EEPROM off by default === */
  HAL_GPIO_WritePin(EEPROM_POW_GPIO_Port, EEPROM_POW_Pin, GPIO_PIN_RESET);

  /* === Fix unused pins for low power === */
  MX_GPIO_LowPower_Unused();

  /* All 3 LEDs are now TIM2 HW PWM — no software init needed */
  MotionLogger_Init();
  HAL_Delay(100);
  LIS2DUX12_Init();
  BATTERY_Init();

  /* Safe boot mode: hold while cable plugged in. If the cable stays
   * plugged for 30 consecutive seconds we additionally wipe the loyalty
   * token (recovery hatch for users with a lost owner phone). The wipe
   * fires once per safe-boot session and is announced with an ascending
   * tone so the user knows the reset took effect. */
  if (IS_CABLE_PLUGGED()) {
      BUZZER_Tone(300, 50);
      BUZZER_Tone(200, 30);
      BUZZER_Tone(100, 20);

      uint32_t cable_start_ms   = HAL_GetTick();
      uint8_t  loyalty_wiped    = 0;
      while (IS_CABLE_PLUGGED()) {
          if (!loyalty_wiped &&
              (HAL_GetTick() - cable_start_ms) >= 30000U) {
              if (Loyalty_Wipe()) {
                  /* Distinct "reset" jingle (ascending) so the user
                   * recognises the hatch fired. */
                  BUZZER_Tone(300, 30);
                  HAL_Delay(20);
                  BUZZER_Tone(450, 30);
                  HAL_Delay(20);
                  BUZZER_Tone(600, 60);
              }
              loyalty_wiped = 1;
          }
      }
  }
  /* USER CODE END 2 */

  /* Init code for STM32_BLE */
  MX_APPE_Init(NULL);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  firstBootTone();
  StateMachine_Init();

  /* Application-layer loyalty token: load from EEPROM after BLE_Init
   * has touched the BD-address region, so the two operations don't race
   * on the EEPROM power rail. */
  Loyalty_Init();

  /* Belt-and-braces: clear any pending GPIOB IRQs and force stayAwakeFlag
   * = 0 so nothing pinned during boot blocks DEEPSTOP. */
  NVIC_ClearPendingIRQ(GPIOB_IRQn);
  stayAwakeFlag = 0;

  while (1)
  {
    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */
    static uint32_t last_battery_check = 0;
    if (HAL_GetTick() - last_battery_check > 1000) {
        last_battery_check = HAL_GetTick();
        if (!PowerMgmt_IsLowPower()) {
            BATTERY_UpdateState();
            LOCKSERVICE_SendBatteryDiagnostic();
        }
    }

    /* BLE status update: 20ms (~50Hz) in high-perf mode, 500ms otherwise */
    static uint32_t last_status_send = 0;
    uint32_t status_interval = GET_HIGHPERF_BIT(deviceInfo) ? 20 : 500;
    if (HAL_GetTick() - last_status_send >= status_interval) {
        last_status_send = HAL_GetTick();
        if (!PowerMgmt_IsLowPower()) {
            LOCKSERVICE_SendStatusUpdate();
        }
    }

    StateMachine_Run();

    /* Drain-mode test feature: keep LED white + 100 Hz tone asserted while
     * active; auto-stops when SOC drops to the configured threshold.
     * StateMachine_Run() already calls BUZZER_Update(), so the tone keeps
     * looping without an extra call here. */
    Drain_Tick();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource and SYSCLKDivider
  */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_DIRECT_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_DIRECT_HSE_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_WAIT_STATES_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLK_DIV4;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief PKA Initialization Function
  * @param None
  * @retval None
  */
static void MX_PKA_Init(void)
{

  /* USER CODE BEGIN PKA_Init 0 */

  /* USER CODE END PKA_Init 0 */

  /* USER CODE BEGIN PKA_Init 1 */

  /* USER CODE END PKA_Init 1 */
  hpka.Instance = PKA;
  if (HAL_PKA_Init(&hpka) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN PKA_Init 2 */

  /* USER CODE END PKA_Init 2 */

}

/**
  * @brief RADIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_RADIO_Init(void)
{

  /* USER CODE BEGIN RADIO_Init 0 */

  /* USER CODE END RADIO_Init 0 */

  RADIO_HandleTypeDef hradio = {0};

  /* USER CODE BEGIN RADIO_Init 1 */

  /* USER CODE END RADIO_Init 1 */

  if (__HAL_RCC_RADIO_IS_CLK_DISABLED())
  {
    /* Radio Peripheral reset */
    __HAL_RCC_RADIO_FORCE_RESET();
    __HAL_RCC_RADIO_RELEASE_RESET();

    /* Enable Radio peripheral clock */
    __HAL_RCC_RADIO_CLK_ENABLE();
  }
  hradio.Instance = RADIO;
  HAL_RADIO_Init(&hradio);
  /* USER CODE BEGIN RADIO_Init 2 */

  /* USER CODE END RADIO_Init 2 */

}

/**
  * @brief RADIO_TIMER Initialization Function
  * @param None
  * @retval None
  */
static void MX_RADIO_TIMER_Init(void)
{

  /* USER CODE BEGIN RADIO_TIMER_Init 0 */

  /* USER CODE END RADIO_TIMER_Init 0 */

  RADIO_TIMER_InitTypeDef RADIO_TIMER_InitStruct = {0};

  /* USER CODE BEGIN RADIO_TIMER_Init 1 */

  /* USER CODE END RADIO_TIMER_Init 1 */

  if (__HAL_RCC_RADIO_IS_CLK_DISABLED())
  {
    /* Radio Peripheral reset */
    __HAL_RCC_RADIO_FORCE_RESET();
    __HAL_RCC_RADIO_RELEASE_RESET();

    /* Enable Radio peripheral clock */
    __HAL_RCC_RADIO_CLK_ENABLE();
  }
  /* Wait to be sure that the Radio Timer is active */
  while(LL_RADIO_TIMER_GetAbsoluteTime(WAKEUP) < 0x10);
  RADIO_TIMER_InitStruct.XTAL_StartupTime = 320;
  RADIO_TIMER_InitStruct.enableInitialCalibration = TRUE;
  RADIO_TIMER_InitStruct.periodicCalibrationInterval = 10000;
  HAL_RADIO_TIMER_Init(&RADIO_TIMER_InitStruct);
  /* USER CODE BEGIN RADIO_TIMER_Init 2 */

  /* USER CODE END RADIO_TIMER_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 31;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* Buzzer pin (PB0) is now TIM16_CH1 AF — configured by HAL_TIM_MspPostInit.
   * No manual GPIO setup needed here. BUZZER_Init() ensures PWM is stopped. */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPOUT_GPIO_Port, GPOUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(I2C_POWER_GPIO_Port, I2C_POWER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EEPROM_POW_GPIO_Port, EEPROM_POW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_SWDIO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPOUT_Pin I2C_POWER_Pin */
  GPIO_InitStruct.Pin = GPOUT_Pin|I2C_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : STAT_Pin */
  GPIO_InitStruct.Pin = STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(STAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ACCEL_INT_Pin DEBUG_GPIO_Pin */
  GPIO_InitStruct.Pin = ACCEL_INT_Pin|DEBUG_GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EEPROM_POW_Pin */
  GPIO_InitStruct.Pin = EEPROM_POW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EEPROM_POW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BQ251_PG_Pin */
  GPIO_InitStruct.Pin = BQ251_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BQ251_PG_GPIO_Port, &GPIO_InitStruct);

  /**/
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_2|PWR_GPIO_BIT_11);

  /**/
  HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_8|PWR_GPIO_BIT_10);

  /**/
  HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_B, PWR_GPIO_BIT_6);

  /**/
  HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_A, PWR_GPIO_BIT_8|PWR_GPIO_BIT_10);

  /**/
  HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_6);

  /*RT DEBUG GPIO_Init */
  RT_DEBUG_GPIO_Init();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(GPIOB_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(GPIOB_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* PB5 (DEBUG_GPIO): EXTI removed. Pin is set to ANALOG to eliminate any
   * leakage and any chance of a spurious wake. */
  {
      GPIO_InitTypeDef debug_gpio = {0};
      debug_gpio.Pin  = DEBUG_GPIO_Pin;
      debug_gpio.Mode = GPIO_MODE_ANALOG;
      debug_gpio.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(DEBUG_GPIO_GPIO_Port, &debug_gpio);
  }

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MX_I2C1_Reinit(void)  { MX_I2C1_Init();  }
void MX_TIM2_Reinit(void)  { MX_TIM2_Init();   }
void MX_TIM16_Reinit(void) { MX_TIM16_Init();  }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
