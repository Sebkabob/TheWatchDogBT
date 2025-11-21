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
  * probably not MISRA compliant
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
#include "bq25186_reg.h"
#include "battery.h"
#include "sound.h"
#include "lights.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_BUS hi2c1

#define LIS2DUX_ADDRESS 0x19
#define LIS2DUX12_I2C_ADDRESS_HIGH  0x19   // When SA0/SDO = VDD

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
/* USER CODE BEGIN PFP */
/** Please note that is MANDATORY: return 0 -> no Error.**/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t LIS2DUX12_ProperInit(void);
void LIS2DUX12_Test(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LIS2DUX12_ClearAllInterrupts(void) {
    // Clear wake-up interrupt by reading the wake-up source register
    lis2dux12_wake_up_src_t wake_src;
    lis2dux12_read_reg(&dev_ctx, LIS2DUX12_WAKE_UP_SRC, (uint8_t*)&wake_src, 1);

    // Clear all other interrupt sources
    lis2dux12_all_int_src_t all_int_src;
    lis2dux12_read_reg(&dev_ctx, LIS2DUX12_ALL_INT_SRC, (uint8_t*)&all_int_src, 1);

    // Clear tap interrupt
    lis2dux12_tap_src_t tap_src;
    lis2dux12_read_reg(&dev_ctx, LIS2DUX12_TAP_SRC, (uint8_t*)&tap_src, 1);

    // Clear 6D interrupt
    lis2dux12_sixd_src_t sixd_src;
    lis2dux12_read_reg(&dev_ctx, LIS2DUX12_SIXD_SRC, (uint8_t*)&sixd_src, 1);
}

void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    if (GPIOx == GPIOB && GPIO_Pin == GPIO_PIN_0) {
        playTone(1000, 200);  // Indicate wake-up

        // Clear ALL sensor interrupts to reset the INT1 pin
        LIS2DUX12_ClearAllInterrupts();

        // Small delay to ensure interrupt is cleared
        HAL_Delay(1);
    }
}

void twiScan(void) {
    for (uint8_t i = 0; i < 128; i++) {
        uint16_t address = (uint16_t)(i << 1);
        if (HAL_I2C_IsDeviceReady(&hi2c1, address, 3, 5) == HAL_OK) {
        	playTone(address*4, 25);  // Indicate wake-up
        	HAL_Delay(300);
        } else {

        }
    }
}

/** Please note that is MANDATORY: return 0 -> no Error.**/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  HAL_I2C_Mem_Write(handle, LIS2DUX12_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  HAL_I2C_Mem_Read(handle, LIS2DUX12_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

int32_t LIS2DUX12_ProperInit(void) {
    // Initialize device context
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &hi2c1;

    // Check WHO_AM_I register
    uint8_t whoami;
    int32_t ret = lis2dux12_device_id_get(&dev_ctx, &whoami);
    if (ret != 0 || whoami != LIS2DUX12_ID) {
        // Error indication - play error tone
        playTone(200, 300);
        return -1;
    }

    // Success - device found
    playTone(800, 50);

    // Initialize the sensor
    ret = lis2dux12_init_set(&dev_ctx, LIS2DUX12_SENSOR_ONLY_ON);
    if (ret != 0) {
        playTone(300, 300);
        return -1;
    }

    // Clear any existing interrupts before configuration
    LIS2DUX12_ClearAllInterrupts();

    // Set output data rate and power mode for low power operation
    lis2dux12_md_t md = {
        .fs = LIS2DUX12_2g,        // ±2g full scale
        .odr = LIS2DUX12_25Hz_LP,  // 25Hz low power mode
        .bw = LIS2DUX12_ODR_div_2  // Bandwidth ODR/2
    };
    ret = lis2dux12_mode_set(&dev_ctx, &md);
    if (ret != 0) {
        playTone(400, 300);
        return -1;
    }

    // Configure wake-up detection
    lis2dux12_wakeup_config_t wake_cfg = {
        .wake_enable = LIS2DUX12_SLEEP_ON,
        .wake_ths = 8,
        .wake_ths_weight = 0,
        .wake_dur = LIS2DUX12_1_ODR,
        .sleep_dur = 1,
        .inact_odr = LIS2DUX12_ODR_NO_CHANGE
    };
    ret = lis2dux12_wakeup_config_set(&dev_ctx, wake_cfg);
    if (ret != 0) {
        playTone(500, 300);
        return -1;
    }

    // Enable wake-up detection on all axes
    lis2dux12_ctrl1_t ctrl1_reg;
    ret = lis2dux12_read_reg(&dev_ctx, LIS2DUX12_CTRL1, (uint8_t*)&ctrl1_reg, 1);
    if (ret != 0) {
        playTone(600, 300);
        return -1;
    }

    ctrl1_reg.wu_x_en = PROPERTY_ENABLE;
    ctrl1_reg.wu_y_en = PROPERTY_ENABLE;
    ctrl1_reg.wu_z_en = PROPERTY_ENABLE;

    ret = lis2dux12_write_reg(&dev_ctx, LIS2DUX12_CTRL1, (uint8_t*)&ctrl1_reg, 1);
    if (ret != 0) {
        playTone(700, 300);
        return -1;
    }

    // Configure interrupt settings
    lis2dux12_int_config_t int_cfg = {
        .int_cfg = LIS2DUX12_INT_LATCHED,
        .dis_rst_lir_all_int = 0,
        .sleep_status_on_int = 0
    };
    ret = lis2dux12_int_config_set(&dev_ctx, &int_cfg);
    if (ret != 0) {
        playTone(800, 300);
        return -1;
    }

    // Route wake-up interrupt to INT1 pin
    lis2dux12_pin_int_route_t int_route = {0};
    int_route.wake_up = PROPERTY_ENABLE;
    ret = lis2dux12_pin_int1_route_set(&dev_ctx, &int_route);
    if (ret != 0) {
        playTone(900, 300);
        return -1;
    }

    // Configure interrupt pin polarity (active high)
    ret = lis2dux12_int_pin_polarity_set(&dev_ctx, LIS2DUX12_ACTIVE_HIGH);
    if (ret != 0) {
        playTone(1000, 300);
        return -1;
    }

    // Configure pin settings for push-pull output
    lis2dux12_pin_conf_t pin_conf = {
        .int1_int2_push_pull = PROPERTY_ENABLE,
        .int1_pull_down = PROPERTY_DISABLE,
        .int2_pull_down = PROPERTY_ENABLE,
        .cs_pull_up = PROPERTY_ENABLE,
        .sda_pull_up = PROPERTY_DISABLE,
        .sdo_pull_up = PROPERTY_DISABLE
    };
    ret = lis2dux12_pin_conf_set(&dev_ctx, &pin_conf);
    if (ret != 0) {
        playTone(1100, 300);
        return -1;
    }

    // Final interrupt clear after configuration
    LIS2DUX12_ClearAllInterrupts();

    // All initialization successful!
    playTone(1200, 100);
    HAL_Delay(50);
    playTone(1400, 100);

    return 0;
}

void Configure_Wakeup(void) {
    // Enable PB0 as wakeup pin with rising edge polarity
    LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PB0);
    LL_PWR_SetWakeUpPinPolarityHigh(LL_PWR_WAKEUP_PB0);

    // Clear PB0 wakeup flag
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF0);

    // Clear any pending EXTI interrupt - CORRECTED VERSION
    __HAL_GPIO_EXTI_CLEAR_IT(GPIOB, GPIO_PIN_0);
}

void Enter_Sleep_Mode(void) {
    // Clear any pending sensor interrupts before sleep
    LIS2DUX12_ClearAllInterrupts();

    Configure_Wakeup();

    // Properly deinitialize I2C since it will lose power anyway
    HAL_I2C_DeInit(&hi2c1);
    HAL_TIM_Base_Stop(&htim2);

    // Configure DEEPSTOP mode
    PWR_DEEPSTOPTypeDef deepstop_config;
    deepstop_config.deepStopMode = PWR_DEEPSTOP_WITH_SLOW_CLOCK_OFF;
    HAL_PWR_ConfigDEEPSTOP(&deepstop_config);

    // Enter DEEPSTOP mode
    HAL_PWR_EnterDEEPSTOPMode();

    // After wakeup from DEEPSTOP
    playTone(600,50);
    playTone(700,70);
    playTone(800,90);

    // Reinitialize everything
    SystemClock_Config();
    MX_GPIO_Init();  // Reinitialize GPIO including interrupt
    MX_I2C1_Init();

    // Re-initialize the sensor after wake-up
    LIS2DUX12_ProperInit();

    // Clear any interrupts that may have occurred during reinitialization
    LIS2DUX12_ClearAllInterrupts();
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
  /* USER CODE BEGIN 2 */
  // In main(), after MX_TIM2_Init():
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  firstBootTone();
  HAL_Delay(100);
  //twiScan();
  //LIS2DUX12_ProperInit();
  batteryInit();

  // Safe boot mode in case of sleep loop
  if (HAL_GPIO_ReadPin(GPIOB, CHARGE_Pin) == 0){
	  playTone(400,20);
	  playTone(500,30);
	  playTone(600,50);
	  HAL_Delay(15000);
  }
  /* USER CODE END 2 */

  /* Init code for STM32_BLE */
  MX_APPE_Init(NULL);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  StateMachine_Init();
  while (1)
  {
    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */
    StateMachine_Run();

    //sensorUpdates();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  RADIO_TIMER_InitStruct.XTAL_StartupTime = 1000;
  RADIO_TIMER_InitStruct.enableInitialCalibration = FALSE;
  RADIO_TIMER_InitStruct.periodicCalibrationInterval = 0;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : CHARGE_Pin */
  GPIO_InitStruct.Pin = CHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CHARGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_B, PWR_GPIO_BIT_3);

  /*RT DEBUG GPIO_Init */
  RT_DEBUG_GPIO_Init();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
