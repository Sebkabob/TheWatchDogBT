/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lis2dux12_reg.h"
#include <string.h>
#include <stdio.h>

#define SENSOR_BUS hi2c1

#define LIS2DUX_ADDRESS 0x19
#define BQ25_ADDRESS 0x6A
#define BQ25186_ADDR (0x6A << 1)

static uint32_t pulse_value = 0;
static uint8_t pulse_direction = 1; // 1 = increasing, 0 = decreasing
static uint8_t charging_active = 0;
static uint32_t pulse_step = 5; // Adjust for pulse speed (smaller = slower)

static uint8_t whoamI;

/** Please note that is MANDATORY: return 0 -> no Error.**/
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/** Please note that is MANDATORY: return 0 -> no Error.**/
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
	reg |= 0x80;
	HAL_I2C_Mem_Write(handle, LIS2DUX_ADDRESS, reg, 1, (uint8_t*) bufp, len, 1000); //define 8bit
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
	reg |= 0x80;
	HAL_I2C_Mem_Read(handle, LIS2DUX_ADDRESS, reg, 1, bufp, len, 1000);
}

uint8_t readBQ25186Register(uint8_t regAddr) {
    uint8_t data = 0;

    // Step 1: Set the register pointer (write operation)
    if (HAL_I2C_Master_Transmit(&hi2c1, BQ25186_ADDR, &regAddr, 1, HAL_MAX_DELAY) != HAL_OK) {
        // Error handling
        return 0xFF; // Indicate error
    }

    // Step 2: Read the register data
    if (HAL_I2C_Master_Receive(&hi2c1, BQ25186_ADDR | 0x01, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
        // Error handling
        return 0xFF; // Indicate error
    }

    return data;
}

void BQgetData() {
    readBQ25186Register(0x00); // Read Status Register
    readBQ25186Register(0x00); // Read Status Register
}

void playTone(uint32_t frequency_hz, uint32_t duration_ms, uint32_t system_clock_hz) {
    if (frequency_hz == 0 || system_clock_hz == 0 || duration_ms == 0) return;

    uint32_t cycles_per_half_period = system_clock_hz / (2 * frequency_hz); // CPU cycles for half period
    uint32_t start_time = HAL_GetTick();

    // Play tone for the specified duration
    while (HAL_GetTick() - start_time < duration_ms) {
        for (volatile uint32_t i = 0; i < cycles_per_half_period; i++); // High phase
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);

        for (volatile uint32_t i = 0; i < cycles_per_half_period; i++); // Low phase
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
    }
}

void Handle_Pulsing_LED(void)
{
    if (1)
    {
        if (!charging_active)
        {
            charging_active = 1;
            pulse_value = 0;
            pulse_direction = 1;
        }

        if (pulse_direction)
        {
            pulse_value += pulse_step;
            if (pulse_value >= htim2.Init.Period)
            {
                pulse_value = htim2.Init.Period;
                pulse_direction = 0;
            }
        }
        else
        {
            if (pulse_value >= pulse_step)
            {
                pulse_value -= pulse_step;
            }
            else
            {
                pulse_value = 0;
                pulse_direction = 1;
            }
        }

        // Direct duty cycle (works with OCPolarity = LOW for active low LED)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_value);
    }
    else
    {
        if (charging_active)
        {
            charging_active = 0;
            pulse_value = 0;
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); // LED off
        }
    }
}

/**
 * @brief Creates a continuous soft pulsing effect on an active-low LED (10% to 50% brightness).
 * @param htim Pointer to TIM2 handle.
 * @param pulse_period_ms Duration of one pulse cycle in milliseconds (e.g., 2000 for 2 seconds).
 */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void twiScan(void) {
    for (uint8_t i = 0; i < 128; i++) {
        uint16_t address = (uint16_t)(i << 1);
        if (HAL_I2C_IsDeviceReady(&hi2c1, address, 3, 5) == HAL_OK) {
            //HAL_GPIO_WritePin(GPIOA, LED_Pin, 0); // LED on
            HAL_Delay(1000);
        } else {
        	//HAL_GPIO_WritePin(GPIOA, LED_Pin, 1); // LED on
        }
    }
}

void forceScan(uint8_t address) {
    // Ensure the address is a valid 7-bit I2C address (0x00 to 0x7F)
    if (address > 0x7F) {
        return; // Invalid address, exit
    }

    // Shift the address left by 1 to align for I2C protocol
    uint16_t i2c_address = (uint16_t)(address << 1);

    // Continuously try to communicate with the specified address
    while (1) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, i2c_address, 3, 5) == HAL_OK) {
            // Device acknowledged
            //HAL_GPIO_WritePin(GPIOA, LED_Pin, 0); // Turn on LED
        } else {
            // No acknowledgment
            //HAL_GPIO_WritePin(GPIOA, LED_Pin, 1); // Turn off LED
        }
        // Optional: Add a small delay to prevent overwhelming the I2C bus
        HAL_Delay(1); // 100ms delay, adjust as needed
    }
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  twiScan();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if ((HAL_GPIO_ReadPin(GPIOB, CHARGE_Pin) == GPIO_PIN_SET) || 1){
		  Handle_Pulsing_LED();
		  HAL_Delay(5);
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the SYSCLKSource and SYSCLKDivider
  */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_RC64MPLL_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_WAIT_STATES_1) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : CHARGE_Pin */
  GPIO_InitStruct.Pin = CHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHARGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB7 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_B, PWR_GPIO_BIT_3|PWR_GPIO_BIT_7|PWR_GPIO_BIT_6);

  /**/
  HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_3|PWR_GPIO_BIT_7|PWR_GPIO_BIT_6);

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
