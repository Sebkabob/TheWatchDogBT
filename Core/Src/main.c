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
  * Not MISRA compliant
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lis2dux12_reg.h"
#include "bq25186_reg.h"
#include "buzzer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define SENSOR_BUS hi2c1

#define LIS2DUX_ADDRESS 0x19
#define LIS2DUX12_I2C_ADDRESS_LOW   0x18   // When SA0/SDO = GND
#define LIS2DUX12_I2C_ADDRESS_HIGH  0x19   // When SA0/SDO = VDD
#define BQ25_ADDRESS 0x6A
#define BQ25186_ADDR (0x6A << 1)

static uint32_t pulse_value = 0;
static uint8_t pulse_direction = 1; // 1 = increasing, 0 = decreasing
static uint32_t pulse_step = 5; // Adjust for pulse speed (smaller = slower)

stmdev_ctx_t dev_ctx;
static uint8_t lis2dux12_address = 0x19; // Start with your current address

/** Please note that is MANDATORY: return 0 -> no Error.**/
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
HAL_StatusTypeDef LIS2DUX12_PowerUp(void);
HAL_StatusTypeDef LIS2DUX12_Init(void);
HAL_StatusTypeDef LIS2DUX12_ReadAccel(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg);
void LIS2DUX12_Test(void);
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
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(handle, (lis2dux12_address << 1), reg,
                                                 I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000);
    return (status == HAL_OK) ? 0 : 1;  // Return 0 for success, 1 for error
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle, (lis2dux12_address << 1), reg,
                                                I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    return (status == HAL_OK) ? 0 : 1;  // Return 0 for success, 1 for error
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

///////////////////Claude slop/////////////////////////////
// Power-up sequence for LIS2DUX12
HAL_StatusTypeDef LIS2DUX12_PowerUp(void) {
    HAL_StatusTypeDef status;
    uint8_t dummy_data = 0;

    // Step 1: Send power-up command - this will NACK (expected behavior)
    status = HAL_I2C_Master_Transmit(&hi2c1, (lis2dux12_address << 1), &dummy_data, 0, 100);
    // Don't check status here - NACK is expected

    // Step 2: Wait 25ms for power-up completion
    HAL_Delay(25);

    // Step 3: Try to read WHO_AM_I to verify communication
    uint8_t who_am_i = 0;
    status = HAL_I2C_Mem_Read(&hi2c1, (lis2dux12_address << 1), LIS2DUX12_WHO_AM_I,
                              I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 1000);

    if (status != HAL_OK) {
        return status;
    }

    if (who_am_i != LIS2DUX12_ID) {  // Should be 0x47
        return HAL_ERROR;
    }

    return HAL_OK;
}

// LIS2DUX12 initialization function
HAL_StatusTypeDef LIS2DUX12_Init(void) {
    HAL_StatusTypeDef status;

    // Try power-up with current address (0x19)
    status = LIS2DUX12_PowerUp();
    if (status != HAL_OK) {
        // Try the other address (0x18)
        lis2dux12_address = LIS2DUX12_I2C_ADDRESS_LOW;
        status = LIS2DUX12_PowerUp();
        if (status != HAL_OK) {
            return status;  // Both addresses failed
        }
    }

    // Initialize the device context
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &hi2c1;

    // Configure the sensor using direct register writes (simpler approach)
    uint8_t reg_value;

    // Set CTRL5 register: 100Hz ODR, ±2g full scale
    // ODR[3:0] = 1000 (100Hz), BW[1:0] = 00, FS[1:0] = 00 (±2g)
    reg_value = 0x80;
    if (platform_write(&hi2c1, LIS2DUX12_CTRL5, &reg_value, 1) != 0) {
        return HAL_ERROR;
    }

    HAL_Delay(10);  // Small delay after configuration

    return HAL_OK;
}

// Read acceleration data using basic register reads
HAL_StatusTypeDef LIS2DUX12_ReadAccel(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg) {
    uint8_t raw_data[6];
    int16_t raw_x, raw_y, raw_z;

    // Read 6 bytes starting from OUT_X_L
    if (platform_read(&hi2c1, LIS2DUX12_OUT_X_L, raw_data, 6) != 0) {
        return HAL_ERROR;
    }

    // Combine LSB and MSB for each axis
    raw_x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    raw_y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    raw_z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    // Convert to mg (for ±2g range, sensitivity is 0.061 mg/LSB)
    *x_mg = (int16_t)((float)raw_x * 0.061f);
    *y_mg = (int16_t)((float)raw_y * 0.061f);
    *z_mg = (int16_t)((float)raw_z * 0.061f);

    return HAL_OK;
}

// Test function to add to your main loop
void LIS2DUX12_Test(void) {
    static uint32_t last_read = 0;
    int16_t x_mg, y_mg, z_mg;

    // Read every 100ms
    if (HAL_GetTick() - last_read > 100) {
        last_read = HAL_GetTick();

        HAL_StatusTypeDef status = LIS2DUX12_ReadAccel(&x_mg, &y_mg, &z_mg);

        if (status == HAL_OK) {
            // Data read successfully - set breakpoint here to see values
            // x_mg, y_mg, z_mg contain acceleration in milligrams

            // Optional: Toggle LED to indicate successful reading
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        } else {
            // Error reading data - set breakpoint here to debug
        }
    }
}

void LIS2DUX12_Debug(void) {
    uint8_t who_am_i = 0;
    HAL_StatusTypeDef status;

    // Test both I2C addresses
    uint8_t addresses[2] = {0x18, 0x19};  // Both possible addresses

    for (int addr_idx = 0; addr_idx < 2; addr_idx++) {
        uint8_t test_addr = addresses[addr_idx];

        // First try direct communication (no power-up sequence)
        status = HAL_I2C_Mem_Read(&hi2c1, (test_addr << 1), LIS2DUX12_WHO_AM_I,
                                  I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 1000);

        if (status == HAL_OK) {
            // Direct read worked - device might already be powered up
            if (who_am_i == LIS2DUX12_ID) {
                // Success with address test_addr
                for (int i = 0; i < (addr_idx + 1); i++) {
                    playTone(1000, 50);
                    HAL_Delay(100);
                }
                return;
            } else {
                // Wrong WHO_AM_I value
                playTone(500, 50); // Mid tone = wrong ID
                HAL_Delay(100);
            }
        } else {
            // Direct read failed - try power-up sequence
            uint8_t dummy = 0;
            HAL_I2C_Master_Transmit(&hi2c1, (test_addr << 1), &dummy, 0, 100);
            HAL_Delay(25);

            status = HAL_I2C_Mem_Read(&hi2c1, (test_addr << 1), LIS2DUX12_WHO_AM_I,
                                      I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 1000);

            if (status == HAL_OK && who_am_i == LIS2DUX12_ID) {
                // Power-up sequence worked
                for (int i = 0; i < (addr_idx + 3); i++) {  // 3+ beeps for power-up success
                    playTone(1000, 50);
                    HAL_Delay(100);
                }
                return;
            }
        }
    }

    // All tests failed
    playTone(200, 500);  // Long low tone = total failure
}

/* Simple I2C scanner - Add to USER CODE BEGIN 4 */
void I2C_Scanner(void) {
    HAL_StatusTypeDef status;
    uint8_t found_devices = 0;

    for (uint8_t addr = 1; addr < 128; addr++) {
        status = HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 3, 5);
        if (status == HAL_OK) {
            found_devices++;
            // Beep for each device found, with frequency based on address
            playTone(400 + (addr * 5), 20);
            HAL_Delay(200);
        }
    }

    if (found_devices == 0) {
        // No devices found - 3 low beeps
        for (int i = 0; i < 3; i++) {
            playTone(200, 200);
            HAL_Delay(300);
        }
    }
}
///////////////////Claude slop////////////////////////////

void Handle_Pulsing_LED(int ms_delay)
{
    // Simple breathing effect - goes from 0 to max brightness and back
    if (pulse_direction) {
        pulse_value += pulse_step;
        if (pulse_value >= htim2.Init.Period) {
            pulse_direction = 0;  // Start decreasing
        }
    } else {
        pulse_value -= pulse_step;
        if (pulse_value <= 0) {
            pulse_direction = 1;  // Start increasing
        }
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_value);

    HAL_Delay(ms_delay);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void twiScan(void) {
    for (uint8_t i = 0; i < 128; i++) {
        uint16_t address = (uint16_t)(i << 1);
        if (HAL_I2C_IsDeviceReady(&hi2c1, address, 3, 5) == HAL_OK) {
        } else {

        }
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
  I2C_Scanner();        // First scan for any I2C devices
  HAL_Delay(100);
  //LIS2DUX12_Debug();    // Then specifically test LIS2DUX12

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  Handle_Pulsing_LED(10);
    /* USER CODE BEGIN 3 */


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
