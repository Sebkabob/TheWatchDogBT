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
#define LIS2DUX12_I2C_ADDRESS_HIGH  0x19   // When SA0/SDO = VDD

#define BQ25_ADDRESS 0x6A
#define BQ25186_ADDR (0x6A << 1)


static uint32_t pulse_value = 0;
static uint8_t pulse_direction = 1; // 1 = increasing, 0 = decreasing
static uint32_t pulse_step = 5; // Adjust for pulse speed (smaller = slower)

stmdev_ctx_t dev_ctx;

/** Please note that is MANDATORY: return 0 -> no Error.**/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
int32_t LIS2DUX12_ProperInit(void);
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
static int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write(handle, LIS2DUX12_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

  HAL_I2C_Mem_Read(handle, LIS2DUX12_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  return 0;
}

void BQ25186_SetChargeCurrent(uint16_t current_mA) {
    uint8_t ichg_code;

    // Calculate ICHG code based on desired current
    if (current_mA <= 35) {
        ichg_code = current_mA - 5;  // For currents 5-35mA
    } else if (current_mA >= 40) {
        ichg_code = 31 + ((current_mA - 40) / 10);  // For currents 40mA+
    } else {
        ichg_code = 30;  // Default to 35mA for invalid range
    }

    // Ensure we don't exceed maximum
    if (ichg_code > 127) ichg_code = 127;  // Max code for 1000mA

    // Write to ICHG_CTRL register (0x4)
    // Bit 7 = 0 (charging enabled), Bits 6-0 = ichg_code
    uint8_t reg_value = ichg_code & 0x7F;  // Ensure bit 7 is 0

    // I2C write to register 0x4
    HAL_I2C_Mem_Write(&hi2c1, BQ25186_ADDR, 0x04,
                      I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
}

void BQ25186_SetBatteryVoltage(float voltage_V) {
    uint8_t vbat_code;

    // Clamp voltage to valid range
    if (voltage_V < 3.5f) voltage_V = 3.5f;
    if (voltage_V > 4.65f) voltage_V = 4.65f;

    // Calculate VBATREG code
    vbat_code = (uint8_t)((voltage_V - 3.5f) / 0.01f);  // 0.01V = 10mV steps

    // Ensure we don't exceed 7-bit range
    if (vbat_code > 127) vbat_code = 127;

    // Read current register to preserve PG_MODE bit
    uint8_t current_reg;
    HAL_I2C_Mem_Read(&hi2c1, BQ25186_ADDR, 0x03,
                     I2C_MEMADD_SIZE_8BIT, &current_reg, 1, HAL_MAX_DELAY);

    // Preserve bit 7 (PG_MODE), update bits 6-0
    uint8_t new_reg = (current_reg & 0x80) | (vbat_code & 0x7F);

    // Write to VBAT_CTRL register
    HAL_I2C_Mem_Write(&hi2c1, BQ25186_ADDR, 0x03,
                      I2C_MEMADD_SIZE_8BIT, &new_reg, 1, HAL_MAX_DELAY);
}

int32_t LIS2DUX12_ProperInit(void) {
    // Initialize device context
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &hi2c1;

    // Add actual sensor configuration here
    // Check WHO_AM_I register, configure ODR, etc.

    return 0; // Return 0 for success
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


// Dynamic gradient impact detection with smooth buzzer response
void LIS2DUX12_ImpactDetection_Dynamic(void) {
    static lis2dux12_xl_data_t xl_data;
    static float prev_magnitude = 1000.0f;
    static uint32_t last_beep_time = 0;
    static uint32_t last_read_time = 0;
    lis2dux12_status_t status;
    int32_t ret;

    uint32_t current_time = HAL_GetTick();

    // Read data every 20ms for better responsiveness
    if (current_time - last_read_time < 20) {
        return;
    }
    last_read_time = current_time;

    // Prevent beep spam - minimum 150ms between beeps
    if (current_time - last_beep_time < 150) {
        return;
    }

    ret = lis2dux12_status_get(&dev_ctx, &status);
    if (ret != 0 || !status.drdy) {
        return;
    }

    lis2dux12_md_t md = {
        .fs = LIS2DUX12_2g,
        .odr = LIS2DUX12_100Hz_LP,
        .bw = LIS2DUX12_ODR_div_2
    };

    ret = lis2dux12_xl_data_get(&dev_ctx, &md, &xl_data);
    if (ret != 0) {
        return;
    }

    // Calculate current magnitude
    float x_g = xl_data.mg[0];
    float y_g = xl_data.mg[1];
    float z_g = xl_data.mg[2];
    float current_magnitude = sqrtf(x_g*x_g + y_g*y_g + z_g*z_g);

    // Look for sudden changes in acceleration (impact detection)
    float magnitude_change = fabsf(current_magnitude - prev_magnitude);

    // Dynamic threshold - minimum impact to trigger
    float min_threshold = 100.0f;
    float max_threshold = 2000.0f; // Maximum expected impact

    if (magnitude_change > min_threshold) {
        // Normalize impact to 0-1 range
        float normalized_impact = (magnitude_change - min_threshold) / (max_threshold - min_threshold);

        // Clamp to 0-1 range
        if (normalized_impact > 1.0f) normalized_impact = 1.0f;
        if (normalized_impact < 0.0f) normalized_impact = 0.0f;

        // Dynamic frequency calculation (200Hz to 2000Hz)
        uint16_t min_frequency = 200;
        uint16_t max_frequency = 2000;
        uint16_t frequency = min_frequency + (uint16_t)(normalized_impact * (max_frequency - min_frequency));

        // Dynamic duration calculation (30ms to 250ms)
        uint16_t min_duration = 30;
        uint16_t max_duration = 250;
        uint16_t duration = min_duration + (uint16_t)(normalized_impact * (max_duration - min_duration));

        // Optional: Add exponential scaling for more dramatic effect
        // float exponential_impact = normalized_impact * normalized_impact; // Square for exponential curve
        // frequency = min_frequency + (uint16_t)(exponential_impact * (max_frequency - min_frequency));
        // duration = min_duration + (uint16_t)(exponential_impact * (max_duration - min_duration));

        playTone(frequency, duration);
        last_beep_time = current_time;
    }

    // Update previous magnitude for next comparison
    prev_magnitude = current_magnitude;
}


void ChargeLED(int ms_delay)
{
    // Read Power Good status (LOW = good power present)
    int power_good = (HAL_GPIO_ReadPin(GPIOB, CHARGE_Pin) == GPIO_PIN_RESET);

    // Read charging status from BQ25186
    uint8_t stat0 = readBQ25186Register(0x00);
    uint8_t charge_status = (stat0 >> 5) & 0x03;
    int is_charging = (charge_status == 0x01 || charge_status == 0x02); // CC or CV

    // Static variable to track previous charging state
    static int was_charging = 0;

    // Check if we just started charging (transition from not charging to charging)
    if (power_good && is_charging && !was_charging) {
        // Reset pulse variables for smooth start
        pulse_value = 10;  // Start at minimum (LED off for active low)
        pulse_direction = 1;  // Start increasing (getting brighter)
    }

    // Update the previous state
    was_charging = (power_good && is_charging);

    // Only pulse LED when power is good AND actively charging
    if (power_good && is_charging) {
        // Start PWM if not already running
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

        // Keep pulse range away from extremes to avoid hardware timing issues
        uint32_t min_pulse = 0;   // Don't go below 10
        uint32_t max_pulse = htim2.Init.Period - 200;  // Don't go above 989

        if (pulse_direction) {
            pulse_value += pulse_step;
            if (pulse_value >= max_pulse) {
                pulse_direction = 0;  // Start decreasing
            }
        } else {
            pulse_value -= pulse_step;
            if (pulse_value <= min_pulse) {
                pulse_direction = 1;  // Start increasing
            }
        }

        // For active low LED: invert the PWM value
        uint32_t inverted_pulse = htim2.Init.Period - pulse_value;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, inverted_pulse);
        HAL_Delay(ms_delay);
    } else {
        // Stop PWM completely to turn off LED and save power
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    }
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
  twiScan();
  LIS2DUX12_ProperInit();
  BQ25186_SetChargeCurrent(150);   // Set to 150mA for liPo, 0.5C
  BQ25186_SetBatteryVoltage(4.2f);   // charge to 90% for safety
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  ChargeLED(5);
	  //LIS2DUX12_ImpactDetection_Dynamic();

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

  /**/
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_B, PWR_GPIO_BIT_3);

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
