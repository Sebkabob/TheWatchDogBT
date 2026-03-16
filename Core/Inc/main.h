/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wb0x_hal.h"
#include "app_entry.h"
#include "app_common.h"
#include "app_debug.h"
#include "compiler.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_1
#define LED3_GPIO_Port GPIOB
#define EEPROM_POWER_Pin GPIO_PIN_0
#define EEPROM_POWER_GPIO_Port GPIOB
#define GPOUT_Pin GPIO_PIN_8
#define GPOUT_GPIO_Port GPIOA
#define I2C_POWER_Pin GPIO_PIN_10
#define I2C_POWER_GPIO_Port GPIOA
#define ACCEL_INT_Pin GPIO_PIN_15
#define ACCEL_INT_GPIO_Port GPIOB
#define BUZZ_1_Pin GPIO_PIN_6
#define BUZZ_1_GPIO_Port GPIOB
#define BQ251_PG_Pin GPIO_PIN_4
#define BQ251_PG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define BQ251_STAT_Pin GPIO_PIN_9
#define BQ251_STAT_GPIO_Port GPIOA

#define IS_CABLE_PLUGGED()  (HAL_GPIO_ReadPin(BQ251_PG_GPIO_Port, BQ251_PG_Pin) == GPIO_PIN_RESET)
#define IS_CHARGING_NOW()   (HAL_GPIO_ReadPin(BQ251_STAT_GPIO_Port, BQ251_STAT_Pin) == GPIO_PIN_RESET)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
