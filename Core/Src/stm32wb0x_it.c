/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wb0x_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32wb0x_it.h"
#include "hw_pka.h"
#include "ble_stack.h"
#include "miscutil.h"
#include "stm32wb0x_ll_usart.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "state_machine.h"
#include "crash_forensics.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim16;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  *
  * On entry to a Cortex-M exception, the hardware has already pushed
  *   R0, R1, R2, R3, R12, LR, return-PC, xPSR
  * onto whichever stack was active (MSP or PSP — selected via EXC_RETURN
  * bit 2). We peek at that frame, hand the PC/LR/xPSR to the forensics
  * module, then NVIC_SystemReset from inside CrashForensics_RecordFault.
  *
  * Replacing the CubeMX default while(1) here is the difference between
  * "device froze silently" (previous behaviour) and "device reset and the
  * diagnostic dump shows where it died." See crash_forensics.h.
  */
__attribute__((naked))
void HardFault_Handler(void)
{
    __asm volatile (
        "movs r0, #4              \n"   /* test EXC_RETURN bit 2 */
        "mov  r1, lr              \n"
        "tst  r0, r1              \n"
        "bne  use_psp             \n"
        "mrs  r0, msp             \n"
        "b    have_sp             \n"
        "use_psp:                 \n"
        "mrs  r0, psp             \n"
        "have_sp:                 \n"
        "ldr  r1, [r0, #24]       \n"   /* stacked PC   = SP[6] */
        "ldr  r2, [r0, #20]       \n"   /* stacked LR   = SP[5] */
        "ldr  r3, [r0, #28]       \n"   /* stacked xPSR = SP[7] */
        "mov  r0, r1              \n"   /* arg0 = pc   */
        "mov  r1, r2              \n"   /* arg1 = lr   */
        "mov  r2, r3              \n"   /* arg2 = xpsr */
        "bl   CrashForensics_RecordFault\n"
        /* CrashForensics_RecordFault calls NVIC_SystemReset — never returns. */
        "b    .                   \n"
    );
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32WB0x Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wb0x.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RCC interrupt.
  */
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

/**
  * @brief This function handles GPIOB interrupt.
  */
void GPIOB_IRQHandler(void)
{
  /* USER CODE BEGIN GPIOB_IRQn 0 */

	  /*
	   * Check PB4 (BQ251_PG) FIRST — cable plug/unplug event.
	   * PG is active-low: falling edge = cable plugged in.
	   */
	  if (__HAL_GPIO_EXTI_GET_IT(GPIOB, BQ251_PG_Pin)) {
	      __HAL_GPIO_EXTI_CLEAR_IT(GPIOB, BQ251_PG_Pin);

	      /* Wake MCU and keep it awake — the state machine will
	       * handle restoring peripherals and showing charge status. */
	      CablePlug_IRQCallback();
	  }

  /* USER CODE END GPIOB_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIOB,GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIOB,GPIO_PIN_15);
  HAL_GPIO_EXTI_IRQHandler(GPIOB,GPIO_PIN_4);
  /* USER CODE BEGIN GPIOB_IRQn 1 */

  /* USER CODE END GPIOB_IRQn 1 */
}

/**
  * @brief This function handles RADIO_TIMER_CPU_WKUP global interrupt.
  */
void RADIO_TIMER_CPU_WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN RADIO_TIMER_CPU_WKUP_IRQn 0 */

  /* USER CODE END RADIO_TIMER_CPU_WKUP_IRQn 0 */
  HAL_RADIO_TIMER_CPU_WKUP_IRQHandler();
  /* USER CODE BEGIN RADIO_TIMER_CPU_WKUP_IRQn 1 */

  /* USER CODE END RADIO_TIMER_CPU_WKUP_IRQn 1 */
}

/**
  * @brief This function handles RADIO_TIMER_ERROR global interrupt.
  */
void RADIO_TIMER_ERROR_IRQHandler(void)
{
  /* USER CODE BEGIN RADIO_TIMER_ERROR_IRQn 0 */

  /* USER CODE END RADIO_TIMER_ERROR_IRQn 0 */
  HAL_RADIO_TIMER_ERROR_IRQHandler();
  /* USER CODE BEGIN RADIO_TIMER_ERROR_IRQn 1 */

  /* USER CODE END RADIO_TIMER_ERROR_IRQn 1 */
}

/**
  * @brief This function handles RADIO_TXRX global interrupt.
  */
void RADIO_TXRX_IRQHandler(void)
{
  /* USER CODE BEGIN RADIO_TXRX_IRQn 0 */

  /* USER CODE END RADIO_TXRX_IRQn 0 */
  HAL_RADIO_TXRX_IRQHandler();
  /* USER CODE BEGIN RADIO_TXRX_IRQn 1 */

  /* USER CODE END RADIO_TXRX_IRQn 1 */
}

/**
  * @brief This function handles RADIO_TXRX_SEQ global interrupt.
  */
void RADIO_TXRX_SEQ_IRQHandler(void)
{
  /* USER CODE BEGIN RADIO_TXRX_SEQ_IRQn 0 */

  /* USER CODE END RADIO_TXRX_SEQ_IRQn 0 */
  HAL_RADIO_TXRX_SEQ_IRQHandler();
  /* USER CODE BEGIN RADIO_TXRX_SEQ_IRQn 1 */

  /* USER CODE END RADIO_TXRX_SEQ_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
