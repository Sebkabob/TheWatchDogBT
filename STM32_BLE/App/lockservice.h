/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    LockService.h
  * @author  MCD Application Team
  * @brief   Header for LockService.c
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
#ifndef LOCKSERVICE_H
#define LOCKSERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ble_status.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported defines ----------------------------------------------------------*/
/* USER CODE BEGIN ED */

/* USER CODE END ED */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  LOCKSERVICE_APPTOWD,
  LOCKSERVICE_DEVICESTATUS,
  LOCKSERVICE_MOTIONDATA,

  /* USER CODE BEGIN Service1_CharOpcode_t */

  /* USER CODE END Service1_CharOpcode_t */

  LOCKSERVICE_CHAROPCODE_LAST
} LOCKSERVICE_CharOpcode_t;

typedef enum
{
  LOCKSERVICE_APPTOWD_WRITE_EVT,
  LOCKSERVICE_DEVICESTATUS_NOTIFY_ENABLED_EVT,
  LOCKSERVICE_DEVICESTATUS_NOTIFY_DISABLED_EVT,

  /* USER CODE BEGIN Service1_OpcodeEvt_t */

  /* USER CODE END Service1_OpcodeEvt_t */

  LOCKSERVICE_BOOT_REQUEST_EVT
} LOCKSERVICE_OpcodeEvt_t;

typedef struct
{
  uint8_t *p_Payload;
  uint8_t Length;

  /* USER CODE BEGIN Service1_Data_t */

  /* USER CODE END Service1_Data_t */

} LOCKSERVICE_Data_t;

typedef struct
{
  LOCKSERVICE_OpcodeEvt_t       EvtOpcode;
  LOCKSERVICE_Data_t             DataTransfered;
  uint16_t                ConnectionHandle;
  uint16_t                AttributeHandle;
  uint8_t                 ServiceInstance;

  /* USER CODE BEGIN Service1_NotificationEvt_t */

  /* USER CODE END Service1_NotificationEvt_t */

} LOCKSERVICE_NotificationEvt_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void LOCKSERVICE_Init(void);
void LOCKSERVICE_Notification(LOCKSERVICE_NotificationEvt_t *p_Notification);
tBleStatus LOCKSERVICE_UpdateValue(LOCKSERVICE_CharOpcode_t CharOpcode, LOCKSERVICE_Data_t *pData);
tBleStatus LOCKSERVICE_NotifyValue(LOCKSERVICE_CharOpcode_t CharOpcode, LOCKSERVICE_Data_t *pData, uint16_t ConnectionHandle);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*LOCKSERVICE_H */
