/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    LockService_app.h
  * @author  MCD Application Team
  * @brief   Header for LockService_app.c
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
#ifndef LOCKSERVICE_APP_H
#define LOCKSERVICE_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  LOCKSERVICE_CONN_HANDLE_EVT,
  LOCKSERVICE_DISCON_HANDLE_EVT,

  /* USER CODE BEGIN Service1_OpcodeNotificationEvt_t */

  /* USER CODE END Service1_OpcodeNotificationEvt_t */

  LOCKSERVICE_LAST_EVT,
} LOCKSERVICE_APP_OpcodeNotificationEvt_t;

typedef struct
{
  LOCKSERVICE_APP_OpcodeNotificationEvt_t          EvtOpcode;
  uint16_t                                 ConnectionHandle;

  /* USER CODE BEGIN LOCKSERVICE_APP_ConnHandleNotEvt_t */

  /* USER CODE END LOCKSERVICE_APP_ConnHandleNotEvt_t */
} LOCKSERVICE_APP_ConnHandleNotEvt_t;
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* BLE Command Definitions */
#define CMD_REQUEST_LOG_COUNT    0xF0  // iOS: Request total event count
#define CMD_REQUEST_EVENT        0xF1  // iOS: Request specific event by index
#define CMD_CLEAR_LOG            0xF2  // iOS: Clear all events
#define CMD_ACK_EVENT            0xF3  // iOS: Acknowledge received event

/* Response Types */
#define RESP_LOG_COUNT           0xE0  // WD: Sending log count
#define RESP_EVENT_DATA          0xE1  // WD: Sending event data
#define RESP_NO_MORE_EVENTS      0xE2  // WD: No event at requested index
#define RESP_LOG_CLEARED         0xE3  // WD: Log cleared confirmation
/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void LOCKSERVICE_APP_Init(void);
void LOCKSERVICE_APP_EvtRx(LOCKSERVICE_APP_ConnHandleNotEvt_t *p_Notification);
/* USER CODE BEGIN EF */
void LOCKSERVICE_SendStatusUpdate(void);
void LOCKSERVICE_ForceStatusUpdate(void);
/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*LOCKSERVICE_APP_H */
