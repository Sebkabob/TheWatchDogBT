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
// iOS → device opcodes (first byte of an APPTOWD write, after the token).
#define CMD_REQUEST_LOG_COUNT    0xF0
#define CMD_REQUEST_EVENT        0xF1   // payload: uint16 BE event index
#define CMD_CLEAR_LOG            0xF2
#define CMD_ACK_EVENT            0xF3
#define CMD_FIND_MY_DEVICE       0xFA   // byte[1] bit 0 = start
#define CMD_RESET_DEVICE         0xFB
#define CMD_DRAIN_MODE           0xFC   // byte[1] bit 0: 1=start, 0=stop

// Device → iOS response markers (DEVICESTATUS notify).
#define RESP_LOG_COUNT           0xE0
#define RESP_EVENT_DATA          0xE1
#define RESP_NO_MORE_EVENTS      0xE2
#define RESP_LOG_CLEARED         0xE3

// Loyalty (application-layer ownership) opcodes — self-contained writes.
#define CMD_CLAIM_DEVICE         0xC1   // first claim
#define CMD_VERIFY_OWNER         0xC2   // reconnect
#define CMD_UNBOND_DEVICE        0xC0   // user unpair

// Loyalty response markers (firmware → iOS via DEVICESTATUS notify).
#define RESP_CLAIM_OK            0xE7   // [0xE7, 0x01]
#define RESP_REJECT              0xE8   // [0xE8, 0x01], followed by disconnect
#define RESP_VERIFY_OK           0xE9   // [0xE9, 0x01]
#define RESP_UNPAIR_ACK          0xE4   // [0xE4, 0x01], followed by disconnect

#define LOYALTY_TOKEN_LEN        4
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
void LOCKSERVICE_SendMotionAlert(uint8_t motionType);
void LOCKSERVICE_SendBatteryDiagnostic(void);

// Drain-mode test: white LED at max + continuous tone, auto-stops at SOC=5%.
// Drain_Tick() must run every main-loop iteration to re-assert outputs.
#define DRAIN_AUTO_STOP_SOC      5
#define DRAIN_TONE_FREQUENCY_HZ  60

void Drain_Start(void);
void Drain_Stop(void);
uint8_t Drain_IsActive(void);
void Drain_Tick(void);
/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*LOCKSERVICE_APP_H */
