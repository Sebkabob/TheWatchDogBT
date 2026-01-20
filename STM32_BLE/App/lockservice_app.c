/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    LockService_app.c
  * @author  MCD Application Team
  * @brief   LockService_app application definition.
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
#include "app_common.h"
#include "app_ble.h"
#include "ble.h"
#include "lockservice_app.h"
#include "lockservice.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sound.h"
#include "motion_logger.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

typedef enum
{
  Devicestatus_NOTIFICATION_OFF,
  Devicestatus_NOTIFICATION_ON,
  /* USER CODE BEGIN Service1_APP_SendInformation_t */

  /* USER CODE END Service1_APP_SendInformation_t */
  LOCKSERVICE_APP_SENDINFORMATION_LAST
} LOCKSERVICE_APP_SendInformation_t;

typedef struct
{
  LOCKSERVICE_APP_SendInformation_t     Devicestatus_Notification_Status;
  /* USER CODE BEGIN Service1_APP_Context_t */

  /* USER CODE END Service1_APP_Context_t */
  uint16_t              ConnectionHandle;
} LOCKSERVICE_APP_Context_t;

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static LOCKSERVICE_APP_Context_t LOCKSERVICE_APP_Context;

uint8_t a_LOCKSERVICE_UpdateCharData[247];

/* USER CODE BEGIN PV */
extern volatile uint8_t deviceState;
extern volatile uint8_t deviceInfo;
extern volatile uint8_t deviceBattery;

// Track current transfer state
static uint16_t currentEventIndex = 0;
static uint8_t transferInProgress = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void LOCKSERVICE_Devicestatus_SendNotification(void);

/* USER CODE BEGIN PFP */
/**
 * @brief Send motion alert notification to iOS
 * This triggers iOS to auto-sync
 */
void LOCKSERVICE_SendMotionAlert(void)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) {
        return;
    }

    // Send special alert byte: 0xFF
    a_LOCKSERVICE_UpdateCharData[0] = 0xFF;  // Motion alert marker
    a_LOCKSERVICE_UpdateCharData[1] = deviceBattery;

    LOCKSERVICE_Data_t lockservice_notification_data;
    lockservice_notification_data.p_Payload = (uint8_t*)a_LOCKSERVICE_UpdateCharData;
    lockservice_notification_data.Length = 2;

    LOCKSERVICE_NotifyValue(LOCKSERVICE_DEVICESTATUS, &lockservice_notification_data,
                           LOCKSERVICE_APP_Context.ConnectionHandle);

    printf("🚨 Motion alert sent to iOS\n");
}

/**
 * @brief Update RTC from iOS timestamp
 * @param data Pointer to timestamp data (6 bytes: year, month, day, hour, minute, second)
 */
static void UpdateBootTimeFromiOS(uint8_t *timestamp_data)
{
    // Set boot time in motion logger
    MotionLogger_SetBootTime(
        timestamp_data[0],  // Year offset from 2000
        timestamp_data[1],  // Month
        timestamp_data[2],  // Day
        timestamp_data[3],  // Hour
        timestamp_data[4],  // Minute
        timestamp_data[5]   // Second
    );
}

/**
 * @brief Send event count to iOS
 */
static void LOCKSERVICE_SendEventCount(void)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) {
        return;
    }

    uint16_t eventCount = MotionLogger_GetEventCount();

    a_LOCKSERVICE_UpdateCharData[0] = RESP_LOG_COUNT;
    a_LOCKSERVICE_UpdateCharData[1] = (eventCount >> 8) & 0xFF;  // High byte
    a_LOCKSERVICE_UpdateCharData[2] = eventCount & 0xFF;         // Low byte

    LOCKSERVICE_Data_t lockservice_notification_data;
    lockservice_notification_data.p_Payload = (uint8_t*)a_LOCKSERVICE_UpdateCharData;
    lockservice_notification_data.Length = 3;

    LOCKSERVICE_NotifyValue(LOCKSERVICE_DEVICESTATUS, &lockservice_notification_data,
                           LOCKSERVICE_APP_Context.ConnectionHandle);
}

/**
 * @brief Send specific event to iOS
 * @param index Event index to send
 */

/**
 * @brief Send specific event to iOS
 * @param index Event index to send
 */
static void LOCKSERVICE_SendEvent(uint16_t index)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) {
        return;
    }

    MotionEvent_t *event = MotionLogger_GetEvent(index);

    if (event == NULL) {
        // No event at this index
        a_LOCKSERVICE_UpdateCharData[0] = RESP_NO_MORE_EVENTS;
        a_LOCKSERVICE_UpdateCharData[1] = (index >> 8) & 0xFF;
        a_LOCKSERVICE_UpdateCharData[2] = index & 0xFF;

        LOCKSERVICE_Data_t lockservice_notification_data;
        lockservice_notification_data.p_Payload = (uint8_t*)a_LOCKSERVICE_UpdateCharData;
        lockservice_notification_data.Length = 3;

        LOCKSERVICE_NotifyValue(LOCKSERVICE_DEVICESTATUS, &lockservice_notification_data,
                               LOCKSERVICE_APP_Context.ConnectionHandle);
        return;
    }

    // Convert tick timestamp to real date/time
    uint8_t year, month, day, hour, minute, second;
    MotionLogger_TickToDateTime(event->timestamp_ms, &year, &month, &day, &hour, &minute, &second);

    // Pack event data (same format as before)
    uint8_t dataIdx = 0;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = RESP_EVENT_DATA;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = (index >> 8) & 0xFF;  // Index high byte
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = index & 0xFF;         // Index low byte
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = year;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = month;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = day;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = hour;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = minute;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = second;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = event->motionType;

    LOCKSERVICE_Data_t lockservice_notification_data;
    lockservice_notification_data.p_Payload = (uint8_t*)a_LOCKSERVICE_UpdateCharData;
    lockservice_notification_data.Length = dataIdx;

    LOCKSERVICE_NotifyValue(LOCKSERVICE_DEVICESTATUS, &lockservice_notification_data,
                           LOCKSERVICE_APP_Context.ConnectionHandle);
}

/**
 * @brief Send log cleared confirmation
 */
static void LOCKSERVICE_SendLogCleared(void)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) {
        return;
    }

    a_LOCKSERVICE_UpdateCharData[0] = RESP_LOG_CLEARED;

    LOCKSERVICE_Data_t lockservice_notification_data;
    lockservice_notification_data.p_Payload = (uint8_t*)a_LOCKSERVICE_UpdateCharData;
    lockservice_notification_data.Length = 1;

    LOCKSERVICE_NotifyValue(LOCKSERVICE_DEVICESTATUS, &lockservice_notification_data,
                           LOCKSERVICE_APP_Context.ConnectionHandle);
}

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void LOCKSERVICE_Notification(LOCKSERVICE_NotificationEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service1_Notification_1 */

  /* USER CODE END Service1_Notification_1 */
  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service1_Notification_Service1_EvtOpcode */

    /* USER CODE END Service1_Notification_Service1_EvtOpcode */

    case LOCKSERVICE_APPTOWD_WRITE_EVT:
      /* USER CODE BEGIN Service1Char1_WRITE_EVT */
        uint8_t *received_data = p_Notification->DataTransfered.p_Payload;
        uint8_t data_length = p_Notification->DataTransfered.Length;

        if(data_length > 0)
        {
            // Extract timestamp from last 6 bytes (if present)
        	if (data_length >= 7) {
        	    UpdateBootTimeFromiOS(&received_data[data_length - 6]);
        	}

            // Now process the command (first byte, or first N bytes before timestamp)
            uint8_t command = received_data[0];

            switch(command) {
                case CMD_REQUEST_LOG_COUNT:
                    transferInProgress = 1;
                    currentEventIndex = 0;
                    LOCKSERVICE_SendEventCount();
                    break;

                case CMD_REQUEST_EVENT:
                    if (data_length >= 3) {
                        uint16_t requestedIndex = ((uint16_t)received_data[1] << 8) | received_data[2];
                        LOCKSERVICE_SendEvent(requestedIndex);
                    }
                    break;

                case CMD_ACK_EVENT:
                    // iOS acknowledged receiving event
                    break;

                case CMD_CLEAR_LOG:
                    MotionLogger_Clear();
                    transferInProgress = 0;
                    currentEventIndex = 0;
                    LOCKSERVICE_SendLogCleared();
                    break;

                default:
                    // Regular device state update
                    deviceState = received_data[0];
                    HAL_Delay(5);
                    LOCKSERVICE_ForceStatusUpdate();
                    break;
            }
        }
        break;
      /* USER CODE END Service1Char1_WRITE_EVT */
      break;

    case LOCKSERVICE_DEVICESTATUS_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service1Char2_NOTIFY_ENABLED_EVT */
    	LOCKSERVICE_ForceStatusUpdate();
      /* USER CODE END Service1Char2_NOTIFY_ENABLED_EVT */
      break;

    case LOCKSERVICE_DEVICESTATUS_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service1Char2_NOTIFY_DISABLED_EVT */

      /* USER CODE END Service1Char2_NOTIFY_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN Service1_Notification_default */

      /* USER CODE END Service1_Notification_default */
      break;
  }
  /* USER CODE BEGIN Service1_Notification_2 */

  /* USER CODE END Service1_Notification_2 */
  return;
}

void LOCKSERVICE_APP_EvtRx(LOCKSERVICE_APP_ConnHandleNotEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service1_APP_EvtRx_1 */

  /* USER CODE END Service1_APP_EvtRx_1 */

  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service1_APP_EvtRx_Service1_EvtOpcode */

    /* USER CODE END Service1_APP_EvtRx_Service1_EvtOpcode */
    case LOCKSERVICE_CONN_HANDLE_EVT :
      LOCKSERVICE_APP_Context.ConnectionHandle = p_Notification->ConnectionHandle;
      /* USER CODE BEGIN Service1_APP_CENTR_CONN_HANDLE_EVT */
      LOCKSERVICE_ForceStatusUpdate();  // Force send on connection
      /* USER CODE END Service1_APP_CENTR_CONN_HANDLE_EVT */
      break;
    case LOCKSERVICE_DISCON_HANDLE_EVT :
      LOCKSERVICE_APP_Context.ConnectionHandle = 0xFFFF;
      /* USER CODE BEGIN Service1_APP_DISCON_HANDLE_EVT */
      playTone(380,10);
      HAL_Delay(10);
      playTone(280,12);
      HAL_Delay(10);
      playTone(100,15);
      /* USER CODE END Service1_APP_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN Service1_APP_EvtRx_default */

      /* USER CODE END Service1_APP_EvtRx_default */
      break;
  }

  /* USER CODE BEGIN Service1_APP_EvtRx_2 */

  /* USER CODE END Service1_APP_EvtRx_2 */

  return;
}

void LOCKSERVICE_APP_Init(void)
{
  LOCKSERVICE_APP_Context.ConnectionHandle = 0xFFFF;
  LOCKSERVICE_Init();

  /* USER CODE BEGIN Service1_APP_Init */

  /* USER CODE END Service1_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void LOCKSERVICE_SendStatusUpdate(void)
{
    // Only send if the state has actually changed OR if it's the first time
    //if (deviceInfo != lastSentDeviceInfo)
    //{
        LOCKSERVICE_Devicestatus_SendNotification();
        //lastSentDeviceInfo = deviceInfo;  // Update the last sent value
    //}
}

void LOCKSERVICE_ForceStatusUpdate(void)
{
    // Force send regardless of state change (for initial connection)
    LOCKSERVICE_Devicestatus_SendNotification();
    //lastSentDeviceInfo = deviceInfo;
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
__USED void LOCKSERVICE_Devicestatus_SendNotification(void) /* Property Notification */
{
  LOCKSERVICE_APP_SendInformation_t notification_on_off = Devicestatus_NOTIFICATION_OFF;
  LOCKSERVICE_Data_t lockservice_notification_data;

  lockservice_notification_data.p_Payload = (uint8_t*)a_LOCKSERVICE_UpdateCharData;
  lockservice_notification_data.Length = 0;

  /* USER CODE BEGIN Service1Char2_NS_1*/
  // Set notification to ON so it actually sends
  notification_on_off = Devicestatus_NOTIFICATION_ON;

  // Send deviceState (not deviceInfo)
  a_LOCKSERVICE_UpdateCharData[0] = deviceState;
  a_LOCKSERVICE_UpdateCharData[1] = deviceBattery;

  // Set the actual length of data you're sending
  lockservice_notification_data.Length = 2;
  /* USER CODE END Service1Char2_NS_1*/

  if (notification_on_off != Devicestatus_NOTIFICATION_OFF && LOCKSERVICE_APP_Context.ConnectionHandle != 0xFFFF)
  {
    LOCKSERVICE_NotifyValue(LOCKSERVICE_DEVICESTATUS, &lockservice_notification_data, LOCKSERVICE_APP_Context.ConnectionHandle);
  }

  /* USER CODE BEGIN Service1Char2_NS_Last*/

  /* USER CODE END Service1Char2_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
