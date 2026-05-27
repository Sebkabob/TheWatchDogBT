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
#include "battery.h"
#include "state_machine.h"
#include "lights.h"
#include "accelerometer.h"
#include "lis2dux12_app.h"
#include "power_management.h"
#include "loyalty.h"
#include "firmware_version.h"
#include "ee_scratch.h"
#include "crash_forensics.h"
#include <string.h>
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
extern uint8_t g_bd_address[6];   // populated in BLE_Init() (app_ble.c)
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

extern volatile uint8_t connectionStatus;

static uint16_t currentEventIndex = 0;
static uint8_t transferInProgress = 0;

static uint8_t drain_mode_active = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void LOCKSERVICE_Devicestatus_SendNotification(void);

/* USER CODE BEGIN PFP */

/***************************************************************************
 * LOCKSERVICE_SendMotionAlert — push a 4-byte motion alert over DEVICESTATUS
 *   Payload: [0xFF, motionType, duration_ticks_250ms, deviceBattery].
 *   iOS uses the 0xFF marker to drive auto-sync on receipt, and the
 *   duration field to populate MotionEvent.durationTicks250ms (rendered as
 *   seconds in the log UI). Grew from 3 → 4 bytes when bout tracking was
 *   added; iOS already had length-branching to tolerate either format so
 *   no app-side change is needed for this byte addition.
 ***************************************************************************/
void LOCKSERVICE_SendMotionAlert(uint8_t motionType, uint8_t duration_ticks_250ms)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) {
        return;
    }

    a_LOCKSERVICE_UpdateCharData[0] = 0xFF;
    a_LOCKSERVICE_UpdateCharData[1] = motionType;
    a_LOCKSERVICE_UpdateCharData[2] = duration_ticks_250ms;
    a_LOCKSERVICE_UpdateCharData[3] = deviceBattery;

    LOCKSERVICE_Data_t lockservice_notification_data;
    lockservice_notification_data.p_Payload = (uint8_t*)a_LOCKSERVICE_UpdateCharData;
    lockservice_notification_data.Length = 4;

    LOCKSERVICE_NotifyValue(LOCKSERVICE_DEVICESTATUS, &lockservice_notification_data,
                           LOCKSERVICE_APP_Context.ConnectionHandle);
}

// timestamp_data: [year-2000, month, day, hour, minute, second].
static void UpdateBootTimeFromiOS(uint8_t *timestamp_data)
{
    MotionLogger_SetBootTime(
        timestamp_data[0],
        timestamp_data[1],
        timestamp_data[2],
        timestamp_data[3],
        timestamp_data[4],
        timestamp_data[5]
    );
}

/***************************************************************************
 * LOCKSERVICE_SendEventCount — RESP_LOG_COUNT + 16-bit BE event count
 ***************************************************************************/
static void LOCKSERVICE_SendEventCount(void)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) {
        return;
    }

    uint16_t eventCount = MotionLogger_GetEventCount();

    a_LOCKSERVICE_UpdateCharData[0] = RESP_LOG_COUNT;
    a_LOCKSERVICE_UpdateCharData[1] = (eventCount >> 8) & 0xFF;
    a_LOCKSERVICE_UpdateCharData[2] = eventCount & 0xFF;

    LOCKSERVICE_Data_t lockservice_notification_data;
    lockservice_notification_data.p_Payload = (uint8_t*)a_LOCKSERVICE_UpdateCharData;
    lockservice_notification_data.Length = 3;

    LOCKSERVICE_NotifyValue(LOCKSERVICE_DEVICESTATUS, &lockservice_notification_data,
                           LOCKSERVICE_APP_Context.ConnectionHandle);
}

/***************************************************************************
 * LOCKSERVICE_SendEvent — pack one event for iOS (or RESP_NO_MORE_EVENTS)
 *   Wire format on hit (12 bytes — grew from 11 when bout duration was added):
 *     [RESP_EVENT_DATA, idx_hi, idx_lo, YY, MM, DD, hh, mm, ss, type,
 *      duration_ticks_250ms, batt]
 *
 *   Old 11-byte format had no duration. iOS branches on payload length so a
 *   stale-firmware-against-new-app pairing still works (duration defaults to
 *   nil and is rendered as "—").
 ***************************************************************************/
static void LOCKSERVICE_SendEvent(uint16_t index)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) {
        return;
    }

    MotionEvent_t *event = MotionLogger_GetEvent(index);

    if (event == NULL) {
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

    uint8_t year, month, day, hour, minute, second;
    MotionLogger_EpochSecondsToDateTime(event->epoch_seconds_2000,
                                        &year, &month, &day, &hour, &minute, &second);

    uint8_t dataIdx = 0;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = RESP_EVENT_DATA;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = (index >> 8) & 0xFF;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = index & 0xFF;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = year;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = month;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = day;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = hour;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = minute;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = second;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = event->motionType;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = event->duration_ticks_250ms;
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = deviceBattery;

    LOCKSERVICE_Data_t lockservice_notification_data;
    lockservice_notification_data.p_Payload = (uint8_t*)a_LOCKSERVICE_UpdateCharData;
    lockservice_notification_data.Length = dataIdx;

    LOCKSERVICE_NotifyValue(LOCKSERVICE_DEVICESTATUS, &lockservice_notification_data,
                           LOCKSERVICE_APP_Context.ConnectionHandle);
}

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

/***************************************************************************
 * Loyalty_SendResponse — 2-byte CLAIM/VERIFY/UNBOND/REJECT response
 *   disconnect_after = 1 leaves a 50 ms gap so the radio can TX the notify
 *   before aci_gap_terminate() tears down the link.
 ***************************************************************************/
static void Loyalty_SendResponse(uint8_t marker, uint8_t value, uint8_t disconnect_after)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) {
        return;
    }

    a_LOCKSERVICE_UpdateCharData[0] = marker;
    a_LOCKSERVICE_UpdateCharData[1] = value;

    LOCKSERVICE_Data_t resp;
    resp.p_Payload = (uint8_t *)a_LOCKSERVICE_UpdateCharData;
    resp.Length    = 2;
    LOCKSERVICE_NotifyValue(LOCKSERVICE_DEVICESTATUS, &resp,
                            LOCKSERVICE_APP_Context.ConnectionHandle);

    if (disconnect_after) {
        HAL_Delay(50);
        (void)aci_gap_terminate(LOCKSERVICE_APP_Context.ConnectionHandle,
                                0x13 /* REMOTE_USER_TERMINATED */);
    }
}

/***************************************************************************
 * Unlock_OnOwnerAuthenticated — intentional no-op
 *   Kept as the named hook into the loyalty success path so iOS-side
 *   behaviour can be added back here without changing the call sites.
 *   Disarm is driven by the explicit settings-write ARMED bit instead.
 ***************************************************************************/
static void Unlock_OnOwnerAuthenticated(void)
{
    (void)deviceState;
}

/***************************************************************************
 * Send_EeReadResponse — push a scratch-region read result on DEVICESTATUS
 *   Frame: [RESP_EE_READ, offset, length, ...data]. Length is the actual
 *   bytes returned, clamped to EE_SCRATCH_MAX_CHUNK on the caller side.
 ***************************************************************************/
static void Send_EeReadResponse(uint8_t offset, uint8_t length, const uint8_t *data)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) return;
    if (length > EE_SCRATCH_MAX_CHUNK) length = EE_SCRATCH_MAX_CHUNK;

    a_LOCKSERVICE_UpdateCharData[0] = RESP_EE_READ;
    a_LOCKSERVICE_UpdateCharData[1] = offset;
    a_LOCKSERVICE_UpdateCharData[2] = length;
    if (length > 0 && data != NULL) {
        memcpy(&a_LOCKSERVICE_UpdateCharData[3], data, length);
    }

    LOCKSERVICE_Data_t resp;
    resp.p_Payload = (uint8_t *)a_LOCKSERVICE_UpdateCharData;
    resp.Length    = (uint16_t)(3 + length);
    LOCKSERVICE_NotifyValue(LOCKSERVICE_DEVICESTATUS, &resp,
                            LOCKSERVICE_APP_Context.ConnectionHandle);
}

/***************************************************************************
 * Send_EeAck — 2-byte ACK/NACK on DEVICESTATUS for scratch-region writes
 *   marker: RESP_EE_WRITE_ACK (success) or RESP_EE_REJECT (failure).
 *   value : 1=ok on ACK path; 1=bounds / 2=I2C on REJECT path.
 ***************************************************************************/
static void Send_EeAck(uint8_t marker, uint8_t value)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) return;

    a_LOCKSERVICE_UpdateCharData[0] = marker;
    a_LOCKSERVICE_UpdateCharData[1] = value;

    LOCKSERVICE_Data_t resp;
    resp.p_Payload = (uint8_t *)a_LOCKSERVICE_UpdateCharData;
    resp.Length    = 2;
    LOCKSERVICE_NotifyValue(LOCKSERVICE_DEVICESTATUS, &resp,
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
      {
        uint8_t *received_data = p_Notification->DataTransfered.p_Payload;
        uint8_t  data_length   = p_Notification->DataTransfered.Length;

        if (data_length == 0) {
            break;
        }

        // If the EEPROM read at boot failed, refuse everything. A transient
        // I2C glitch must not let any phone CLAIM (and therefore hijack) a
        // device that is actually owned.
        if (Loyalty_StoreUnhealthy()) {
            Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
            break;
        }

        // Loyalty layer.
        //   CLAIM  : [0xC1, t0, t1, t2, t3]   (5 bytes, self-contained)
        //   VERIFY : [0xC2, t0, t1, t2, t3]   (5 bytes, self-contained)
        //   UNBOND : [0xC0, t0, t1, t2, t3]   (5 bytes, self-contained)
        // Everything else is token-prefixed: [t0..t3, opcode, ...payload].
        uint8_t first_byte = received_data[0];

        if (first_byte == CMD_CLAIM_DEVICE) {
            if (data_length < 1 + LOYALTY_TOKEN_LEN) {
                Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
                break;
            }

            // USB-C reset window: gated overwrite of an existing claim. This is
            // how a different phone (or the same phone after a Keychain wipe)
            // takes ownership of an already-claimed device — the user proves
            // physical possession by plugging in USB-C within the window.
            if (Loyalty_IsResetWindowOpen()) {
                if (Loyalty_Claim(&received_data[1])) {
                    Loyalty_SendResponse(RESP_CLAIM_OK, 0x01, 0);
                    Unlock_OnOwnerAuthenticated();
                } else {
                    Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
                }
                break;
            }

            // Unowned device (factory-fresh or post-UNBOND): anyone may claim.
            // No security gate — there's no existing claim to protect, and
            // UNBOND deliberately put the device in this state.
            if (!Loyalty_IsClaimed()) {
                if (Loyalty_Claim(&received_data[1])) {
                    Loyalty_SendResponse(RESP_CLAIM_OK, 0x01, 0);
                    Unlock_OnOwnerAuthenticated();
                } else {
                    Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
                }
                break;
            }

            // Claimed: a matching token means the legitimate owner is
            // re-establishing a session (their phone still has the token but
            // BondManager / system state was cleared). Reply VERIFY_OK so the
            // app can distinguish "we accepted your CLAIM as a verify" from
            // "we wrote a fresh token". Mismatch is a different phone — REJECT.
            if (Loyalty_Verify(&received_data[1])) {
                Loyalty_SendResponse(RESP_VERIFY_OK, 0x01, 0);
                Unlock_OnOwnerAuthenticated();
                break;
            }
            Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
            break;
        }

        if (first_byte == CMD_VERIFY_OWNER) {
            if (data_length < 1 + LOYALTY_TOKEN_LEN ||
                !Loyalty_IsClaimed() ||
                !Loyalty_Verify(&received_data[1])) {
                Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
                break;
            }
            Loyalty_SendResponse(RESP_VERIFY_OK, 0x01, 0);
            Unlock_OnOwnerAuthenticated();
            break;
        }

        if (first_byte == CMD_UNBOND_DEVICE) {
            if (data_length < 1 + LOYALTY_TOKEN_LEN ||
                !Loyalty_IsClaimed() ||
                !Loyalty_Verify(&received_data[1])) {
                Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
                break;
            }
            // Always send UNPAIR_ACK + disconnect even if the EEPROM write
            // failed — iOS still needs to clear its local state, and
            // Loyalty_Wipe() has already cleared s_claimed unconditionally
            // so the next CLAIM in this session is accepted.
            (void)Loyalty_Wipe();
            Loyalty_SendResponse(RESP_UNPAIR_ACK, 0x01, 1);
            break;
        }

        // Regular command path — token-prefixed.
        if (!Loyalty_IsClaimed() ||
            data_length < LOYALTY_TOKEN_LEN + 1 ||
            !Loyalty_Verify(received_data)) {
            Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
            break;
        }

        uint8_t *cmd_data   = &received_data[LOYALTY_TOKEN_LEN];
        uint8_t  cmd_length = data_length - LOYALTY_TOKEN_LEN;
        uint8_t  command    = cmd_data[0];

        // Settings writes carry a trailing 6-byte timestamp; the dedicated
        // opcodes don't.
        if (cmd_length >= 7 && command != CMD_REQUEST_EVENT &&
            command != CMD_REQUEST_LOG_COUNT && command != CMD_CLEAR_LOG &&
            command != CMD_ACK_EVENT && command != CMD_FIND_MY_DEVICE &&
            command != CMD_RESET_DEVICE && command != CMD_REQUEST_DIAG &&
            command != CMD_DRAIN_MODE &&
            command != CMD_EE_READ && command != CMD_EE_WRITE) {
            UpdateBootTimeFromiOS(&cmd_data[cmd_length - 6]);
        }

        switch (command) {
            case CMD_REQUEST_LOG_COUNT:
                transferInProgress = 1;
                currentEventIndex = 0;
                LOCKSERVICE_SendEventCount();
                break;

            case CMD_REQUEST_EVENT:
                if (cmd_length >= 3) {
                    uint16_t requestedIndex = ((uint16_t)cmd_data[1] << 8) | cmd_data[2];
                    LOCKSERVICE_SendEvent(requestedIndex);
                }
                break;

            case CMD_ACK_EVENT:
                break;

            case CMD_CLEAR_LOG:
                MotionLogger_Clear();
                transferInProgress = 0;
                currentEventIndex = 0;
                LOCKSERVICE_SendLogCleared();
                break;

            case CMD_FIND_MY_DEVICE:
                if (cmd_length >= 2 && (cmd_data[1] & 0x01)) {
                    FindMyDevice_Start();
                }
                break;

            case CMD_RESET_DEVICE:
                NVIC_SystemReset();
                break;

            case CMD_DRAIN_MODE:
                if (cmd_length >= 2 && (cmd_data[1] & 0x01)) {
                    Drain_Start();
                } else {
                    Drain_Stop();
                }
                break;

            case CMD_REQUEST_DIAG: {
                // Optional byte[1] = section_mask. Default to all sections.
                uint8_t mask = (cmd_length >= 2) ? cmd_data[1] : 0xFF;
                LOCKSERVICE_SendDiagnostic(mask);
                break;
            }

            case CMD_EE_READ: {
                // Wire: [0xE5, offset, length]
                if (cmd_length < 3) {
                    Send_EeAck(RESP_EE_REJECT, (uint8_t)EE_SCRATCH_OUT_OF_BOUNDS);
                    break;
                }
                uint8_t offset = cmd_data[1];
                uint8_t length = cmd_data[2];
                if (length > EE_SCRATCH_MAX_CHUNK) {
                    Send_EeAck(RESP_EE_REJECT, (uint8_t)EE_SCRATCH_OUT_OF_BOUNDS);
                    break;
                }
                uint8_t buf[EE_SCRATCH_MAX_CHUNK];
                EeScratchStatus_t st = EeScratch_Read(offset, length, buf);
                if (st == EE_SCRATCH_OK) {
                    Send_EeReadResponse(offset, length, buf);
                } else {
                    Send_EeAck(RESP_EE_REJECT, (uint8_t)st);
                }
                break;
            }

            case CMD_EE_WRITE: {
                // Wire: [0xE6, offset, length, ...data]
                if (cmd_length < 3) {
                    Send_EeAck(RESP_EE_REJECT, (uint8_t)EE_SCRATCH_OUT_OF_BOUNDS);
                    break;
                }
                uint8_t offset = cmd_data[1];
                uint8_t length = cmd_data[2];
                if (length > EE_SCRATCH_MAX_CHUNK || cmd_length < (uint8_t)(3 + length)) {
                    Send_EeAck(RESP_EE_REJECT, (uint8_t)EE_SCRATCH_OUT_OF_BOUNDS);
                    break;
                }
                EeScratchStatus_t st = EeScratch_Write(offset, length, &cmd_data[3]);
                if (st == EE_SCRATCH_OK) {
                    Send_EeAck(RESP_EE_WRITE_ACK, 1);
                } else {
                    Send_EeAck(RESP_EE_REJECT, (uint8_t)st);
                }
                break;
            }

            default: {
                // Settings update. Post-token payload is
                //   [settings, deviceInfo, alarmDur?, ledBright?, bleTxPwr?, ts0..ts5]
                // The 6-byte timestamp tail is stripped above when
                // cmd_length >= 7 — discount it here so a length-2..5
                // settings core is recognised correctly.
                uint8_t settings_len = (cmd_length >= 7) ? (uint8_t)(cmd_length - 6)
                                                         : cmd_length;
                if (settings_len >= 1) {
                    deviceState = cmd_data[0];
                }
                if (settings_len >= 2) {
                    // Bit 0 = HIGH_PERF, bit 1 = alarmDisabled, bit 2 =
                    // disconnectSoundDisabled. Bits 3..7 reserved (mask off
                    // so reads stay clean).
                    deviceInfo = cmd_data[1] & 0x07;
                    (void)AlarmDisabled_Set((cmd_data[1] >> 1) & 0x01);
                    (void)DisconnectSoundDisabled_Set((cmd_data[1] >> 2) & 0x01);
                }
                if (settings_len >= 3) {
                    (void)AlarmDuration_Set(cmd_data[2]);
                }
                if (settings_len >= 4) {
                    (void)LedBrightness_Set(cmd_data[3]);
                }
                if (settings_len >= 5) {
                    // 0=NORMAL (0 dBm), 1=HIGH (+8 dBm). Set() clamps +
                    // persists + applies via aci_hal_set_tx_power_level.
                    (void)BleTxPower_Set((BleTxPower_t)cmd_data[4]);
                }
                // Persist the deviceState/deviceInfo bytes (ARMED bit
                // excluded). The other persisted records (alarm duration /
                // LED brightness / alarm-disabled / BLE TX power) already
                // wrote inside their own Set() calls above.
                DeviceSettings_Persist();
                HAL_Delay(5);
                LOCKSERVICE_ForceStatusUpdate();
                break;
            }
        }
      }
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
      PowerMgmt_RestoreAll();
      // RestoreAll runs LIS2DUX12_Init() which SW-resets and reloads the
      // UCF — INT1 glitches and the user is invariably handling the device
      // during connect. Suppress motion-triggered alarm transitions for a
      // short grace window so this doesn't fire the alarm.
      LIS2DUX12_ClearMotion();
      StateMachine_StartMotionGrace(2000);
      connectionStatus = 1;
      LOCKSERVICE_ForceStatusUpdate();

      // Motion-log drain is loyalty-gated: iOS calls requestMotionLogCount()
      // inside onLoyaltyVerifiedHook (0.5 s after RESP_CLAIM_OK / VERIFY_OK)
      // so the pending-event count is never leaked to an unverified phone.
      /* USER CODE END Service1_APP_CENTR_CONN_HANDLE_EVT */
      break;
    case LOCKSERVICE_DISCON_HANDLE_EVT :
      LOCKSERVICE_APP_Context.ConnectionHandle = 0xFFFF;
      /* USER CODE BEGIN Service1_APP_DISCON_HANDLE_EVT */
      connectionStatus = 0;
      SOUND_Disconnected();
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
    LOCKSERVICE_Devicestatus_SendNotification();
}

void LOCKSERVICE_ForceStatusUpdate(void)
{
    LOCKSERVICE_Devicestatus_SendNotification();
}

/***************************************************************************
 * LOCKSERVICE_SendDiagnostic — on-demand TLV diagnostic dump
 *
 * Replaces the old auto-1Hz BATTERYDIAG push. iOS asks for it via
 * CMD_REQUEST_DIAG (0xF4) on APPTOWD; the response is one notification on
 * the BATTERYDIAG characteristic (UUID unchanged for compatibility).
 *
 * Wire format (header + variable-length sections, all LE):
 *   byte 0  format_version  (= DIAG_FORMAT_VERSION)
 *   byte 1  section_count
 *   then `section_count` repetitions of:
 *     byte 0  section_id
 *     byte 1  section_len  (N)
 *     bytes 2..N+1  payload
 *
 * Section IDs (1-indexed in the mask — bit (id-1) of section_mask):
 *   0x01 SYSTEM         — uptime, boot count, reset cause, fw ver, init bitmask,
 *                         last-fault-present flag
 *   0x02 BATTERY        — full BQ27427 snapshot (schema v11)
 *   0x04 SENSOR         — cached MLC state, FSM event, transitions/int1 counters
 *   0x06 STORAGE        — motion-log count, loyalty health, error counters
 *   0x07 FAULT          — last hard-fault PC/LR/xPSR snapshot (see crash_forensics)
 *   0x08 RESET_HISTORY  — count + up to 8 (cause, boot_count, uptime) entries
 *
 * Section IDs 0x03 (BLE) and 0x05 (POWER) were removed when iOS dropped the
 * corresponding UI. The ID gaps are deliberate — iOS parsers are keyed by
 * section_id, so leaving the numbers alone avoids accidentally re-using
 * an ID for an unrelated future section.
 *
 * Per-section struct layouts are defined immediately below — keep the
 * _Static_assert sizes in sync if you add/remove fields.
 *
 * `section_mask` selects which sections to include. Bit i = include section
 * (i+1). 0xFF = all current sections (mask bits 2 and 4 for the removed
 * BLE / POWER are simply no-ops). iOS sends 0xFF by default.
 *
 * Each section's payload is sized for the on-wire layout with no padding;
 * reserved[] fields zero-fill remaining bytes so iOS parsers that read
 * fewer bytes than the section_len keep working when fields are added.
 ***************************************************************************/

#define DIAG_FORMAT_VERSION   1

#define DIAG_SECTION_SYSTEM        0x01
#define DIAG_SECTION_BATTERY       0x02
/* 0x03 BLE — removed; was RSSI / connect count / disconnect reason / MTU */
#define DIAG_SECTION_SENSOR        0x04
/* 0x05 POWER — removed; was wake-source counters + time-in-LP */
#define DIAG_SECTION_STORAGE       0x06
#define DIAG_SECTION_FAULT         0x07  /* hard-fault PC/LR/xPSR snapshot */
#define DIAG_SECTION_RESET_HISTORY 0x08  /* last 8 reset-cause ring entries */

/* Per-section payload structs — packed so the on-wire layout matches the
 * struct field order exactly. Every reserved[] field is zero-filled and
 * iOS is required to ignore trailing reserved bytes; that's how we add
 * fields later without breaking the app. */

typedef struct __attribute__((packed)) {
    uint32_t uptime_seconds;
    uint32_t boot_count;
    uint8_t  reset_cause;
    uint8_t  fw_version_major;
    uint8_t  fw_version_main;
    uint8_t  fw_version_v2;
    uint8_t  init_bitmask;
    uint8_t  last_fault_marker;
    uint8_t  reserved[5];
} diag_system_t;
_Static_assert(sizeof(diag_system_t) == 19, "diag_system_t must be 19 bytes");

typedef struct __attribute__((packed)) {
    uint8_t  version;                  /* = 11, the existing battery-diag schema */
    uint8_t  soc_percent;
    uint16_t voltage_mV;
    int16_t  current_mA;
    uint16_t remaining_mAh;
    uint16_t full_charge_mAh;
    int16_t  temperature_0_1K;
    uint16_t flags_raw;
    uint16_t control_status_raw;
    uint8_t  status_bits;
    uint8_t  soc_unfiltered;
    uint16_t design_capacity_mAh;
    uint16_t terminate_voltage_mV;
    uint16_t taper_rate;
    uint16_t op_config_raw;
    int16_t  average_power_mW;
    int8_t   board_offset;
    uint8_t  deadband_mA;
    uint8_t  calib_bytes[16];
    uint8_t  init_fail_stage;
    uint8_t  init_completed;
    uint8_t  post_reset_fired;
    uint16_t chem_id_read;
} diag_battery_t;
_Static_assert(sizeof(diag_battery_t) == 51, "diag_battery_t must be 51 bytes");

/* diag_ble_t removed — section 0x03 no longer emitted. */

typedef struct __attribute__((packed)) {
    uint8_t  cached_mlc_state;
    uint8_t  last_fsm_event;
    uint32_t mlc_transitions_since_boot;
    uint32_t int1_fires_since_boot;
    uint32_t motion_events_logged_since_boot;
    uint8_t  reserved[4];
} diag_sensor_t;
_Static_assert(sizeof(diag_sensor_t) == 18, "diag_sensor_t must be 18 bytes");

/* diag_power_t removed — section 0x05 no longer emitted. */

typedef struct __attribute__((packed)) {
    uint16_t motion_log_count;
    uint16_t motion_log_max;
    uint8_t  loyalty_store_healthy;
    uint8_t  loyalty_claimed;
    uint32_t i2c_errors_since_boot;
    uint32_t eeprom_fail_count;
    uint32_t bq27427_fail_count;
    uint32_t lis2dux12_fail_count;
    uint8_t  reserved[4];
} diag_storage_t;
_Static_assert(sizeof(diag_storage_t) == 26, "diag_storage_t must be 26 bytes");

/* FAULT section — last hard-fault snapshot. valid=0 means no fault has ever
 * been recorded (or it was cleared). Fields PC/LR/xPSR are from the stacked
 * exception frame the M0+ pushed at fault time. */
typedef struct __attribute__((packed)) {
    uint8_t  valid;
    uint8_t  schema_version;
    uint16_t reserved0;
    uint32_t pc;
    uint32_t lr;
    uint32_t xpsr;
} diag_fault_t;
_Static_assert(sizeof(diag_fault_t) == 16, "diag_fault_t must be 16 bytes");

/* RESET_HISTORY — count + up to 8 entries × (cause, boot_count, uptime).
 * count = number of entries actually populated (0..8). Each entry is 9
 * bytes; iOS reads section_len, infers count = (section_len - 1) / 9. */
#define DIAG_RESET_HISTORY_MAX_ENTRIES   8
typedef struct __attribute__((packed)) {
    uint8_t  reset_cause;
    uint32_t boot_count;
    uint32_t uptime_secs;
} diag_reset_entry_t;
_Static_assert(sizeof(diag_reset_entry_t) == 9, "diag_reset_entry_t must be 9 bytes");
typedef struct __attribute__((packed)) {
    uint8_t            count;
    diag_reset_entry_t entries[DIAG_RESET_HISTORY_MAX_ENTRIES];
} diag_reset_history_t;
_Static_assert(sizeof(diag_reset_history_t) == 73,
               "diag_reset_history_t must be 73 bytes");

/* Append `len` bytes of section `id` payload `src` to `dst[*offset]`,
 * including the 2-byte TLV header. Caller bumps the section count.
 * Bounds-checked against BATTERYDIAG_SIZE-equivalent caller-supplied cap. */
static void diag_append_section(uint8_t *dst, uint16_t cap, uint16_t *offset,
                                uint8_t id, const void *src, uint8_t len)
{
    if ((uint16_t)(*offset + 2u + len) > cap) {
        return; /* Silently drop — caller will report a smaller section_count */
    }
    dst[(*offset)++] = id;
    dst[(*offset)++] = len;
    memcpy(&dst[*offset], src, len);
    *offset += len;
}

void LOCKSERVICE_SendDiagnostic(uint8_t section_mask)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) {
        return;
    }

    /* Local staging buffer — sized to fit every TLV section in one shot.
     * Worst-case payload (all current sections + headers, no BLE/POWER):
     *   header 2 + SYSTEM 21 + BATTERY 53 + SENSOR 20 + STORAGE 28
     *         + FAULT 18 + RESET_HISTORY 75 = 217
     * Rounded up to 232 for slack. iOS / the BLE stack copies out before
     * we return, so a stack-allocated buffer is fine on the M0+ as long
     * as we don't approach the few-KB MSP budget. */
    uint8_t  buf[232];
    uint16_t offset = 0;
    uint8_t  count  = 0;

    /* Reserve header — fill in count once we know it. */
    buf[offset++] = DIAG_FORMAT_VERSION;
    uint16_t count_offset = offset++;

    /* SYSTEM */
    if (section_mask & (1u << 0)) {
        diag_system_t sys = {0};
        sys.uptime_seconds    = PowerMgmt_GetUptimeSeconds();
        sys.boot_count        = PowerMgmt_GetBootCount();
        sys.reset_cause       = PowerMgmt_GetResetCause();
        sys.fw_version_major  = FW_VERSION_MAJOR;
        sys.fw_version_main   = FW_VERSION_MAIN;
        sys.fw_version_v2     = FW_VERSION_V2;
        /* init_bitmask: until each subsystem reports its own status, the
         * one signal we have is the battery init stage. Treat any other
         * subsystem as "we got far enough to be running" = 1, and let
         * iOS surface battery-init failure via the BATTERY section's
         * init_fail_stage field for now. Future revisions will replace
         * this with per-subsystem flip-on-success bits. */
        sys.init_bitmask = 0x7F; /* bits 0..6 all "ok" placeholder */
        if (BATTERY_GetInitCompleted() == 0) {
            sys.init_bitmask &= ~(1u << 1); /* battery */
        }
        if (Loyalty_StoreUnhealthy()) {
            sys.init_bitmask &= ~(1u << 4); /* loyalty */
        }
        /* Promoted: 1 iff a hard-fault snapshot is sitting in EEPROM. iOS
         * can short-circuit asking for the FAULT section when this is 0. */
        sys.last_fault_marker = CrashForensics_GetLastFault().valid ? 1u : 0u;
        diag_append_section(buf, sizeof(buf), &offset, DIAG_SECTION_SYSTEM,
                            &sys, sizeof(sys));
        count++;
    }

    /* BATTERY */
    if (section_mask & (1u << 1)) {
        diag_battery_t bat = {0};
        bat.version              = 11;
        bat.soc_percent          = (uint8_t)(BATTERY_GetSOC() & 0xFF);
        bat.voltage_mV           = BATTERY_GetVoltage();
        bat.current_mA           = BATTERY_GetCurrent();
        bat.remaining_mAh        = BATTERY_GetRemainingCapacity();
        bat.full_charge_mAh      = BATTERY_GetFullChargeCapacity();
        bat.temperature_0_1K     = BATTERY_GetTemperature_0_1K();
        bat.flags_raw            = BATTERY_GetFlags();
        bat.control_status_raw   = BATTERY_GetControlStatus();

        uint8_t bits = 0;
        if (BATTERY_IsCharging())          bits |= (1u << 0);
        if (BATTERY_IsFullCached())        bits |= (1u << 1);
        if (BATTERY_IsLowCached())         bits |= (1u << 2);
        if (BATTERY_IsCriticallyCached())  bits |= (1u << 3);
        if (BATTERY_IsBatteryDetected())   bits |= (1u << 4);
        if (BATTERY_IsQmaxLearned())       bits |= (1u << 5);
        if (BATTERY_IsResistanceLearned()) bits |= (1u << 6);
        if (BATTERY_IsItpor())             bits |= (1u << 7);
        bat.status_bits          = bits;
        bat.soc_unfiltered       = BATTERY_GetSOC_Unfiltered();
        bat.design_capacity_mAh  = BATTERY_GetDesignCapacity();
        bat.terminate_voltage_mV = BATTERY_GetTerminateVoltage();
        bat.taper_rate           = BATTERY_GetTaperRate();
        bat.op_config_raw        = BATTERY_GetOpConfig();
        bat.average_power_mW     = BATTERY_GetAveragePower();
        bat.board_offset         = BATTERY_GetBoardOffset();
        bat.deadband_mA          = BATTERY_GetDeadband();
        memcpy(bat.calib_bytes, BATTERY_GetCalibBytes(), 16);
        bat.init_fail_stage      = BATTERY_GetInitFailStage();
        bat.init_completed       = BATTERY_GetInitCompleted();
        bat.post_reset_fired     = BATTERY_GetPostResetFired();
        bat.chem_id_read         = BATTERY_GetChemIdRead();
        diag_append_section(buf, sizeof(buf), &offset, DIAG_SECTION_BATTERY,
                            &bat, sizeof(bat));
        count++;
    }

    /* BLE section (0x03) removed — iOS does not surface this data. */

    /* SENSOR — current MLC state is already cached. The other counters
     * are TODO; emit 0 for now. */
    if (section_mask & (1u << 3)) {
        diag_sensor_t sen = {0};
        sen.cached_mlc_state                 = lis2dux12_app_get_cached_mlc_state();
        sen.last_fsm_event                   = 0;
        sen.mlc_transitions_since_boot       = 0;
        sen.int1_fires_since_boot            = 0;
        sen.motion_events_logged_since_boot  = 0;
        diag_append_section(buf, sizeof(buf), &offset, DIAG_SECTION_SENSOR,
                            &sen, sizeof(sen));
        count++;
    }

    /* POWER section (0x05) removed — iOS does not surface this data. */

    /* STORAGE */
    if (section_mask & (1u << 5)) {
        diag_storage_t sto = {0};
        sto.motion_log_count      = MotionLogger_GetEventCount();
        sto.motion_log_max        = MAX_MOTION_EVENTS;
        sto.loyalty_store_healthy = Loyalty_StoreUnhealthy() ? 0u : 1u;
        sto.loyalty_claimed       = Loyalty_IsClaimed() ? 1u : 0u;
        sto.i2c_errors_since_boot = 0;
        sto.eeprom_fail_count     = 0;
        sto.bq27427_fail_count    = 0;
        sto.lis2dux12_fail_count  = 0;
        diag_append_section(buf, sizeof(buf), &offset, DIAG_SECTION_STORAGE,
                            &sto, sizeof(sto));
        count++;
    }

    /* FAULT — last hard-fault snapshot (PC/LR/xPSR). All zeros + valid=0
     * when no fault has ever fired since the EEPROM was wiped. */
    if (section_mask & (1u << 6)) {
        FaultSnapshot_t snap = CrashForensics_GetLastFault();
        diag_fault_t flt = {0};
        flt.valid          = snap.valid;
        flt.schema_version = snap.schema_version;
        flt.pc             = snap.pc;
        flt.lr             = snap.lr;
        flt.xpsr           = snap.xpsr;
        diag_append_section(buf, sizeof(buf), &offset, DIAG_SECTION_FAULT,
                            &flt, sizeof(flt));
        count++;
    }

    /* RESET_HISTORY — up to 8 most-recent reset events (oldest first).
     * iOS reads count and iterates count × 9 bytes of entries. Unused tail
     * slots stay zero-filled so trailing-byte ignore still works. */
    if (section_mask & (1u << 7)) {
        diag_reset_history_t rh = {0};
        uint8_t n = CrashForensics_GetResetEventCount();
        if (n > DIAG_RESET_HISTORY_MAX_ENTRIES) n = DIAG_RESET_HISTORY_MAX_ENTRIES;
        rh.count = n;
        for (uint8_t i = 0; i < n; i++) {
            uint8_t  cause = 0;
            uint32_t bc    = 0;
            uint32_t upt   = 0;
            if (CrashForensics_GetResetEvent(i, &cause, &bc, &upt)) {
                rh.entries[i].reset_cause = cause;
                rh.entries[i].boot_count  = bc;
                rh.entries[i].uptime_secs = upt;
            }
        }
        diag_append_section(buf, sizeof(buf), &offset, DIAG_SECTION_RESET_HISTORY,
                            &rh, sizeof(rh));
        count++;
    }

    buf[count_offset] = count;

    LOCKSERVICE_Data_t notification_data;
    notification_data.p_Payload = buf;
    notification_data.Length    = offset;

    LOCKSERVICE_NotifyValue(LOCKSERVICE_BATTERYDIAG, &notification_data,
                            LOCKSERVICE_APP_Context.ConnectionHandle);
}

/***************************************************************************
 * Drain Mode — gauge-health diagnostic
 *   White LED at full brightness + continuous DRAIN_TONE_FREQUENCY_HZ
 *   buzzer tone. Auto-stops at DRAIN_AUTO_STOP_SOC. Drain_Tick() runs every
 *   loop iteration so other subsystems can't override outputs while active.
 ***************************************************************************/

void Drain_Start(void)
{
    if (drain_mode_active) return;
    drain_mode_active = 1;
    LED_Solid(255, 255, 255, 255);
    BUZZER_StartContinuousTone(DRAIN_TONE_FREQUENCY_HZ);
}

void Drain_Stop(void)
{
    if (!drain_mode_active) return;
    drain_mode_active = 0;
    BUZZER_Stop();
    LED_Off();
}

uint8_t Drain_IsActive(void)
{
    return drain_mode_active;
}

void Drain_Tick(void)
{
    if (!drain_mode_active) return;

    if (BATTERY_GetSOC() <= DRAIN_AUTO_STOP_SOC) {
        Drain_Stop();
        return;
    }

    LED_Solid(255, 255, 255, 255);
    if (!BUZZER_IsPlaying()) {
        BUZZER_StartContinuousTone(DRAIN_TONE_FREQUENCY_HZ);
    }
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
  	notification_on_off = Devicestatus_NOTIFICATION_ON;

    uint16_t voltage_mV = BATTERY_GetVoltage();
    int16_t current_mA = BATTERY_GetCurrent();
    uint16_t soc_percent = BATTERY_GetSOC();

    deviceBattery = soc_percent & 0x7F;

    if (IS_CABLE_PLUGGED() && IS_CHARGING_NOW() && !BATTERY_IsFullCached()) {
        SET_BATTERY_CHARGING(deviceBattery);
    } else {
        CLEAR_BATTERY_CHARGING(deviceBattery);
    }

    // Live I2C accel read costs ~3-5 ms per call. At the 40 ms status-notify
    // cadence that's tolerable, but during ALARM_ACTIVE every ms of main-
    // loop block stretches the buzzer note (TIM16 PWM keeps running but
    // BUZZER_Update can't advance). Zero the bytes during alarm — iOS only
    // uses the per-axis data outside of an alarming session.
    int16_t accel[3] = {0, 0, 0};
    if (currentState != STATE_ALARM_ACTIVE) {
        LIS2DUX12_ReadAcceleration(accel);
    }

    // 19-byte DEVICESTATUS payload. Bytes 14..15 carry the low 2 bytes of
    // the BD address (LE) — used by the iOS app as the user-visible
    // "WatchDog #" identifier. Bytes 16..18 are firmware version
    // (MAJOR, MAIN, V2) — see firmware_version.h.
    a_LOCKSERVICE_UpdateCharData[0]  = deviceState;
    a_LOCKSERVICE_UpdateCharData[1]  = deviceBattery;
    a_LOCKSERVICE_UpdateCharData[2]  = (uint8_t)(current_mA & 0xFF);
    a_LOCKSERVICE_UpdateCharData[3]  = (uint8_t)((current_mA >> 8) & 0xFF);
    a_LOCKSERVICE_UpdateCharData[4]  = (uint8_t)(voltage_mV & 0xFF);
    a_LOCKSERVICE_UpdateCharData[5]  = (uint8_t)((voltage_mV >> 8) & 0xFF);
    a_LOCKSERVICE_UpdateCharData[6]  = lis2dux12_app_get_cached_mlc_state();
    a_LOCKSERVICE_UpdateCharData[7]  = (uint8_t)(accel[0] & 0xFF);
    a_LOCKSERVICE_UpdateCharData[8]  = (uint8_t)((accel[0] >> 8) & 0xFF);
    a_LOCKSERVICE_UpdateCharData[9]  = (uint8_t)(accel[1] & 0xFF);
    a_LOCKSERVICE_UpdateCharData[10] = (uint8_t)((accel[1] >> 8) & 0xFF);
    a_LOCKSERVICE_UpdateCharData[11] = (uint8_t)(accel[2] & 0xFF);
    a_LOCKSERVICE_UpdateCharData[12] = (uint8_t)((accel[2] >> 8) & 0xFF);
    // Bit 0 = HIGH_PERF, bit 1 = alarmDisabled, bit 2 = disconnectSoundDisabled,
    // bit 3  = BLE TX power level (0=NORMAL, 1=HIGH). The flag bits are
    // sourced from their persisted stores so they survive boot.
    // Bits 4..7 reserved.
    a_LOCKSERVICE_UpdateCharData[13] = (uint8_t)((deviceInfo & 0x01)
                                                | (AlarmDisabled_Get() ? 0x02 : 0)
                                                | (DisconnectSoundDisabled_Get() ? 0x04 : 0)
                                                | ((BleTxPower_Get() == BLE_TX_POWER_HIGH) ? 0x08 : 0));
    a_LOCKSERVICE_UpdateCharData[14] = g_bd_address[0];
    a_LOCKSERVICE_UpdateCharData[15] = g_bd_address[1];
    a_LOCKSERVICE_UpdateCharData[16] = FW_VERSION_MAJOR;
    a_LOCKSERVICE_UpdateCharData[17] = FW_VERSION_MAIN;
    a_LOCKSERVICE_UpdateCharData[18] = FW_VERSION_V2;

    lockservice_notification_data.Length = 19;
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
