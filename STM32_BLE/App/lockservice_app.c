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
#include "battery.h"
#include "accelerometer.h"
#include "lis2dux12_app.h"
#include "power_management.h"
#include "loyalty.h"
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
extern uint8_t g_bd_address[6]; /* defined in app_ble.c, populated in BLE_Init() */
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

// Track current transfer state
static uint16_t currentEventIndex = 0;
static uint8_t transferInProgress = 0;

static uint8_t drain_mode_active = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void LOCKSERVICE_Devicestatus_SendNotification(void);

/* USER CODE BEGIN PFP */
/**
 * @brief Send motion alert notification to iOS
 * This triggers iOS to auto-sync
 */
void LOCKSERVICE_SendMotionAlert(uint8_t motionType)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) {
        return;
    }

    /* Motion alert: [0xFF, motionType, battery] — 3 bytes */
    a_LOCKSERVICE_UpdateCharData[0] = 0xFF;          /* motion alert marker */
    a_LOCKSERVICE_UpdateCharData[1] = motionType;    /* MLC/FSM classification */
    a_LOCKSERVICE_UpdateCharData[2] = deviceBattery;

    LOCKSERVICE_Data_t lockservice_notification_data;
    lockservice_notification_data.p_Payload = (uint8_t*)a_LOCKSERVICE_UpdateCharData;
    lockservice_notification_data.Length = 3;

    LOCKSERVICE_NotifyValue(LOCKSERVICE_DEVICESTATUS, &lockservice_notification_data,
                           LOCKSERVICE_APP_Context.ConnectionHandle);
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

    // Pack event data
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

    // Add deviceBattery to match your status update format
    a_LOCKSERVICE_UpdateCharData[dataIdx++] = deviceBattery;

    LOCKSERVICE_Data_t lockservice_notification_data;
    lockservice_notification_data.p_Payload = (uint8_t*)a_LOCKSERVICE_UpdateCharData;
    lockservice_notification_data.Length = dataIdx;  // Should be 11 now

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

/**
 * @brief Send a 2-byte loyalty response on DEVICESTATUS, optionally followed
 *        by a deferred disconnect. Used by CLAIM/VERIFY/UNBOND/REJECT flows.
 */
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
        HAL_Delay(50);  /* let the radio TX the notify */
        (void)aci_gap_terminate(LOCKSERVICE_APP_Context.ConnectionHandle,
                                0x13 /* REMOTE_USER_TERMINATED */);
    }
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
      StateMachine_UpdateBLEActivity();
      {
        uint8_t *received_data = p_Notification->DataTransfered.p_Payload;
        uint8_t  data_length   = p_Notification->DataTransfered.Length;

        if (data_length == 0) {
            break;
        }

        /* If the EEPROM read at boot failed, refuse everything. Otherwise a
         * transient I2C glitch at boot would let any phone CLAIM (and thus
         * hijack) a device that is actually owned. */
        if (Loyalty_StoreUnhealthy()) {
            APP_DBG_MSG("Loyalty store UNHEALTHY - rejecting all writes\n");
            Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
            break;
        }

        /* ── Loyalty layer ────────────────────────────────────────────
         * Three special opcodes are self-contained:
         *   CLAIM  : [0xC1, t0, t1, t2, t3]            (5 bytes)
         *   VERIFY : [0xC2, t0, t1, t2, t3]            (5 bytes)
         *   UNBOND : [0xC0, t0, t1, t2, t3]            (5 bytes)
         *
         * Every other (existing) opcode is now prefixed with the 4-byte
         * token, i.e. [t0, t1, t2, t3, opcode, ...payload]. */
        uint8_t first_byte = received_data[0];

        if (first_byte == CMD_CLAIM_DEVICE) {
            if (data_length < 1 + LOYALTY_TOKEN_LEN) {
                Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
                break;
            }

            /* If already claimed, only accept a CLAIM whose token matches the
             * existing one - that's the legitimate owner with stale local
             * state (app reinstall, BondManager cleared, etc.). Treat it as
             * a successful re-claim so iOS BondManager re-adds the device.
             * A non-matching CLAIM (different phone) is still rejected. */
            if (Loyalty_IsClaimed()) {
                if (Loyalty_Verify(&received_data[1])) {
                    APP_DBG_MSG("CLAIM: token matches existing - idempotent re-claim accepted\n");
                    Loyalty_SendResponse(RESP_CLAIM_OK, 0x01, 0);
                } else {
                    APP_DBG_MSG("CLAIM: token mismatch with existing - REJECT\n");
                    Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
                }
                break;
            }

            /* Unowned: persist the new token. */
            if (Loyalty_Claim(&received_data[1])) {
                Loyalty_SendResponse(RESP_CLAIM_OK, 0x01, 0);
            } else {
                Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
            }
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
            break;
        }

        if (first_byte == CMD_UNBOND_DEVICE) {
            if (data_length < 1 + LOYALTY_TOKEN_LEN) {
                APP_DBG_MSG("UNBOND: rejected - data length %u < 5\n", (unsigned)data_length);
                Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
                break;
            }
            if (!Loyalty_IsClaimed()) {
                APP_DBG_MSG("UNBOND: rejected - device not claimed\n");
                Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
                break;
            }
            if (!Loyalty_Verify(&received_data[1])) {
                APP_DBG_MSG("UNBOND: rejected - token mismatch\n");
                Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
                break;
            }
            APP_DBG_MSG("UNBOND: token verified, calling Loyalty_Wipe...\n");
            bool wipe_ok = Loyalty_Wipe();
            APP_DBG_MSG("UNBOND: Loyalty_Wipe returned %d (true=sentinel persisted)\n",
                        (int)wipe_ok);
            /* Always send UNPAIR_ACK + disconnect - the iOS app needs to clean
             * up its local state regardless of EEPROM outcome. Loyalty_Wipe()
             * has already cleared s_claimed unconditionally, so the next CLAIM
             * in this session will be accepted. */
            Loyalty_SendResponse(RESP_UNPAIR_ACK, 0x01, 1);
            break;
        }

        /* ── Regular command path (token-prefixed) ──────────────────── */
        if (!Loyalty_IsClaimed() ||
            data_length < LOYALTY_TOKEN_LEN + 1 ||
            !Loyalty_Verify(received_data)) {
            Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
            break;
        }

        /* Token verified — strip the prefix and dispatch the rest with
         * the existing per-opcode logic. */
        uint8_t *cmd_data   = &received_data[LOYALTY_TOKEN_LEN];
        uint8_t  cmd_length = data_length - LOYALTY_TOKEN_LEN;
        uint8_t  command    = cmd_data[0];

        /* Only extract timestamp on settings-update writes (as before). */
        if (cmd_length >= 7 && command != CMD_REQUEST_EVENT &&
            command != CMD_REQUEST_LOG_COUNT && command != CMD_CLEAR_LOG &&
            command != CMD_ACK_EVENT && command != CMD_FIND_MY_DEVICE &&
            command != CMD_RESET_DEVICE) {
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
                /* iOS acknowledged receiving event */
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

            default:
                /* Regular device state update */
                deviceState = cmd_data[0];
                if (cmd_length >= 2) {
                    deviceInfo = cmd_data[1];
                }
                HAL_Delay(5);
                LOCKSERVICE_ForceStatusUpdate();
                break;
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
      /* RestoreAll runs LIS2DUX12_Init() which SW-resets and reloads
       * the UCF — INT1 glitches and the user is invariably handling
       * the device.  Suppress motion-triggered alarm transitions for
       * a short grace window so this doesn't fire the alarm. */
      LIS2DUX12_ClearMotion();
      StateMachine_StartMotionGrace(2000);
      StateMachine_UpdateBLEActivity();
      connectionStatus = 1;
      LOCKSERVICE_ForceStatusUpdate();  // Force send on connection

      /* If events were logged while disconnected, notify the app
       * so it can pull them via the existing request/response protocol. */
      if (MotionLogger_GetEventCount() > 0) {
          LOCKSERVICE_SendEventCount();
      }
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

/**
 * @brief Build and send the BatteryDiagnostic notification.
 *
 * Wire format (51 bytes, little-endian, version 11):
 *   bytes  0..29 — same layout as v3 (with version byte = 11)
 *   bytes 30..45 — uint8_t calib_bytes[16] from Subclass 104 (Calibration)
 *                  per BQ27427 TRM:
 *                    0..3  CC Gain    (4-byte TI custom float)
 *                    4..7  CC Delta   (4-byte TI custom float)
 *                    8..9  CC Offset  (int16)
 *                    10..11 candidate Board Offset (silicon-rev dependent)
 *                    12..15 spare/other
 *   byte  46    — uint8 init_fail_stage   (0 = ok; codes in battery.c)
 *   byte  47    — uint8 init_completed    (1 if BATTERY_Init reached the end)
 *   byte  48    — uint8 post_reset_fired  (1 if CC-Gain self-heal triggered RESET)
 *   bytes 49..50 — uint16 chem_id_read    (BQ27427 chem_id() snapshot, LE)
 *
 * Original v3 layout:
 *   uint8_t  version             = 3
 *   uint8_t  soc_percent         (filtered, 0-100)
 *   uint16_t voltage_mV
 *   int16_t  current_mA          (negative = discharging)
 *   uint16_t remaining_mAh
 *   uint16_t full_charge_mAh
 *   int16_t  temperature_0_1K    (divide by 10, subtract 273.15 for °C)
 *   uint16_t flags_raw           (BQ27427 Flags() register)
 *   uint16_t control_status_raw  (BQ27427 CONTROL_STATUS register)
 *   uint8_t  status_bits         (packed convenience flags, see below)
 *   uint8_t  soc_unfiltered      (raw IT SOC, 0-100)
 *   --- v3 fields (config readback + power + calibration) ---
 *   uint16_t design_capacity_mAh    (expected: 300)
 *   uint16_t terminate_voltage_mV   (expected: 3000)
 *   uint16_t taper_rate             (expected: 100)
 *   uint16_t op_config_raw          (expected: 0x6458 — SLEEP cleared)
 *   int16_t  average_power_mW       (signed; negative = discharging)
 *   int8_t   board_offset           (signed counts; expected: 0)
 *   uint8_t  deadband_mA            (expected: 5)
 *
 * status_bits layout (LSB first):
 *   bit 0: is_charging   (FLAG_CHG)
 *   bit 1: is_full       (FLAG_FC)
 *   bit 2: is_low        (FLAG_SOC1)
 *   bit 3: is_critical   (FLAG_SOCF)
 *   bit 4: bat_detected  (FLAG_BAT_DET)
 *   bit 5: qmax_learned  (CTRL_STATUS bit 9)
 *   bit 6: res_learned   (CTRL_STATUS bit 8)
 *   bit 7: itpor         (FLAG_ITPOR)
 *
 * STM32 is little-endian; multi-byte fields are written via direct memcpy
 * of the packed struct, which matches the LE wire spec.
 */
void LOCKSERVICE_SendBatteryDiagnostic(void)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) {
        return;
    }

    typedef struct __attribute__((packed)) {
        uint8_t  version;
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
        // --- v9 one-shot Calibration subclass dump (temporary) ---
        uint8_t  calib_bytes[16];
        // --- v10 init-failure tracker (temporary) ---
        uint8_t  init_fail_stage;
        uint8_t  init_completed;
        uint8_t  post_reset_fired;
        // --- v11 chem_id snapshot (temporary) ---
        uint16_t chem_id_read;
    } battery_diag_payload_t;

    _Static_assert(sizeof(battery_diag_payload_t) == 51,
                   "BatteryDiagnostic payload must be exactly 51 bytes");

    battery_diag_payload_t payload;
    payload.version              = 11;
    payload.soc_percent          = (uint8_t)(BATTERY_GetSOC() & 0xFF);
    payload.voltage_mV           = BATTERY_GetVoltage();
    payload.current_mA           = BATTERY_GetCurrent();
    payload.remaining_mAh        = BATTERY_GetRemainingCapacity();
    payload.full_charge_mAh      = BATTERY_GetFullChargeCapacity();
    payload.temperature_0_1K     = BATTERY_GetTemperature_0_1K();
    payload.flags_raw            = BATTERY_GetFlags();
    payload.control_status_raw   = BATTERY_GetControlStatus();

    uint8_t bits = 0;
    if (BATTERY_IsCharging())          bits |= (1u << 0);
    if (BATTERY_IsFullCached())        bits |= (1u << 1);
    if (BATTERY_IsLowCached())         bits |= (1u << 2);
    if (BATTERY_IsCriticallyCached())  bits |= (1u << 3);
    if (BATTERY_IsBatteryDetected())   bits |= (1u << 4);
    if (BATTERY_IsQmaxLearned())       bits |= (1u << 5);
    if (BATTERY_IsResistanceLearned()) bits |= (1u << 6);
    if (BATTERY_IsItpor())             bits |= (1u << 7);
    payload.status_bits          = bits;
    payload.soc_unfiltered       = BATTERY_GetSOC_Unfiltered();

    payload.design_capacity_mAh  = BATTERY_GetDesignCapacity();
    payload.terminate_voltage_mV = BATTERY_GetTerminateVoltage();
    payload.taper_rate           = BATTERY_GetTaperRate();
    payload.op_config_raw        = BATTERY_GetOpConfig();
    payload.average_power_mW     = BATTERY_GetAveragePower();
    payload.board_offset         = BATTERY_GetBoardOffset();
    payload.deadband_mA          = BATTERY_GetDeadband();
    memcpy(payload.calib_bytes, BATTERY_GetCalibBytes(), 16);
    payload.init_fail_stage      = BATTERY_GetInitFailStage();
    payload.init_completed       = BATTERY_GetInitCompleted();
    payload.post_reset_fired     = BATTERY_GetPostResetFired();
    payload.chem_id_read         = BATTERY_GetChemIdRead();

    LOCKSERVICE_Data_t notification_data;
    notification_data.p_Payload = (uint8_t *)&payload;
    notification_data.Length    = sizeof(payload);

    LOCKSERVICE_NotifyValue(LOCKSERVICE_BATTERYDIAG, &notification_data,
                            LOCKSERVICE_APP_Context.ConnectionHandle);
}

/******************************************************************************
 * Drain Mode — gauge-health diagnostic
 *
 * Drives the device into a high-load state so the fuel gauge can characterise
 * the battery: white LED at full brightness + continuous 100 Hz buzzer tone.
 * Auto-stops once SOC drops to DRAIN_AUTO_STOP_SOC (5 %).
 *****************************************************************************/

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

    /* Auto-stop when battery is sufficiently drained. */
    if (BATTERY_GetSOC() <= DRAIN_AUTO_STOP_SOC) {
        Drain_Stop();
        return;
    }

    /* Re-assert outputs every tick so other subsystems (state machine,
     * alarm patterns) can't override us while drain is active. */
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

    /* Use cached battery values */
    uint16_t voltage_mV = BATTERY_GetVoltage();
    int16_t current_mA = BATTERY_GetCurrent();
    uint16_t soc_percent = BATTERY_GetSOC();

    deviceBattery = soc_percent & 0x7F;

    /* Set charging flag: cable plugged AND actually charging AND gauge not full */
    if (IS_CABLE_PLUGGED() && IS_CHARGING_NOW() && !BATTERY_IsFullCached()) {
        SET_BATTERY_CHARGING(deviceBattery);
    } else {
        CLEAR_BATTERY_CHARGING(deviceBattery);
    }

    /* Read live accelerometer data */
    int16_t accel[3];
    LIS2DUX12_ReadAcceleration(accel);

    /* Pack data into BLE notification — 16 bytes
     * Bytes 14..15 carry the low 2 bytes of the BD address (LE), used by
     * the iOS app as the user-visible "WatchDog #" identifier in Settings. */
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
    a_LOCKSERVICE_UpdateCharData[13] = deviceInfo;
    a_LOCKSERVICE_UpdateCharData[14] = g_bd_address[0]; /* WatchDog # low byte  (LSB of BD addr) */
    a_LOCKSERVICE_UpdateCharData[15] = g_bd_address[1]; /* WatchDog # high byte */

    lockservice_notification_data.Length = 16;
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
