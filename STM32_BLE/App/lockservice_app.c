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
 * LOCKSERVICE_SendMotionAlert — push a 3-byte motion alert over DEVICESTATUS
 *   Payload: [0xFF, motionType, deviceBattery]. iOS uses the 0xFF marker
 *   to drive auto-sync on receipt.
 ***************************************************************************/
void LOCKSERVICE_SendMotionAlert(uint8_t motionType)
{
    if (LOCKSERVICE_APP_Context.ConnectionHandle == 0xFFFF) {
        return;
    }

    a_LOCKSERVICE_UpdateCharData[0] = 0xFF;
    a_LOCKSERVICE_UpdateCharData[1] = motionType;
    a_LOCKSERVICE_UpdateCharData[2] = deviceBattery;

    LOCKSERVICE_Data_t lockservice_notification_data;
    lockservice_notification_data.p_Payload = (uint8_t*)a_LOCKSERVICE_UpdateCharData;
    lockservice_notification_data.Length = 3;

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
 *   Wire format on hit (11 bytes):
 *     [RESP_EVENT_DATA, idx_hi, idx_lo, YY, MM, DD, hh, mm, ss, type, batt]
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
    MotionLogger_TickToDateTime(event->timestamp_ms, &year, &month, &day, &hour, &minute, &second);

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

            // Already claimed: only accept a CLAIM whose token matches —
            // that's the legitimate owner with stale local state (app
            // reinstall, BondManager cleared, etc.). Treat it as a
            // successful re-claim. Any other token is a different phone.
            if (Loyalty_IsClaimed()) {
                if (Loyalty_Verify(&received_data[1])) {
                    Loyalty_SendResponse(RESP_CLAIM_OK, 0x01, 0);
                } else {
                    Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
                }
                break;
            }

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

            default: {
                // Settings update. Post-token payload is
                //   [settings, deviceInfo, alarmDur?, ledBright?, ts0..ts5]
                // The 6-byte timestamp tail is stripped above when
                // cmd_length >= 7 — discount it here so a length-2/3/4
                // settings core is recognised correctly.
                uint8_t settings_len = (cmd_length >= 7) ? (uint8_t)(cmd_length - 6)
                                                         : cmd_length;
                if (settings_len >= 1) {
                    deviceState = cmd_data[0];
                }
                if (settings_len >= 2) {
                    // Bit 0 = HIGH_PERF, bit 1 = alarmDisabled, bits 2..7
                    // reserved (mask off so reads stay clean).
                    deviceInfo = cmd_data[1] & 0x03;
                    (void)AlarmDisabled_Set((cmd_data[1] >> 1) & 0x01);
                }
                if (settings_len >= 3) {
                    (void)AlarmDuration_Set(cmd_data[2]);
                }
                if (settings_len >= 4) {
                    (void)LedBrightness_Set(cmd_data[3]);
                }
                // Persist the deviceState/deviceInfo bytes (ARMED bit
                // excluded). The other persisted records (alarm duration /
                // LED brightness / alarm-disabled) already wrote inside
                // their own Set() calls above.
                DeviceSettings_Persist();
                APP_DBG_MSG("Recv settings · 0x%02X deviceInfo 0x%02X alarmDur=%us ledBright=%u alarmDisabled=%u\n",
                            deviceState, deviceInfo,
                            AlarmDuration_Get(), LedBrightness_Get(),
                            AlarmDisabled_Get() ? 1u : 0u);
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

      // Drain any events logged while disconnected so iOS can pull them.
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
    LOCKSERVICE_Devicestatus_SendNotification();
}

void LOCKSERVICE_ForceStatusUpdate(void)
{
    LOCKSERVICE_Devicestatus_SendNotification();
}

/***************************************************************************
 * LOCKSERVICE_SendBatteryDiagnostic — 51-byte BATTERYDIAG payload (v11)
 *
 * Wire format (little-endian, packed):
 *   uint8   version              = 11
 *   uint8   soc_percent          (filtered, 0..100)
 *   uint16  voltage_mV
 *   int16   current_mA           (negative = discharging)
 *   uint16  remaining_mAh
 *   uint16  full_charge_mAh
 *   int16   temperature_0_1K     (÷10 then -273.15 for °C)
 *   uint16  flags_raw            (BQ27427 Flags() register)
 *   uint16  control_status_raw
 *   uint8   status_bits          (packed convenience flags, see below)
 *   uint8   soc_unfiltered       (raw IT SOC, 0..100)
 *   uint16  design_capacity_mAh  (expected: 300)
 *   uint16  terminate_voltage_mV (expected: 3000)
 *   uint16  taper_rate           (expected: 100)
 *   uint16  op_config_raw        (expected: 0x6458 — SLEEP cleared)
 *   int16   average_power_mW     (signed; negative = discharging)
 *   int8    board_offset
 *   uint8   deadband_mA          (expected: 5)
 *   uint8   calib_bytes[16]      Subclass 104 dump (CC Gain/Delta/Offset)
 *   uint8   init_fail_stage      0 = ok; codes documented in battery.c
 *   uint8   init_completed       1 if BATTERY_Init reached the end
 *   uint8   post_reset_fired     1 if the CC-Gain self-heal RESET fired
 *   uint16  chem_id_read         BQ27427 chem_id() snapshot
 *
 * status_bits (LSB first):
 *   0 is_charging   (FLAG_CHG)
 *   1 is_full       (FLAG_FC)
 *   2 is_low        (FLAG_SOC1)
 *   3 is_critical   (FLAG_SOCF)
 *   4 bat_detected  (FLAG_BAT_DET)
 *   5 qmax_learned  (CTRL_STATUS bit 9)
 *   6 res_learned   (CTRL_STATUS bit 8)
 *   7 itpor         (FLAG_ITPOR)
 ***************************************************************************/
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
        uint8_t  calib_bytes[16];
        uint8_t  init_fail_stage;
        uint8_t  init_completed;
        uint8_t  post_reset_fired;
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

    int16_t accel[3];
    LIS2DUX12_ReadAcceleration(accel);

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
    // Bit 0 = HIGH_PERF, bit 1 = alarmDisabled (sourced from the persisted
    // store so the value survives boot). Bits 2..7 reserved.
    a_LOCKSERVICE_UpdateCharData[13] = (uint8_t)((deviceInfo & 0x01)
                                                | (AlarmDisabled_Get() ? 0x02 : 0));
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
