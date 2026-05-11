/***************************************************************************
 * motion_logger.h
 * created by Sebastian Forenza 2026
 *
 * Motion-event ring buffer API. SRAM-backed for fast access; mirrored to
 * the M24C08 EEPROM so events survive DEEPSTOP and power loss.
 *
 * Per-event storage is the CALENDAR TIME at the moment of logging, not a
 * HAL tick — see the long comment in motion_logger.c on the architecture
 * change. Briefly: storing ticks made events un-resolvable after any anchor
 * move (every settings write re-anchored) and across resets (HAL_GetTick
 * restarts at 0). Storing seconds-since-2000 captured at log time freezes
 * each event's calendar at the moment it happened; nothing downstream can
 * corrupt it.
 *
 * EEPROM map (M24C08, 1024 B):
 *   0x000..0x03F  device info (64 B, reserved)
 *   0x040..0x047  motion-log header (8 B)
 *   0x048..0x3FF  motion event data (952 B)
 ***************************************************************************/

#ifndef MOTION_LOGGER_H
#define MOTION_LOGGER_H

#include "main.h"
#include "stm32wb0x_hal.h"

#define MAX_MOTION_EVENTS 169

#define EEPROM_DEVICE_INFO_ADDR      0x000
#define EEPROM_DEVICE_INFO_SIZE      64

/* Boot-time anchor (calendar + monotonic-radio-timer seconds at iOS sync).
 * Lives inside the 0x000..0x03F reserved device-info block. Layout:
 *   [0]    magic 0xB7
 *   [1..6] year (offset from 2000) / month / day / hour / minute / second
 *   [7..10] boot_monotonic_secs (uint32 LE) — seconds reported by
 *           HAL_RADIO_TIMER_GetCurrentSysTime() / 409600 at the moment
 *           iOS pushed the anchor. NOT HAL_GetTick(): SysTick is suspended
 *           in DEEPSTOP so HAL_GetTick deltas miss every sleep period.
 *
 * The anchor is RAM-only authority from iOS each session — we no longer
 * reload it at boot. EEPROM persistence is kept for diagnostic / forensic
 * inspection; MotionLogger_Init does NOT consume it. */
#define EEPROM_BOOT_TIME_ADDR        0x00
#define EEPROM_BOOT_TIME_LEN         11
#define EEPROM_BOOT_TIME_MAGIC       0xB7

#define EEPROM_MOTION_HEADER_ADDR    0x040
#define EEPROM_MOTION_HEADER_SIZE    8

#define EEPROM_MOTION_DATA_ADDR      0x048
#define EEPROM_MOTION_EVENT_SIZE     5      // 4 epoch_seconds_2000 + 1 type

/* Bumped from 0xA5 → 0xA6 when the per-slot timestamp semantics changed
 * from "HAL tick milliseconds (this-boot)" to "seconds since 2000-01-01
 * 00:00:00 UTC, captured at log time". Old EEPROMs with 0xA5 will load
 * as empty after the upgrade — those legacy events couldn't be displayed
 * correctly anyway. */
#define EEPROM_MAGIC_BYTE            0xA6
#define EEPROM_I2C_ADDRESS           0x50   // M24C08 with A2=0

typedef enum {
    MOTION_TYPE_NONE       = 0,
    MOTION_TYPE_IN_MOTION  = 1,  // MLC: general movement detected
    MOTION_TYPE_SHAKEN     = 2,  // MLC: device was shaken
    MOTION_TYPE_IMPACT     = 3,  // FSM: impact event
    MOTION_TYPE_FREEFALL   = 4,  // FSM: free-fall event
} MotionType_t;

typedef struct {
    /* Seconds since 2000-01-01 00:00:00 (local-time per the iOS anchor —
     * the firmware doesn't know about timezones). 0 = "unknown time"; iOS
     * renders that as the sentinel calendar (0,1,1,0,0,0) which its parser
     * maps to nil. uint32 gives ~136 years headroom. */
    uint32_t epoch_seconds_2000;
    MotionType_t motionType;
    uint8_t valid;               // 1 if entry is valid
} MotionEvent_t;

void MotionLogger_Init(void);
uint8_t MotionLogger_LogEvent(MotionType_t motionType);
uint16_t MotionLogger_GetEventCount(void);
MotionEvent_t* MotionLogger_GetEvent(uint16_t index);
void MotionLogger_Clear(void);

/* Defer EEPROM writes while the alarm is sounding — TIM16 PWM is unaffected
 * by main-loop stalls, but BUZZER_Update can't advance notes during a 15+ ms
 * EEPROM page commit, which the user hears as a stuck frequency. While
 * deferred, LogEvent updates only the RAM ring; FlushPending writes the
 * dirty slots + header out in a single batch (called after BUZZER_Stop). */
void MotionLogger_SetDeferEEPROM(uint8_t enable);
void MotionLogger_FlushPending(void);

void MotionLogger_SetBootTime(uint8_t year, uint8_t month, uint8_t day,
                               uint8_t hour, uint8_t minute, uint8_t second);

/* Force-flush the in-RAM boot-time anchor to EEPROM. SetBootTime already
 * does this implicitly when iOS syncs; state_machine.c calls it again on
 * entry to STATE_LOCKED as a defensive checkpoint. No-op if the anchor
 * isn't valid. The persisted bytes are no longer consumed at boot — see
 * the comment on EEPROM_BOOT_TIME_ADDR. Kept only as a forensic record. */
void MotionLogger_PersistAnchor(void);

/* Decompose a stored seconds-since-2000 timestamp to YY/MM/DD HH:MM:SS.
 * `epoch_seconds_2000` is the value previously stored by LogEvent (or 0 if
 * the event was logged with no anchor available). Output is the unknown-
 * time sentinel (0,1,1,0,0,0) when the input is 0. */
void MotionLogger_EpochSecondsToDateTime(uint32_t epoch_seconds_2000,
                                          uint8_t *year, uint8_t *month, uint8_t *day,
                                          uint8_t *hour, uint8_t *minute, uint8_t *second);

#endif // MOTION_LOGGER_H
