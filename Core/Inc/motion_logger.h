/***************************************************************************
 * motion_logger.h
 * created by Sebastian Forenza 2026
 *
 * Motion-event ring buffer API. SRAM-backed for fast access; mirrored to
 * the M24C08 EEPROM so events survive DEEPSTOP and power loss.
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

/* Boot-time anchor (calendar + HAL_GetTick at iOS sync) persisted so events
 * already on EEPROM can still resolve to correct calendar times after a
 * reset. Lives inside the 0x000..0x03F reserved device-info block. Layout:
 *   [0]    magic 0xB7
 *   [1..6] year (offset from 2000) / month / day / hour / minute / second
 *   [7..10] boot_tick_ms (uint32, little-endian)
 * After a reset, OLD events with old-boot ticks resolve correctly against
 * this loaded anchor. NEW events logged before iOS resyncs will still mis-
 * resolve (HAL_GetTick restarts at 0 each boot) — they get an "unknown"
 * calendar until iOS sends a fresh anchor. */
#define EEPROM_BOOT_TIME_ADDR        0x00
#define EEPROM_BOOT_TIME_LEN         11
#define EEPROM_BOOT_TIME_MAGIC       0xB7

#define EEPROM_MOTION_HEADER_ADDR    0x040
#define EEPROM_MOTION_HEADER_SIZE    8

#define EEPROM_MOTION_DATA_ADDR      0x048
#define EEPROM_MOTION_EVENT_SIZE     5      // 4 timestamp + 1 type

#define EEPROM_MAGIC_BYTE            0xA5
#define EEPROM_I2C_ADDRESS           0x50   // M24C08 with A2=0

typedef enum {
    MOTION_TYPE_NONE       = 0,
    MOTION_TYPE_IN_MOTION  = 1,  // MLC: general movement detected
    MOTION_TYPE_SHAKEN     = 2,  // MLC: device was shaken
    MOTION_TYPE_IMPACT     = 3,  // FSM: impact event
    MOTION_TYPE_FREEFALL   = 4,  // FSM: free-fall event
} MotionType_t;

typedef struct {
    uint32_t timestamp_ms;       // HAL_GetTick() at event time
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
 * entry to STATE_LOCKED as a defensive checkpoint in case the device
 * resets during an alarm. No-op if the anchor isn't valid. */
void MotionLogger_PersistAnchor(void);
void MotionLogger_TickToDateTime(uint32_t tick_ms, uint8_t* year, uint8_t* month,
                                   uint8_t* day, uint8_t* hour, uint8_t* minute, uint8_t* second);

#endif // MOTION_LOGGER_H
