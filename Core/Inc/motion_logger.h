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

void MotionLogger_SetBootTime(uint8_t year, uint8_t month, uint8_t day,
                               uint8_t hour, uint8_t minute, uint8_t second);
void MotionLogger_TickToDateTime(uint32_t tick_ms, uint8_t* year, uint8_t* month,
                                   uint8_t* day, uint8_t* hour, uint8_t* minute, uint8_t* second);

#endif // MOTION_LOGGER_H
