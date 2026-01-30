#ifndef MOTION_LOGGER_H
#define MOTION_LOGGER_H

#include "main.h"
#include "stm32wb0x_hal.h"

/* Maximum number of events to store */
#define MAX_MOTION_EVENTS 100

/* Motion types */
typedef enum {
    MOTION_TYPE_NONE = 0,
    MOTION_TYPE_SMALL = 1,
    MOTION_TYPE_MEDIUM = 2,
    MOTION_TYPE_LARGE = 3
} MotionType_t;

/* Motion event structure - now uses tick timestamps */
typedef struct {
    uint32_t timestamp_ms;  // HAL_GetTick() value when event occurred
    MotionType_t motionType;
    uint8_t valid;  // 1 if entry is valid, 0 if empty
} MotionEvent_t;

/* Function prototypes */
void MotionLogger_Init(void);
uint8_t MotionLogger_LogEvent(MotionType_t motionType);
uint16_t MotionLogger_GetEventCount(void);
MotionEvent_t* MotionLogger_GetEvent(uint16_t index);
void MotionLogger_Clear(void);

/* Boot time management */
void MotionLogger_SetBootTime(uint8_t year, uint8_t month, uint8_t day,
                               uint8_t hour, uint8_t minute, uint8_t second);
void MotionLogger_TickToDateTime(uint32_t tick_ms, uint8_t* year, uint8_t* month,
                                   uint8_t* day, uint8_t* hour, uint8_t* minute, uint8_t* second);

#endif // MOTION_LOGGER_H
