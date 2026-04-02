#ifndef MOTION_LOGGER_H
#define MOTION_LOGGER_H

#include "main.h"
#include "stm32wb0x_hal.h"

/* Maximum number of events to store */
#define MAX_MOTION_EVENTS 100

/* Motion types — MLC/FSM/door classifications */
typedef enum {
    MOTION_TYPE_NONE       = 0,
    MOTION_TYPE_IN_MOTION  = 1,  /* MLC: general movement detected */
    MOTION_TYPE_SHAKEN     = 2,  /* MLC: device was shaken */
    MOTION_TYPE_IMPACT     = 3,  /* FSM: impact event */
    MOTION_TYPE_FREEFALL   = 4,  /* FSM: free-fall event */
    MOTION_TYPE_TILTED     = 5,  /* Legacy: orientation change (kept for compat) */
    MOTION_TYPE_DOOR_OPEN  = 6,  /* Door opening rotation detected */
    MOTION_TYPE_DOOR_CLOSE = 7,  /* Door closing rotation detected */
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
