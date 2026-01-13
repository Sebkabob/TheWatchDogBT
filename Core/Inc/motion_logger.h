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

/* Motion event structure */
typedef struct {
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    MotionType_t motionType;
    uint8_t valid;  // 1 if entry is valid, 0 if empty
} MotionEvent_t;

/* Function prototypes */
void MotionLogger_Init(RTC_HandleTypeDef *hrtc);
uint8_t MotionLogger_LogEvent(MotionType_t motionType);
uint16_t MotionLogger_GetEventCount(void);
MotionEvent_t* MotionLogger_GetEvent(uint16_t index);
void MotionLogger_Clear(void);

#endif // MOTION_LOGGER_H
