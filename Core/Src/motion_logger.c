#include "motion_logger.h"
#include <stdio.h>

/* Private variables */
static MotionEvent_t motionEvents[MAX_MOTION_EVENTS];
static uint16_t eventCount = 0;
static uint16_t nextIndex = 0;
static RTC_HandleTypeDef *hrtc_handle = NULL;

/**
 * @brief Initialize the motion logger
 * @param hrtc Pointer to RTC handle
 */
void MotionLogger_Init(RTC_HandleTypeDef *hrtc)
{
    hrtc_handle = hrtc;
    MotionLogger_Clear();
}

/**
 * @brief Log a motion event with current timestamp
 * @param motionType Type of motion detected
 * @return 1 if successful, 0 if buffer full
 */
uint8_t MotionLogger_LogEvent(MotionType_t motionType)
{
    if (hrtc_handle == NULL) {
        return 0;  // Not initialized
    }

    // Get current time and date from RTC
    HAL_RTC_GetTime(hrtc_handle, &motionEvents[nextIndex].time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc_handle, &motionEvents[nextIndex].date, RTC_FORMAT_BIN);

    // Store motion type
    motionEvents[nextIndex].motionType = motionType;
    motionEvents[nextIndex].valid = 1;

    // Update counters
    nextIndex++;
    if (nextIndex >= MAX_MOTION_EVENTS) {
        nextIndex = 0;  // Wrap around (circular buffer)
    }

    if (eventCount < MAX_MOTION_EVENTS) {
        eventCount++;
    }

    return 1;
}

/**
 * @brief Get the number of logged events
 * @return Number of events in buffer
 */
uint16_t MotionLogger_GetEventCount(void)
{
    return eventCount;
}

/**
 * @brief Get a specific event by index
 * @param index Event index (0 = oldest if buffer is full)
 * @return Pointer to event, or NULL if invalid
 */
MotionEvent_t* MotionLogger_GetEvent(uint16_t index)
{
    if (index >= eventCount) {
        return NULL;
    }

    // Calculate actual index in circular buffer
    uint16_t actualIndex;
    if (eventCount < MAX_MOTION_EVENTS) {
        actualIndex = index;
    } else {
        // Buffer is full, calculate from nextIndex
        actualIndex = (nextIndex + index) % MAX_MOTION_EVENTS;
    }

    if (motionEvents[actualIndex].valid) {
        return &motionEvents[actualIndex];
    }

    return NULL;
}

/**
 * @brief Clear all logged events
 */
void MotionLogger_Clear(void)
{
    for (uint16_t i = 0; i < MAX_MOTION_EVENTS; i++) {
        motionEvents[i].valid = 0;
        motionEvents[i].motionType = MOTION_TYPE_NONE;
    }
    eventCount = 0;
    nextIndex = 0;
}
