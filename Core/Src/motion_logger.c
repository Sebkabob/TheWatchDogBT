#include "motion_logger.h"
#include <stdio.h>

/* Private variables */
static MotionEvent_t motionEvents[MAX_MOTION_EVENTS];
static uint16_t eventCount = 0;
static uint16_t nextIndex = 0;

/* Boot time tracking - set when iOS sends timestamp */
static struct {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t boot_tick_ms;  // HAL_GetTick() when boot time was set
    uint8_t valid;
} boot_time = {0};

/**
 * @brief Initialize the motion logger
 */
void MotionLogger_Init(void)
{
    MotionLogger_Clear();
    boot_time.valid = 0;  // Boot time not yet set
}

/**
 * @brief Set the boot time from iOS timestamp
 * @param year Year offset from 2000
 * @param month Month (1-12)
 * @param day Day (1-31)
 * @param hour Hour (0-23)
 * @param minute Minute (0-59)
 * @param second Second (0-59)
 */
void MotionLogger_SetBootTime(uint8_t year, uint8_t month, uint8_t day,
                               uint8_t hour, uint8_t minute, uint8_t second)
{
    boot_time.year = year;
    boot_time.month = month;
    boot_time.day = day;
    boot_time.hour = hour;
    boot_time.minute = minute;
    boot_time.second = second;
    boot_time.boot_tick_ms = HAL_GetTick();
    boot_time.valid = 1;
}

/**
 * @brief Convert tick timestamp to real date/time
 * @param tick_ms HAL_GetTick() timestamp
 * @param year Output: year offset from 2000
 * @param month Output: month (1-12)
 * @param day Output: day (1-31)
 * @param hour Output: hour (0-23)
 * @param minute Output: minute (0-59)
 * @param second Output: second (0-59)
 */
void MotionLogger_TickToDateTime(uint32_t tick_ms, uint8_t* year, uint8_t* month,
                                   uint8_t* day, uint8_t* hour, uint8_t* minute, uint8_t* second)
{
    if (!boot_time.valid) {
        // Boot time not set yet - return epoch time
        *year = 0;
        *month = 1;
        *day = 1;
        *hour = 0;
        *minute = 0;
        *second = 0;
        return;
    }

    // Calculate elapsed time since boot
    uint32_t elapsed_ms = tick_ms - boot_time.boot_tick_ms;
    uint32_t elapsed_seconds = elapsed_ms / 1000;

    // Start with boot time
    uint32_t total_seconds = boot_time.second +
                             boot_time.minute * 60 +
                             boot_time.hour * 3600 +
                             elapsed_seconds;

    // Calculate new time
    uint8_t new_second = total_seconds % 60;
    uint8_t new_minute = (total_seconds / 60) % 60;
    uint8_t new_hour = (total_seconds / 3600) % 24;
    uint32_t elapsed_days = total_seconds / 86400;  // Days elapsed

    // Simple date calculation (doesn't handle month/year overflow perfectly, but good enough)
    uint8_t new_day = boot_time.day + elapsed_days;
    uint8_t new_month = boot_time.month;
    uint8_t new_year = boot_time.year;

    // Handle day overflow (simplified - assumes 30 days per month for simplicity)
    // For a production system you'd want proper calendar math
    while (new_day > 30) {
        new_day -= 30;
        new_month++;
        if (new_month > 12) {
            new_month = 1;
            new_year++;
        }
    }

    *year = new_year;
    *month = new_month;
    *day = new_day;
    *hour = new_hour;
    *minute = new_minute;
    *second = new_second;
}

/**
 * @brief Log a motion event with current timestamp
 * @param motionType Type of motion detected
 * @return 1 if successful, 0 if buffer full
 */
uint8_t MotionLogger_LogEvent(MotionType_t motionType)
{
    // Store current tick timestamp
    motionEvents[nextIndex].timestamp_ms = HAL_GetTick();
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
        motionEvents[i].timestamp_ms = 0;
    }
    eventCount = 0;
    nextIndex = 0;
}
