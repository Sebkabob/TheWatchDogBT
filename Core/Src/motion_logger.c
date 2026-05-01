/***************************************************************************
 * motion_logger.c
 * created by Sebastian Forenza 2026
 *
 * Functions in charge of handling
 * motion logging storage and sending.
 * Events are stored in SRAM for fast access and mirrored to EEPROM
 * so they survive DEEPSTOP / power loss.
 ***************************************************************************/

#include "motion_logger.h"
#include "power_management.h"
#include <stdio.h>
#include <string.h>

/* M24C08 EEPROM driver */
#define M24CXX_MODEL 0
#include "m24cxx.h"

/* Private variables */
static MotionEvent_t motionEvents[MAX_MOTION_EVENTS];
static uint16_t eventCount = 0;
static uint16_t nextIndex = 0;

/* EEPROM handle */
static M24CXX_HandleTypeDef eeprom;
extern I2C_HandleTypeDef hi2c1;

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

/***************************************************************************
 * EEPROM HELPERS
 ***************************************************************************/

/**
 * @brief  Write the motion log header to EEPROM.
 *         Caller must ensure EEPROM is powered and I2C is ready.
 */
static void EEPROM_WriteMotionHeader(void)
{
    uint8_t hdr[EEPROM_MOTION_HEADER_SIZE];
    hdr[0] = EEPROM_MAGIC_BYTE;
    hdr[1] = 0;  // reserved
    hdr[2] = (uint8_t)(eventCount & 0xFF);
    hdr[3] = (uint8_t)((eventCount >> 8) & 0xFF);
    hdr[4] = (uint8_t)(nextIndex & 0xFF);
    hdr[5] = (uint8_t)((nextIndex >> 8) & 0xFF);
    hdr[6] = 0;  // reserved
    hdr[7] = 0;  // reserved

    m24cxx_write(&eeprom, EEPROM_MOTION_HEADER_ADDR, hdr, sizeof(hdr));
}

/**
 * @brief  Write a single motion event to EEPROM at the given slot index.
 *         Caller must ensure EEPROM is powered and I2C is ready.
 */
static void EEPROM_WriteEvent(uint16_t slot, MotionEvent_t *event)
{
    uint8_t buf[EEPROM_MOTION_EVENT_SIZE];
    uint32_t ts = event->timestamp_ms;
    buf[0] = (uint8_t)(ts & 0xFF);
    buf[1] = (uint8_t)((ts >> 8) & 0xFF);
    buf[2] = (uint8_t)((ts >> 16) & 0xFF);
    buf[3] = (uint8_t)((ts >> 24) & 0xFF);
    buf[4] = (uint8_t)event->motionType;

    uint32_t addr = EEPROM_MOTION_DATA_ADDR + (uint32_t)slot * EEPROM_MOTION_EVENT_SIZE;
    m24cxx_write(&eeprom, addr, buf, sizeof(buf));
}

/**
 * @brief  Load the full motion log from EEPROM into SRAM.
 *         Caller must ensure EEPROM is powered and I2C is ready.
 * @return 1 if valid data was loaded, 0 if EEPROM was empty/invalid.
 */
static uint8_t EEPROM_LoadMotionLog(void)
{
    uint8_t hdr[EEPROM_MOTION_HEADER_SIZE];
    if (m24cxx_read(&eeprom, EEPROM_MOTION_HEADER_ADDR, hdr, sizeof(hdr)) != M24CXX_Ok) {
        return 0;
    }

    if (hdr[0] != EEPROM_MAGIC_BYTE) {
        return 0;
    }

    uint16_t storedCount = (uint16_t)hdr[2] | ((uint16_t)hdr[3] << 8);
    uint16_t storedNext  = (uint16_t)hdr[4] | ((uint16_t)hdr[5] << 8);

    if (storedCount > MAX_MOTION_EVENTS || storedNext >= MAX_MOTION_EVENTS) {
        return 0;
    }

    /* Read each event from EEPROM into SRAM */
    uint16_t slotsToRead = (storedCount < MAX_MOTION_EVENTS) ? storedCount : MAX_MOTION_EVENTS;
    for (uint16_t i = 0; i < slotsToRead; i++) {
        uint8_t buf[EEPROM_MOTION_EVENT_SIZE];
        uint32_t addr = EEPROM_MOTION_DATA_ADDR + (uint32_t)i * EEPROM_MOTION_EVENT_SIZE;

        if (m24cxx_read(&eeprom, addr, buf, sizeof(buf)) != M24CXX_Ok) {
            return 0;
        }

        motionEvents[i].timestamp_ms = (uint32_t)buf[0]
                                     | ((uint32_t)buf[1] << 8)
                                     | ((uint32_t)buf[2] << 16)
                                     | ((uint32_t)buf[3] << 24);
        motionEvents[i].motionType = (MotionType_t)buf[4];
        motionEvents[i].valid = 1;
    }

    eventCount = storedCount;
    nextIndex  = storedNext;
    return 1;
}

/**
 * @brief  Erase the motion data section of EEPROM (header + events).
 *         Caller must ensure EEPROM is powered and I2C is ready.
 */
static void EEPROM_EraseMotionData(void)
{
    /* Zero-fill the header to invalidate it */
    uint8_t zeros[EEPROM_MOTION_HEADER_SIZE];
    memset(zeros, 0x00, sizeof(zeros));
    m24cxx_write(&eeprom, EEPROM_MOTION_HEADER_ADDR, zeros, sizeof(zeros));

    /* Erase event area in page-sized chunks (avoids large stack VLA
     * inside m24cxx_erase which would blow the M0+ stack) */
    uint8_t ff_buf[M24CXX_WRITE_PAGE_SIZE];
    memset(ff_buf, 0xFF, sizeof(ff_buf));

    uint32_t endAddr = EEPROM_MOTION_DATA_ADDR
                     + (uint32_t)MAX_MOTION_EVENTS * EEPROM_MOTION_EVENT_SIZE;

    for (uint32_t addr = EEPROM_MOTION_DATA_ADDR; addr < endAddr; ) {
        uint32_t chunk = endAddr - addr;
        if (chunk > sizeof(ff_buf)) chunk = sizeof(ff_buf);
        m24cxx_write(&eeprom, addr, ff_buf, chunk);
        addr += chunk;
    }
}

/***************************************************************************
 * PUBLIC API
 ***************************************************************************/

/**
 * @brief Initialize the motion logger — loads persisted events from EEPROM
 */
void MotionLogger_Init(void)
{
    /* Clear SRAM buffer first */
    for (uint16_t i = 0; i < MAX_MOTION_EVENTS; i++) {
        motionEvents[i].valid = 0;
        motionEvents[i].motionType = MOTION_TYPE_NONE;
        motionEvents[i].timestamp_ms = 0;
    }
    eventCount = 0;
    nextIndex = 0;
    boot_time.valid = 0;

    /* Power on EEPROM and try to restore saved events */
    PowerMgmt_EEPROM_PowerOn();
    if (m24cxx_init(&eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
        EEPROM_LoadMotionLog();
    }
    PowerMgmt_EEPROM_PowerOff();
}

/**
 * @brief Set the boot time from iOS timestamp
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
 */
void MotionLogger_TickToDateTime(uint32_t tick_ms, uint8_t* year, uint8_t* month,
                                   uint8_t* day, uint8_t* hour, uint8_t* minute, uint8_t* second)
{
    if (!boot_time.valid) {
        *year = 0;
        *month = 1;
        *day = 1;
        *hour = 0;
        *minute = 0;
        *second = 0;
        return;
    }

    uint32_t elapsed_ms = tick_ms - boot_time.boot_tick_ms;
    uint32_t elapsed_seconds = elapsed_ms / 1000;

    uint32_t total_seconds = boot_time.second +
                             boot_time.minute * 60 +
                             boot_time.hour * 3600 +
                             elapsed_seconds;

    uint8_t new_second = total_seconds % 60;
    uint8_t new_minute = (total_seconds / 60) % 60;
    uint8_t new_hour = (total_seconds / 3600) % 24;
    uint32_t elapsed_days = total_seconds / 86400;

    static const uint8_t days_in_month[12] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };

    uint32_t day_acc = (uint32_t)boot_time.day + elapsed_days;
    uint8_t  new_month = boot_time.month;
    uint8_t  new_year  = boot_time.year;

    for (;;) {
        uint8_t dim = days_in_month[new_month - 1];
        /* year is 2-digit (00..99); leap if divisible by 4, treat 00 as leap */
        if (new_month == 2 && (new_year % 4) == 0) {
            dim = 29;
        }
        if (day_acc <= dim) {
            break;
        }
        day_acc -= dim;
        new_month++;
        if (new_month > 12) {
            new_month = 1;
            new_year++;
        }
    }
    uint8_t new_day = (uint8_t)day_acc;

    *year = new_year;
    *month = new_month;
    *day = new_day;
    *hour = new_hour;
    *minute = new_minute;
    *second = new_second;
}

/**
 * @brief Log a motion event — writes to SRAM and persists to EEPROM
 * @param motionType Type of motion detected
 * @return 1 if successful, 0 on failure
 */
uint8_t MotionLogger_LogEvent(MotionType_t motionType)
{
    uint16_t slot = nextIndex;

    /* Write to SRAM */
    motionEvents[slot].timestamp_ms = HAL_GetTick();
    motionEvents[slot].motionType = motionType;
    motionEvents[slot].valid = 1;

    /* Advance ring buffer */
    nextIndex++;
    if (nextIndex >= MAX_MOTION_EVENTS) {
        nextIndex = 0;
    }
    if (eventCount < MAX_MOTION_EVENTS) {
        eventCount++;
    }

    /* Persist to EEPROM */
    PowerMgmt_EEPROM_PowerOn();
    EEPROM_WriteEvent(slot, &motionEvents[slot]);
    EEPROM_WriteMotionHeader();
    PowerMgmt_EEPROM_PowerOff();

    return 1;
}

/**
 * @brief Get the number of logged events
 */
uint16_t MotionLogger_GetEventCount(void)
{
    return eventCount;
}

/**
 * @brief Get a specific event by index (0 = oldest)
 */
MotionEvent_t* MotionLogger_GetEvent(uint16_t index)
{
    if (index >= eventCount) {
        return NULL;
    }

    uint16_t actualIndex;
    if (eventCount < MAX_MOTION_EVENTS) {
        actualIndex = index;
    } else {
        actualIndex = (nextIndex + index) % MAX_MOTION_EVENTS;
    }

    if (motionEvents[actualIndex].valid) {
        return &motionEvents[actualIndex];
    }

    return NULL;
}

/**
 * @brief Clear all logged events — erases SRAM buffer and EEPROM motion section
 */
void MotionLogger_Clear(void)
{
    /* Clear SRAM */
    for (uint16_t i = 0; i < MAX_MOTION_EVENTS; i++) {
        motionEvents[i].valid = 0;
        motionEvents[i].motionType = MOTION_TYPE_NONE;
        motionEvents[i].timestamp_ms = 0;
    }
    eventCount = 0;
    nextIndex = 0;

    /* Erase EEPROM motion section */
    PowerMgmt_EEPROM_PowerOn();
    EEPROM_EraseMotionData();
    PowerMgmt_EEPROM_PowerOff();
}
