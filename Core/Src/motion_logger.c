/***************************************************************************
 * motion_logger.c
 * created by Sebastian Forenza 2026
 *
 * Motion-event ring buffer. Events are stored in SRAM for fast access and
 * mirrored to the M24C08 EEPROM so they survive DEEPSTOP / power loss.
 * Boot time is set by an iOS write; TickToDateTime converts HAL ticks to
 * calendar time when the app pulls the log.
 ***************************************************************************/

#include "motion_logger.h"
#include "power_management.h"
#include <stdio.h>
#include <string.h>

#define M24CXX_MODEL 0
#include "m24cxx.h"

static MotionEvent_t motionEvents[MAX_MOTION_EVENTS];
static uint16_t eventCount = 0;
static uint16_t nextIndex = 0;

/* Deferred-EEPROM state — see MotionLogger_SetDeferEEPROM in the header.
 * s_dirty_first is the slot index of the oldest unflushed entry;
 * s_dirty_count is how many slots from there are unflushed. The ring wrap
 * is handled in FlushPending. */
static uint8_t  s_defer_eeprom = 0;
static uint16_t s_dirty_first  = 0;
static uint16_t s_dirty_count  = 0;

static M24CXX_HandleTypeDef eeprom;
extern I2C_HandleTypeDef hi2c1;

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
 * EEPROM_WriteBootTime / EEPROM_LoadBootTime — persist the iOS-anchor
 *   No on-chip RTC survives power loss, so the only way for events logged
 *   on a prior boot to resolve to correct calendar times after a reset is
 *   to keep the (calendar, boot_tick_ms) pair on EEPROM. Caller must ensure
 *   EEPROM is powered and I2C is ready.
 ***************************************************************************/
static void EEPROM_WriteBootTime(void)
{
    if (!boot_time.valid) {
        return;
    }

    uint8_t buf[EEPROM_BOOT_TIME_LEN];
    buf[0]  = EEPROM_BOOT_TIME_MAGIC;
    buf[1]  = boot_time.year;
    buf[2]  = boot_time.month;
    buf[3]  = boot_time.day;
    buf[4]  = boot_time.hour;
    buf[5]  = boot_time.minute;
    buf[6]  = boot_time.second;
    buf[7]  = (uint8_t)(boot_time.boot_tick_ms       & 0xFF);
    buf[8]  = (uint8_t)((boot_time.boot_tick_ms >> 8)  & 0xFF);
    buf[9]  = (uint8_t)((boot_time.boot_tick_ms >> 16) & 0xFF);
    buf[10] = (uint8_t)((boot_time.boot_tick_ms >> 24) & 0xFF);

    m24cxx_write(&eeprom, EEPROM_BOOT_TIME_ADDR, buf, sizeof(buf));
}

static uint8_t EEPROM_LoadBootTime(void)
{
    uint8_t buf[EEPROM_BOOT_TIME_LEN];
    if (m24cxx_read(&eeprom, EEPROM_BOOT_TIME_ADDR, buf, sizeof(buf)) != M24CXX_Ok) {
        return 0;
    }

    if (buf[0] != EEPROM_BOOT_TIME_MAGIC) {
        return 0;
    }

    boot_time.year   = buf[1];
    boot_time.month  = buf[2];
    boot_time.day    = buf[3];
    boot_time.hour   = buf[4];
    boot_time.minute = buf[5];
    boot_time.second = buf[6];
    boot_time.boot_tick_ms = ((uint32_t)buf[7])
                           | ((uint32_t)buf[8]  << 8)
                           | ((uint32_t)buf[9]  << 16)
                           | ((uint32_t)buf[10] << 24);
    boot_time.valid = 1;
    return 1;
}

/***************************************************************************
 * EEPROM_WriteMotionHeader — write magic + counters to the header slot
 *   Caller must ensure EEPROM is powered and I2C is ready.
 ***************************************************************************/
static void EEPROM_WriteMotionHeader(void)
{
    uint8_t hdr[EEPROM_MOTION_HEADER_SIZE];
    hdr[0] = EEPROM_MAGIC_BYTE;
    hdr[1] = 0;
    hdr[2] = (uint8_t)(eventCount & 0xFF);
    hdr[3] = (uint8_t)((eventCount >> 8) & 0xFF);
    hdr[4] = (uint8_t)(nextIndex & 0xFF);
    hdr[5] = (uint8_t)((nextIndex >> 8) & 0xFF);
    hdr[6] = 0;
    hdr[7] = 0;

    m24cxx_write(&eeprom, EEPROM_MOTION_HEADER_ADDR, hdr, sizeof(hdr));
}

/***************************************************************************
 * EEPROM_WriteEvent — persist a single event into slot <slot>
 ***************************************************************************/
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

/***************************************************************************
 * EEPROM_LoadMotionLog — reload the persisted log into SRAM at boot
 *   Returns 1 on a valid load, 0 if the EEPROM was empty/invalid.
 ***************************************************************************/
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

/***************************************************************************
 * EEPROM_EraseMotionData — invalidate header and 0xFF-fill the event area
 *   Erases in page-sized chunks; a full-region VLA inside m24cxx_erase
 *   would blow the M0+ stack.
 ***************************************************************************/
static void EEPROM_EraseMotionData(void)
{
    uint8_t zeros[EEPROM_MOTION_HEADER_SIZE];
    memset(zeros, 0x00, sizeof(zeros));
    m24cxx_write(&eeprom, EEPROM_MOTION_HEADER_ADDR, zeros, sizeof(zeros));

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
 * MotionLogger_Init — clear SRAM ring buffer, then restore EEPROM contents
 ***************************************************************************/
void MotionLogger_Init(void)
{
    for (uint16_t i = 0; i < MAX_MOTION_EVENTS; i++) {
        motionEvents[i].valid = 0;
        motionEvents[i].motionType = MOTION_TYPE_NONE;
        motionEvents[i].timestamp_ms = 0;
    }
    eventCount = 0;
    nextIndex = 0;
    boot_time.valid = 0;

    PowerMgmt_EEPROM_PowerOn();
    if (m24cxx_init(&eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
        EEPROM_LoadMotionLog();
        // Restore the iOS-sync anchor so events logged on a prior boot still
        // resolve to correct calendar times until iOS sends a fresh anchor.
        (void)EEPROM_LoadBootTime();
    }
    PowerMgmt_EEPROM_PowerOff();
}

/***************************************************************************
 * MotionLogger_SetBootTime — anchor calendar time to the current HAL tick
 ***************************************************************************/
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

    PowerMgmt_EEPROM_PowerOn();
    EEPROM_WriteBootTime();
    PowerMgmt_EEPROM_PowerOff();
}

void MotionLogger_PersistAnchor(void)
{
    if (!boot_time.valid) {
        return;
    }
    PowerMgmt_EEPROM_PowerOn();
    EEPROM_WriteBootTime();
    PowerMgmt_EEPROM_PowerOff();
}

// Gregorian leap-year rule: divisible by 4, except divisible by 100, except
// divisible by 400. Without this the code would invent Feb 29 in 2100.
static uint8_t is_leap_year(uint32_t year_full)
{
    if ((year_full % 4) != 0)   return 0;
    if ((year_full % 100) != 0) return 1;
    if ((year_full % 400) != 0) return 0;
    return 1;
}

/***************************************************************************
 * MotionLogger_TickToDateTime — convert a HAL tick to YY/MM/DD HH:MM:SS
 *   Year is reported as offset from 2000 (matches the iOS wire format).
 *   Returns 00-01-01 00:00:00 if no boot time has been set, or if iOS
 *   sent a malformed boot time (month outside 1..12, day outside 1..31).
 ***************************************************************************/
void MotionLogger_TickToDateTime(uint32_t tick_ms, uint8_t* year, uint8_t* month,
                                   uint8_t* day, uint8_t* hour, uint8_t* minute, uint8_t* second)
{
    if (!boot_time.valid ||
        boot_time.month < 1 || boot_time.month > 12 ||
        boot_time.day   < 1 || boot_time.day   > 31) {
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
                             boot_time.minute * 60u +
                             boot_time.hour * 3600u +
                             elapsed_seconds;

    uint8_t new_second = total_seconds % 60;
    uint8_t new_minute = (total_seconds / 60) % 60;
    uint8_t new_hour = (total_seconds / 3600) % 24;
    uint32_t elapsed_days = total_seconds / 86400;

    static const uint8_t days_in_month[12] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };

    uint32_t day_acc   = (uint32_t)boot_time.day + elapsed_days;
    uint8_t  new_month = boot_time.month;
    uint32_t year_full = 2000u + (uint32_t)boot_time.year;

    for (;;) {
        uint8_t dim = days_in_month[new_month - 1];
        if (new_month == 2 && is_leap_year(year_full)) {
            dim = 29;
        }
        if (day_acc <= dim) {
            break;
        }
        day_acc -= dim;
        new_month++;
        if (new_month > 12) {
            new_month = 1;
            year_full++;
        }
    }
    uint8_t new_day = (uint8_t)day_acc;

    *year   = (uint8_t)((year_full - 2000u) & 0xFFu);
    *month  = new_month;
    *day    = new_day;
    *hour   = new_hour;
    *minute = new_minute;
    *second = new_second;
}

/***************************************************************************
 * MotionLogger_LogEvent — append an event, persist to EEPROM, ring-wrap
 *   When s_defer_eeprom is set, the EEPROM write is skipped and the slot
 *   is added to the dirty range instead. FlushPending writes them all out
 *   in one batch (e.g., on exit from STATE_ALARM_ACTIVE).
 ***************************************************************************/
uint8_t MotionLogger_LogEvent(MotionType_t motionType)
{
    uint16_t slot = nextIndex;

    motionEvents[slot].timestamp_ms = HAL_GetTick();
    motionEvents[slot].motionType = motionType;
    motionEvents[slot].valid = 1;

    nextIndex++;
    if (nextIndex >= MAX_MOTION_EVENTS) {
        nextIndex = 0;
    }
    if (eventCount < MAX_MOTION_EVENTS) {
        eventCount++;
    }

    if (s_defer_eeprom) {
        if (s_dirty_count == 0) {
            s_dirty_first = slot;
            s_dirty_count = 1;
        } else if (s_dirty_count < MAX_MOTION_EVENTS) {
            s_dirty_count++;
        } else {
            // Dirty range already spans the whole ring. New writes overwrite
            // the oldest dirty slot in place, so advance the start instead of
            // growing past the ring size.
            s_dirty_first = (uint16_t)((s_dirty_first + 1) % MAX_MOTION_EVENTS);
        }
        return 1;
    }

    PowerMgmt_EEPROM_PowerOn();
    EEPROM_WriteEvent(slot, &motionEvents[slot]);
    EEPROM_WriteMotionHeader();
    PowerMgmt_EEPROM_PowerOff();

    return 1;
}

/***************************************************************************
 * MotionLogger_SetDeferEEPROM — toggle deferred-write mode
 *   Caller is responsible for pairing every enable=1 with a FlushPending +
 *   enable=0. StateMachine_ChangeState does this around ALARM_ACTIVE.
 ***************************************************************************/
void MotionLogger_SetDeferEEPROM(uint8_t enable)
{
    s_defer_eeprom = enable ? 1 : 0;
}

/***************************************************************************
 * MotionLogger_FlushPending — persist any deferred slots in one batch
 *   Powers the EEPROM once, writes each dirty slot, writes the header, then
 *   powers down. No-op if nothing is dirty.
 ***************************************************************************/
void MotionLogger_FlushPending(void)
{
    if (s_dirty_count == 0) {
        return;
    }

    PowerMgmt_EEPROM_PowerOn();
    for (uint16_t i = 0; i < s_dirty_count; i++) {
        uint16_t slot = (uint16_t)((s_dirty_first + i) % MAX_MOTION_EVENTS);
        EEPROM_WriteEvent(slot, &motionEvents[slot]);
    }
    EEPROM_WriteMotionHeader();
    PowerMgmt_EEPROM_PowerOff();

    s_dirty_count = 0;
}

uint16_t MotionLogger_GetEventCount(void)
{
    return eventCount;
}

/***************************************************************************
 * MotionLogger_GetEvent — fetch by index where 0 = oldest, eventCount-1 = newest
 ***************************************************************************/
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

/***************************************************************************
 * MotionLogger_Clear — wipe SRAM ring buffer and EEPROM motion section
 ***************************************************************************/
void MotionLogger_Clear(void)
{
    for (uint16_t i = 0; i < MAX_MOTION_EVENTS; i++) {
        motionEvents[i].valid = 0;
        motionEvents[i].motionType = MOTION_TYPE_NONE;
        motionEvents[i].timestamp_ms = 0;
    }
    eventCount = 0;
    nextIndex = 0;

    PowerMgmt_EEPROM_PowerOn();
    EEPROM_EraseMotionData();
    PowerMgmt_EEPROM_PowerOff();
}
