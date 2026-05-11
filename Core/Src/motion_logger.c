/***************************************************************************
 * motion_logger.c
 * created by Sebastian Forenza 2026
 *
 * Motion-event ring buffer. Events are stored in SRAM for fast access and
 * mirrored to the M24C08 EEPROM so they survive DEEPSTOP / power loss.
 *
 * Architecture note — calendar at log time, monotonic radio-timer clock:
 *   Prior versions stored HAL_GetTick() per event and converted to calendar
 *   at the moment iOS pulled the log, using whatever anchor was current
 *   then. Three failure modes ensued:
 *     1) iOS calls SetBootTime on every settings write, which moves the
 *        anchor forward — old events' ticks then resolve to garbage future
 *        dates because (tick_ms - new_boot_tick_ms) underflowed as uint32.
 *     2) After a reset, HAL_GetTick restarts at 0; any reloaded anchor
 *        had a boot_tick_ms from the previous boot, so every new event
 *        underflowed.
 *     3) SysTick (the source of HAL_GetTick) is suspended in DEEPSTOP, so
 *        elapsed-tick deltas missed every minute of sleep — events logged
 *        between iOS syncs all clustered at the anchor's calendar value
 *        plus a few ms of wake time, regardless of when they actually fired.
 *   The fixes are: (a) capture calendar at log time and freeze it in the
 *   slot, so anchor moves don't disturb existing events, and (b) source
 *   "now" from HAL_RADIO_TIMER_GetCurrentSysTime() (LSI-clocked, runs in
 *   DEEPSTOP_WITH_SLOW_CLOCK_ON) instead of HAL_GetTick. Slots logged
 *   before iOS first syncs get the unknown-time sentinel (0 secs).
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
    /* Monotonic seconds at the moment iOS sync'd the anchor. Sourced from
     * HAL_RADIO_TIMER_GetCurrentSysTime() — NOT HAL_GetTick(). SysTick
     * stops in DEEPSTOP, so HAL_GetTick deltas under-count by however
     * long the device slept. The radio-timer clock runs from LSI and
     * keeps counting in DEEPSTOP_WITH_SLOW_CLOCK_ON (we configure that
     * in Projects/Common/BLE/Interfaces/stm32_lpm_if.c::PWR_EnterOffMode),
     * so its delta correctly reflects wall-clock elapsed seconds. */
    uint32_t boot_monotonic_secs;
    uint8_t valid;
} boot_time = {0};

/***************************************************************************
 * MotionLogger_MonotonicSeconds — DEEPSTOP-safe seconds-since-boot
 *   Wraps the 64-bit radio-timer counter (409600 ticks/sec). uint32 result
 *   gives ~136 years; we cast down because that fits the boot_time field
 *   and matches the elapsed-seconds math elsewhere.
 ***************************************************************************/
static uint32_t MotionLogger_MonotonicSeconds(void)
{
    return (uint32_t)(HAL_RADIO_TIMER_GetCurrentSysTime() / 409600ULL);
}

/***************************************************************************
 * EEPROM_WriteBootTime / EEPROM_LoadBootTime — persist the iOS-anchor
 *   Persisted for forensic inspection only; MotionLogger_Init no longer
 *   reloads at boot (see header). The on-wire layout's last four bytes
 *   still hold a uint32 LE; the semantics changed from "HAL tick ms" to
 *   "monotonic radio-timer seconds." Magic byte unchanged because the
 *   field is no longer consumed.
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
    buf[7]  = (uint8_t)(boot_time.boot_monotonic_secs       & 0xFF);
    buf[8]  = (uint8_t)((boot_time.boot_monotonic_secs >> 8)  & 0xFF);
    buf[9]  = (uint8_t)((boot_time.boot_monotonic_secs >> 16) & 0xFF);
    buf[10] = (uint8_t)((boot_time.boot_monotonic_secs >> 24) & 0xFF);

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
    boot_time.boot_monotonic_secs = ((uint32_t)buf[7])
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
 *   On-wire layout is unchanged from the prior firmware version (4 bytes
 *   little-endian uint32 + 1 byte motionType). What changed is the meaning
 *   of the uint32: previously HAL tick ms, now seconds-since-2000.
 ***************************************************************************/
static void EEPROM_WriteEvent(uint16_t slot, MotionEvent_t *event)
{
    uint8_t buf[EEPROM_MOTION_EVENT_SIZE];
    uint32_t ts = event->epoch_seconds_2000;
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
 *   On a mid-loop I2C failure, any slots already populated in SRAM are
 *   reset to valid=0 so they don't resurface as ghost events once
 *   eventCount climbs past them on later LogEvent calls.
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
            // Roll back: clear any slot we already populated this load so
            // a later LogEvent doesn't see (valid=1, stale-content) ghosts.
            for (uint16_t j = 0; j < i; j++) {
                motionEvents[j].valid = 0;
                motionEvents[j].motionType = MOTION_TYPE_NONE;
                motionEvents[j].epoch_seconds_2000 = 0;
            }
            return 0;
        }

        motionEvents[i].epoch_seconds_2000 = (uint32_t)buf[0]
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
 *   The persisted iOS-sync anchor is NOT consumed at boot. With timestamps
 *   now stored per-event as seconds-since-2000 (frozen at log time), the
 *   anchor only needs to be valid for NEW events going forward. Reloading
 *   an old anchor would also re-introduce the cross-reset uint32 underflow
 *   that bricked the previous design. iOS pushes a fresh anchor inside
 *   sendSettings() right after VERIFY_OK.
 ***************************************************************************/
void MotionLogger_Init(void)
{
    for (uint16_t i = 0; i < MAX_MOTION_EVENTS; i++) {
        motionEvents[i].valid = 0;
        motionEvents[i].motionType = MOTION_TYPE_NONE;
        motionEvents[i].epoch_seconds_2000 = 0;
    }
    eventCount = 0;
    nextIndex = 0;
    boot_time.valid = 0;

    PowerMgmt_EEPROM_PowerOn();
    if (m24cxx_init(&eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
        EEPROM_LoadMotionLog();
        // Deliberately not calling EEPROM_LoadBootTime() — see header.
    }
    PowerMgmt_EEPROM_PowerOff();
}

/***************************************************************************
 * MotionLogger_SetBootTime — anchor calendar to the monotonic radio-timer
 *   The monotonic reference uses HAL_RADIO_TIMER_GetCurrentSysTime (not
 *   HAL_GetTick), because SysTick stops in DEEPSTOP and would under-count
 *   the elapsed time between iOS sync and any event logged after the
 *   device next slept.
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
    boot_time.boot_monotonic_secs = MotionLogger_MonotonicSeconds();
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

static const uint16_t DAYS_BEFORE_MONTH[12] = {
    0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
};
static const uint8_t DAYS_IN_MONTH[12] = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

/***************************************************************************
 * calendar_to_epoch_seconds_2000 — convert YY/MM/DD HH:MM:SS to a uint32
 *   `year` is offset from 2000 (matches the iOS wire format). Returns 0 if
 *   any component is out of range — note this collides with the unknown-
 *   time sentinel, which is the intended behaviour (the slot is unanchored).
 *   The leap-day prefix is iterated rather than computed by closed-form
 *   because the iterator is trivially correct across the 2100/2400 century
 *   rule and the maximum 256 iterations is negligible at log time.
 ***************************************************************************/
static uint32_t calendar_to_epoch_seconds_2000(uint8_t year, uint8_t month, uint8_t day,
                                                uint8_t hour, uint8_t minute, uint8_t second)
{
    if (month < 1 || month > 12) return 0;
    if (day   < 1 || day   > 31) return 0;
    if (hour > 23 || minute > 59 || second > 59) return 0;

    uint32_t leap_days_before = 0;
    for (uint32_t y = 0; y < (uint32_t)year; y++) {
        if (is_leap_year(2000u + y)) leap_days_before++;
    }

    uint32_t year_full = 2000u + (uint32_t)year;
    uint8_t  is_leap   = is_leap_year(year_full);

    uint32_t days_in_prior_years = (uint32_t)year * 365u + leap_days_before;
    uint32_t day_of_year         = (uint32_t)DAYS_BEFORE_MONTH[month - 1] + (uint32_t)(day - 1);
    if (is_leap && month > 2) day_of_year++;

    uint32_t total_days = days_in_prior_years + day_of_year;
    return total_days * 86400u
         + (uint32_t)hour   * 3600u
         + (uint32_t)minute * 60u
         + (uint32_t)second;
}

/***************************************************************************
 * MotionLogger_EpochSecondsToDateTime — decompose stored timestamp
 *   Reverse of calendar_to_epoch_seconds_2000. Sentinel input (0) maps to
 *   the (0,1,1,0,0,0) "unknown" tuple that iOS recognises.
 ***************************************************************************/
void MotionLogger_EpochSecondsToDateTime(uint32_t epoch_seconds_2000,
                                          uint8_t *year, uint8_t *month, uint8_t *day,
                                          uint8_t *hour, uint8_t *minute, uint8_t *second)
{
    if (epoch_seconds_2000 == 0) {
        *year = 0; *month = 1; *day = 1;
        *hour = 0; *minute = 0; *second = 0;
        return;
    }

    uint32_t total_days  = epoch_seconds_2000 / 86400u;
    uint32_t time_of_day = epoch_seconds_2000 % 86400u;

    *hour   = (uint8_t)(time_of_day / 3600u);
    *minute = (uint8_t)((time_of_day / 60u) % 60u);
    *second = (uint8_t)(time_of_day % 60u);

    uint32_t year_offset = 0;
    while (year_offset < 255u) {
        uint32_t days_in_year = is_leap_year(2000u + year_offset) ? 366u : 365u;
        if (total_days < days_in_year) break;
        total_days -= days_in_year;
        year_offset++;
    }
    *year = (uint8_t)year_offset;

    uint8_t is_leap = is_leap_year(2000u + year_offset);
    uint8_t mon = 1;
    while (mon <= 12) {
        uint8_t dim = DAYS_IN_MONTH[mon - 1];
        if (mon == 2 && is_leap) dim = 29;
        if (total_days < dim) break;
        total_days -= dim;
        mon++;
    }
    *month = mon;
    *day   = (uint8_t)(total_days + 1u);
}

/***************************************************************************
 * MotionLogger_NowSeconds2000 — current calendar as seconds-since-2000
 *   Returns 0 (the unknown-time sentinel) when:
 *     - no anchor has been set this session, OR
 *     - the monotonic counter dipped below boot_monotonic_secs (which
 *       should be impossible — radio timer is monotonic — but we belt-
 *       and-brace anyway).
 *   The monotonic reference survives DEEPSTOP (LSI keeps the radio
 *   timer's wakeup block ticking in DEEPSTOP_WITH_SLOW_CLOCK_ON), so
 *   events logged after a long sleep get a correct calendar value.
 ***************************************************************************/
static uint32_t MotionLogger_NowSeconds2000(void)
{
    if (!boot_time.valid) return 0;

    uint32_t now = MotionLogger_MonotonicSeconds();
    if (now < boot_time.boot_monotonic_secs) return 0;

    uint32_t boot_secs = calendar_to_epoch_seconds_2000(
        boot_time.year, boot_time.month, boot_time.day,
        boot_time.hour, boot_time.minute, boot_time.second);
    if (boot_secs == 0) return 0;   // anchor calendar was malformed

    uint32_t elapsed_secs = now - boot_time.boot_monotonic_secs;
    return boot_secs + elapsed_secs;
}

/***************************************************************************
 * MotionLogger_LogEvent — append an event, persist to EEPROM, ring-wrap
 *   When s_defer_eeprom is set, the EEPROM write is skipped and the slot
 *   is added to the dirty range instead. FlushPending writes them all out
 *   in one batch (e.g., on exit from STATE_ALARM_ACTIVE).
 *
 *   Calendar is captured AT LOG TIME and stored as seconds-since-2000.
 *   This is the architectural piece that fixes "every settings write
 *   shifts the displayed timestamps of previously-logged events" — once
 *   the value lands in the slot, nothing downstream can change it.
 ***************************************************************************/
uint8_t MotionLogger_LogEvent(MotionType_t motionType)
{
    uint16_t slot = nextIndex;

    motionEvents[slot].epoch_seconds_2000 = MotionLogger_NowSeconds2000();
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
 *   Also resets the deferred-EEPROM dirty range. A mid-alarm CMD_CLEAR_LOG
 *   from iOS would otherwise leave stale s_dirty_first/s_dirty_count, and
 *   the next FlushPending would write zero'd RAM over wrong EEPROM slots.
 ***************************************************************************/
void MotionLogger_Clear(void)
{
    for (uint16_t i = 0; i < MAX_MOTION_EVENTS; i++) {
        motionEvents[i].valid = 0;
        motionEvents[i].motionType = MOTION_TYPE_NONE;
        motionEvents[i].epoch_seconds_2000 = 0;
    }
    eventCount = 0;
    nextIndex = 0;
    s_dirty_first = 0;
    s_dirty_count = 0;

    PowerMgmt_EEPROM_PowerOn();
    EEPROM_EraseMotionData();
    PowerMgmt_EEPROM_PowerOff();
}
