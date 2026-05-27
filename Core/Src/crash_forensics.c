/***************************************************************************
 * crash_forensics.c
 * created by Sebastian Forenza 2026
 *
 * Hard-fault snapshot + reset event ring. See crash_forensics.h for the
 * design overview.
 *
 * Hard-fault path is best-effort: from inside the fault handler we can't
 * fully trust that I2C / SysTick / power rails are healthy, but writing
 * the snapshot to EEPROM is the only way to survive a POR following the
 * fault. We attempt the write, then NVIC_SystemReset regardless. If the
 * write fails, the next boot just won't surface a snapshot — better than
 * the prior behaviour (silent freeze in the default Default_Handler).
 ***************************************************************************/

#include "main.h"            /* I2C_HandleTypeDef + CMSIS NVIC_SystemReset */
#include "crash_forensics.h"
#include "power_management.h"
#include <string.h>

#define M24CXX_MODEL 0
#include "m24cxx.h"

extern I2C_HandleTypeDef hi2c1;

static M24CXX_HandleTypeDef s_fault_eeprom;

/***************************************************************************
 * snapshot_write — write the 16-byte fault snapshot at EEPROM_FAULT_SNAPSHOT_ADDR
 *   Caller is responsible for bringing the EEPROM rail up.
 ***************************************************************************/
static void snapshot_write(uint32_t pc, uint32_t lr, uint32_t xpsr)
{
    uint8_t buf[EEPROM_FAULT_SNAPSHOT_LEN] = {0};
    buf[0]  = EEPROM_FAULT_SNAPSHOT_MAGIC;
    buf[1]  = 1;   /* valid */
    buf[2]  = EEPROM_FAULT_SNAPSHOT_VERSION;
    buf[3]  = 0;   /* reserved */
    buf[4]  = (uint8_t)(pc       & 0xFF);
    buf[5]  = (uint8_t)((pc >> 8)  & 0xFF);
    buf[6]  = (uint8_t)((pc >> 16) & 0xFF);
    buf[7]  = (uint8_t)((pc >> 24) & 0xFF);
    buf[8]  = (uint8_t)(lr       & 0xFF);
    buf[9]  = (uint8_t)((lr >> 8)  & 0xFF);
    buf[10] = (uint8_t)((lr >> 16) & 0xFF);
    buf[11] = (uint8_t)((lr >> 24) & 0xFF);
    buf[12] = (uint8_t)(xpsr       & 0xFF);
    buf[13] = (uint8_t)((xpsr >> 8)  & 0xFF);
    buf[14] = (uint8_t)((xpsr >> 16) & 0xFF);
    buf[15] = (uint8_t)((xpsr >> 24) & 0xFF);

    (void)m24cxx_write(&s_fault_eeprom, EEPROM_FAULT_SNAPSHOT_ADDR, buf, sizeof(buf));
}

/***************************************************************************
 * CrashForensics_RecordFault — called from HardFault_Handler
 *   We're in handler mode with the original CPU state on the stack. SysTick
 *   may or may not be ticking depending on what we faulted on; the I2C HAL
 *   uses HAL_GetTick() for its timeout but the timeout is only relevant if
 *   the slave NAKs. In a healthy fault scenario the write completes well
 *   inside the timeout. We don't gate on success — losing forensics is OK,
 *   freezing the device is not.
 ***************************************************************************/
void CrashForensics_RecordFault(uint32_t pc, uint32_t lr, uint32_t xpsr)
{
    PowerMgmt_EEPROM_PowerOn();

    if (m24cxx_init(&s_fault_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
        snapshot_write(pc, lr, xpsr);
    }

    PowerMgmt_EEPROM_PowerOff();

    /* Reset. Anything else risks looping in handler mode. */
    NVIC_SystemReset();
}

/***************************************************************************
 * Reset event ring — header at EEPROM_RESET_RING_ADDR, entries at
 *   EEPROM_RESET_RING_ENTRIES_ADDR. Ring uses write_index + saturating
 *   count semantics.
 ***************************************************************************/

typedef struct {
    uint8_t magic;
    uint8_t version;
    uint8_t write_index;
    uint8_t count;
} RingHeader_t;

static M24CXX_HandleTypeDef s_ring_eeprom;
static FaultSnapshot_t      s_last_fault_cache = {0};
static RingHeader_t         s_ring_header_cache = {0};

static bool ring_read_header(RingHeader_t *out)
{
    uint8_t buf[EEPROM_RESET_RING_HEADER_LEN];
    if (m24cxx_read(&s_ring_eeprom, EEPROM_RESET_RING_ADDR, buf, sizeof(buf))
            != M24CXX_Ok) {
        return false;
    }
    out->magic       = buf[0];
    out->version     = buf[1];
    out->write_index = buf[2];
    out->count       = buf[3];
    return true;
}

static void ring_write_header(const RingHeader_t *in)
{
    uint8_t buf[EEPROM_RESET_RING_HEADER_LEN];
    buf[0] = in->magic;
    buf[1] = in->version;
    buf[2] = in->write_index;
    buf[3] = in->count;
    (void)m24cxx_write(&s_ring_eeprom, EEPROM_RESET_RING_ADDR, buf, sizeof(buf));
}

static void ring_append_entry(uint8_t cause, uint32_t boot_count, uint32_t uptime_secs)
{
    RingHeader_t hdr = {0};
    if (!ring_read_header(&hdr) || hdr.magic != EEPROM_RESET_RING_MAGIC) {
        /* Blank or stale — initialise. */
        hdr.magic       = EEPROM_RESET_RING_MAGIC;
        hdr.version     = EEPROM_RESET_RING_VERSION;
        hdr.write_index = 0;
        hdr.count       = 0;
    }

    uint8_t slot = hdr.write_index;
    uint32_t entry_addr = EEPROM_RESET_RING_ENTRIES_ADDR +
                          (uint32_t)slot * EEPROM_RESET_RING_ENTRY_LEN;

    uint8_t entry[EEPROM_RESET_RING_ENTRY_LEN];
    entry[0] = cause;
    entry[1] = (uint8_t)(boot_count       & 0xFF);
    entry[2] = (uint8_t)((boot_count >> 8)  & 0xFF);
    entry[3] = (uint8_t)((boot_count >> 16) & 0xFF);
    entry[4] = (uint8_t)((boot_count >> 24) & 0xFF);
    entry[5] = (uint8_t)(uptime_secs       & 0xFF);
    entry[6] = (uint8_t)((uptime_secs >> 8)  & 0xFF);
    entry[7] = (uint8_t)((uptime_secs >> 16) & 0xFF);
    entry[8] = (uint8_t)((uptime_secs >> 24) & 0xFF);
    (void)m24cxx_write(&s_ring_eeprom, entry_addr, entry, sizeof(entry));

    hdr.write_index = (uint8_t)((slot + 1) % EEPROM_RESET_RING_ENTRIES);
    if (hdr.count < EEPROM_RESET_RING_ENTRIES) {
        hdr.count++;
    }
    ring_write_header(&hdr);

    s_ring_header_cache = hdr;
}

static void load_last_fault_cache(void)
{
    s_last_fault_cache.valid = 0;

    uint8_t buf[EEPROM_FAULT_SNAPSHOT_LEN];
    if (m24cxx_read(&s_ring_eeprom, EEPROM_FAULT_SNAPSHOT_ADDR, buf, sizeof(buf))
            != M24CXX_Ok) {
        return;
    }
    if (buf[0] != EEPROM_FAULT_SNAPSHOT_MAGIC || buf[1] != 1) {
        return;
    }

    s_last_fault_cache.valid          = 1;
    s_last_fault_cache.schema_version = buf[2];
    s_last_fault_cache.pc   = ((uint32_t)buf[4])  | ((uint32_t)buf[5]  << 8)
                            | ((uint32_t)buf[6]  << 16) | ((uint32_t)buf[7]  << 24);
    s_last_fault_cache.lr   = ((uint32_t)buf[8])  | ((uint32_t)buf[9]  << 8)
                            | ((uint32_t)buf[10] << 16) | ((uint32_t)buf[11] << 24);
    s_last_fault_cache.xpsr = ((uint32_t)buf[12]) | ((uint32_t)buf[13] << 8)
                            | ((uint32_t)buf[14] << 16) | ((uint32_t)buf[15] << 24);
}

void CrashForensics_Init(void)
{
    PowerMgmt_EEPROM_PowerOn();

    if (m24cxx_init(&s_ring_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        PowerMgmt_EEPROM_PowerOff();
        return;
    }

    /* Snapshot first — fault path may have just written it. */
    load_last_fault_cache();

    /* Append the boot event. v1 records uptime=0 for the previous boot; a
     * future revision can heartbeat the running uptime to a separate cell
     * so this entry reflects when the prior boot died. */
    uint8_t  cause    = PowerMgmt_GetResetCause();
    uint32_t boot_ct  = PowerMgmt_GetBootCount();
    uint32_t prev_upt = 0;
    ring_append_entry(cause, boot_ct, prev_upt);

    PowerMgmt_EEPROM_PowerOff();
}

FaultSnapshot_t CrashForensics_GetLastFault(void)
{
    return s_last_fault_cache;
}

uint8_t CrashForensics_GetResetEventCount(void)
{
    return s_ring_header_cache.count;
}

bool CrashForensics_GetResetEvent(uint8_t index,
                                  uint8_t *out_cause,
                                  uint32_t *out_boot_count,
                                  uint32_t *out_uptime_secs)
{
    if (out_cause == NULL || out_boot_count == NULL || out_uptime_secs == NULL) {
        return false;
    }
    if (index >= s_ring_header_cache.count) {
        return false;
    }

    /* Oldest entry sits at (write_index - count) mod N. */
    uint8_t oldest = (uint8_t)((EEPROM_RESET_RING_ENTRIES + s_ring_header_cache.write_index
                                 - s_ring_header_cache.count)
                               % EEPROM_RESET_RING_ENTRIES);
    uint8_t slot   = (uint8_t)((oldest + index) % EEPROM_RESET_RING_ENTRIES);

    uint32_t addr = EEPROM_RESET_RING_ENTRIES_ADDR +
                    (uint32_t)slot * EEPROM_RESET_RING_ENTRY_LEN;
    uint8_t entry[EEPROM_RESET_RING_ENTRY_LEN];

    PowerMgmt_EEPROM_PowerOn();
    bool ok = false;
    if (m24cxx_init(&s_ring_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
        if (m24cxx_read(&s_ring_eeprom, addr, entry, sizeof(entry)) == M24CXX_Ok) {
            ok = true;
        }
    }
    PowerMgmt_EEPROM_PowerOff();

    if (!ok) return false;

    *out_cause      = entry[0];
    *out_boot_count = ((uint32_t)entry[1])       | ((uint32_t)entry[2] <<  8)
                    | ((uint32_t)entry[3] << 16) | ((uint32_t)entry[4] << 24);
    *out_uptime_secs= ((uint32_t)entry[5])       | ((uint32_t)entry[6] <<  8)
                    | ((uint32_t)entry[7] << 16) | ((uint32_t)entry[8] << 24);
    return true;
}
