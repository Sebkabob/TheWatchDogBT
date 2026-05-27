/***************************************************************************
 * crash_forensics.h
 * created by Sebastian Forenza 2026
 *
 * Two related persistence blocks for post-mortem debugging:
 *
 *   1. Hard-fault snapshot — written ONCE by HardFault_Handler from the
 *      stacked exception frame (PC, LR, xPSR), then the handler resets the
 *      chip. The snapshot persists in EEPROM at EEPROM_FAULT_SNAPSHOT_ADDR
 *      and is surfaced via the diagnostic dump on the next boot.
 *
 *   2. Reset event ring — appended once per boot with (reset_cause,
 *      boot_count, uptime_of_previous_boot). 8-deep, oldest-overwritten.
 *      Lets us see a pattern of brownouts vs. software resets vs. faults
 *      over the recent past.
 *
 * Why this exists: without it, the only signal that something went wrong is
 * the device behaving oddly — the prior firmware had no fault handler at
 * all (CubeMX default = infinite while(1) in Default_Handler), so a real
 * hard fault froze the chip silently with no path back to running code.
 ***************************************************************************/

#ifndef INC_CRASH_FORENSICS_H_
#define INC_CRASH_FORENSICS_H_

#include <stdint.h>
#include <stdbool.h>
#include "eeprom_map.h"

/* Decoded hard-fault snapshot fields. Returned as a struct so the
 * diagnostic dump can render each field with its own TLV tag. valid==0
 * means no snapshot has ever been written (or it was cleared by iOS). */
typedef struct {
    uint8_t  valid;
    uint8_t  schema_version;
    uint32_t pc;
    uint32_t lr;
    uint32_t xpsr;
} FaultSnapshot_t;

/* CrashForensics_Init — call after BootCount_Init runs.
 *   Reads the reset cause + boot count + uptime-of-this-boot-so-far (which
 *   really represents the previous boot's uptime once the next reset
 *   happens — we capture it at boot time as "current uptime is whatever it
 *   was when we last reset"). For the FIRST boot after a reset, the prior
 *   boot's uptime isn't knowable without a more elaborate scheme, so we
 *   record 0 there. Subsequent boots see "the uptime of this boot when we
 *   last reset", which is what's diagnostically useful.
 *
 *   Practical implementation: at boot, we read whatever uptime value the
 *   PREVIOUS boot wrote to the persistent "last-known-uptime" field (a
 *   separate cell would be ideal but we're tight on bytes). For v1 we
 *   record uptime = 0 on every boot — a future revision can heartbeat the
 *   current uptime to EEPROM every N seconds for a more accurate picture.
 */
void CrashForensics_Init(void);

/* Returns the most-recent hard-fault snapshot. valid=0 means no fault has
 * been recorded since the EEPROM was wiped. */
FaultSnapshot_t CrashForensics_GetLastFault(void);

/* Reset-ring accessors for the diagnostic dump.
 *   GetResetEventCount: number of populated entries (saturating, 0..8).
 *   GetResetEvent: oldest=index 0, newest = (count-1). Returns false on
 *   out-of-range or if the EEPROM read fails. */
uint8_t CrashForensics_GetResetEventCount(void);
bool    CrashForensics_GetResetEvent(uint8_t index,
                                     uint8_t *out_cause,
                                     uint32_t *out_boot_count,
                                     uint32_t *out_uptime_secs);

/* Called from HardFault_Handler — see crash_forensics.c for the asm wrap. */
void CrashForensics_RecordFault(uint32_t pc, uint32_t lr, uint32_t xpsr);

#endif /* INC_CRASH_FORENSICS_H_ */
