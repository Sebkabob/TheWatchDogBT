/***************************************************************************
 * eeprom_map.h
 * created by Sebastian Forenza 2026
 *
 * Single source of truth for every byte the firmware stores in the M24C08
 * external EEPROM. Every module that touches EEPROM MUST include this header
 * and use these symbols — no local re-#defines. Adding a new persistent
 * record means adding it here first.
 *
 * Why this exists: prior to this header the EEPROM addresses were scattered
 * across motion_logger.h, sound.h, lights.h, power_management.h, loyalty.h,
 * and inline #defines in state_machine.c. That arrangement produced three
 * silent address collisions in the device-info block (boot-time anchor vs
 * BD address, disconnect-sound flag vs device-settings record, boot count
 * vs device-settings deviceInfo byte). The collisions were invisible until
 * read-back of a stomped record produced the wrong value. Centralising the
 * map makes overlap visible at-a-glance and impossible to introduce by
 * accident.
 *
 * ============================================================================
 * Full map — M24C08, 1024 B total, 16 B write pages (0x000..0x3FF):
 * ============================================================================
 *
 *  --- Device-info block (fixed at 0x000..0x03F for historical compat) -----
 *  0x000..0x006  BD address record (7)
 *  0x007..0x00F  reserved
 *  0x010..0x015  Loyalty record (6)
 *  0x016..0x017  reserved
 *  0x018..0x019  Alarm duration (2)
 *  0x01A..0x01B  LED brightness (2)
 *  0x01C..0x01D  Alarm disabled (2)
 *  0x01E..0x01F  Disconnect-sound disabled (2)
 *  0x020..0x022  Device settings (3)             — magic bumped 0xC6 → 0xCA
 *  0x023         reserved
 *  0x024..0x025  BLE TX power (2)
 *  0x026..0x02F  reserved (10) — slack for future single-byte settings
 *  0x030..0x033  Boot count (4)                  — relocated from 0x020
 *  0x034..0x03E  Boot-time anchor (11)           — relocated from 0x000
 *                                                  magic bumped 0xB7 → 0xB8
 *  0x03F         reserved
 *
 *  --- Diagnostics & forensics ---------------------------------------------
 *  0x040..0x067  Lifetime counters (40)          [reserved, init only]
 *  0x068..0x06F  reserved (slack for more counters)
 *  0x070..0x07F  Manufacturing block (16)        [reserved, factory writer]
 *  0x080..0x097  Per-device tunables (24)        [reserved, init only]
 *  0x098..0x0AF  Custom nickname (24)            [reserved, init only]
 *  0x0B0..0x0C7  Fuel-gauge known-good snap (24) [reserved, deferred logic]
 *  0x0C8..0x0D7  Loyalty rotation metadata (16)  [reserved, init only]
 *  0x0D8..0x0E7  Hard-fault last snapshot (16)   [WRITTEN by HardFault_Handler]
 *  0x0E8..0x12F  Reset event ring (72)           [appended every boot]
 *  0x130..0x133  reserved
 *
 *  --- Generic key/value scratch region ------------------------------------
 *  0x134..0x1B3  Generic KV (128)                [iOS-owned, BLE r/w opcodes]
 *
 *  --- Motion log (consumes the remainder) ---------------------------------
 *  0x1B4..0x1BB  Motion ring header (8)
 *  0x1BC..0x3FF  Motion ring data (580) = 116 events x 5 bytes
 *
 * Endurance budget at typical use (M24C08 cell rated ~1M write cycles):
 *   boot count — 1 write per boot. 1M boots = ~273 years at 10 reboots/day.
 *   boot-time anchor — 1 write per iOS connect. 1M = ~137 years at 20/day.
 *   motion ring — wear-levelled across 116 slots, mostly idle.
 * No record is anywhere near wear-out under realistic operation.
 *
 ***************************************************************************/

#ifndef INC_EEPROM_MAP_H_
#define INC_EEPROM_MAP_H_

#include <stdint.h>

/* I2C address of the M24C08 (A2=0). */
#define EEPROM_I2C_ADDRESS                  0x50

/* ---- 0x000..0x006  BD address record ------------------------------------ */
#define EEPROM_BD_ADDR_ADDR                 0x000
#define EEPROM_BD_ADDR_LEN                  7      /* magic(1) + bdaddr(6) */
#define EEPROM_BD_ADDR_MAGIC                0xA7

/* Backwards-compat alias — app_ble.c historically used these names. */
#define EEPROM_DEVICE_INFO_ADDR             EEPROM_BD_ADDR_ADDR
#define EEPROM_MAGIC_BYTE                   EEPROM_BD_ADDR_MAGIC

/* ---- 0x010..0x015  Loyalty record --------------------------------------- */
#define EEPROM_LOYALTY_ADDR                 0x010
#define EEPROM_LOYALTY_LEN                  6      /* status(1) + token(4) + crc(1) */

/* ---- 0x018..0x019  Alarm duration --------------------------------------- */
#define EEPROM_ALARM_DURATION_ADDR          0x018
#define EEPROM_ALARM_DURATION_LEN           2
#define EEPROM_ALARM_DURATION_MAGIC         0xC3

/* ---- 0x01A..0x01B  LED brightness --------------------------------------- */
#define EEPROM_LED_BRIGHTNESS_ADDR          0x01A
#define EEPROM_LED_BRIGHTNESS_LEN           2
#define EEPROM_LED_BRIGHTNESS_MAGIC         0xC4

/* ---- 0x01C..0x01D  Alarm disabled --------------------------------------- */
#define EEPROM_ALARM_DISABLED_ADDR          0x01C
#define EEPROM_ALARM_DISABLED_LEN           2
#define EEPROM_ALARM_DISABLED_MAGIC         0xC5

/* ---- 0x01E..0x01F  Disconnect-sound disabled ---------------------------- */
#define EEPROM_DISCONNECT_SOUND_DISABLED_ADDR   0x01E
#define EEPROM_DISCONNECT_SOUND_DISABLED_LEN    2
#define EEPROM_DISCONNECT_SOUND_DISABLED_MAGIC  0xC7

/* ---- 0x020..0x022  Device settings (3 bytes) ---------------------------- *
 * Magic bumped 0xC6 → 0xCA: the layout itself is unchanged but the address
 * moved out of the boot-count collision zone, and the ARMED bit is now
 * persisted (was previously stripped). Old records load as blank.            */
#define EEPROM_DEVICE_SETTINGS_ADDR         0x020
#define EEPROM_DEVICE_SETTINGS_LEN          3      /* magic + state + info */
#define EEPROM_DEVICE_SETTINGS_MAGIC        0xCA

/* ---- 0x024..0x025  BLE TX power ----------------------------------------- */
#define EEPROM_BLE_TX_POWER_ADDR            0x024
#define EEPROM_BLE_TX_POWER_LEN             2
#define EEPROM_BLE_TX_POWER_MAGIC           0xC9

/* ---- 0x030..0x033  Boot count (uint32 LE) ------------------------------- *
 * Zero on virgin EEPROM (0xFFFFFFFF sentinel → 0), incremented once per boot.
 * Survives reflash because EEPROM is external. The only way to reset is to
 * physically replace the EEPROM or use a factory-tool wipe.                  */
#define EEPROM_BOOT_COUNT_ADDR              0x030
#define EEPROM_BOOT_COUNT_LEN               4

/* ---- 0x034..0x03E  Boot-time anchor ------------------------------------- *
 * Magic + Y/M/D/H/M/S + monotonic uint32 LE. Forensic only — MotionLogger_Init
 * does not consume the persisted bytes at boot; iOS pushes a fresh anchor
 * inside sendSettings() right after loyalty verify.                          */
#define EEPROM_BOOT_TIME_ADDR               0x034
#define EEPROM_BOOT_TIME_LEN                11
#define EEPROM_BOOT_TIME_MAGIC              0xB8   /* bumped from 0xB7 */

/* ---- 0x040..0x067  Lifetime counters ------------------------------------ *
 * Reserved address + magic only in this scope — increment hooks across the
 * codebase land in a follow-up commit. Layout (planned):
 *   [0]    magic 0xD0
 *   [1]    schema version (currently 1)
 *   [2..5]    alarm_fires_count       (uint32 LE)
 *   [6..9]    lifetime_motion_count   (uint32 LE)
 *   [10..13]  ble_disconnect_count    (uint32 LE)
 *   [14..21]  ble_disconnect_reason_histogram (8 buckets x 1 B)
 *   [22..25]  i2c_error_count         (uint32 LE)
 *   [26..29]  eeprom_write_fail_count (uint32 LE)
 *   [30..37]  battery_init_fail_stages_ring (8 entries x 1 B)
 *   [38..39]  reserved
 *  (40 bytes; an extra 24 B of slack lives at 0x068..0x07F for future fields)
 */
#define EEPROM_LIFETIME_COUNTERS_ADDR       0x040
#define EEPROM_LIFETIME_COUNTERS_LEN        40
#define EEPROM_LIFETIME_COUNTERS_MAGIC      0xD0
#define EEPROM_LIFETIME_COUNTERS_VERSION    1

/* ---- 0x070..0x07F  Manufacturing block ---------------------------------- *
 * One-shot write at factory provisioning. Layout (planned):
 *   [0]    magic 0xD1
 *   [1]    schema version
 *   [2..9]    serial number (8 B)
 *   [10]      hardware revision (1 B)
 *   [11..12]  factory test pass/fail bits (16 flags)
 *   [13..15]  manufacturing date (Y/M/D, year offset from 2000)
 */
#define EEPROM_MANUFACTURING_ADDR           0x070
#define EEPROM_MANUFACTURING_LEN            16
#define EEPROM_MANUFACTURING_MAGIC          0xD1
#define EEPROM_MANUFACTURING_VERSION        1

/* ---- 0x080..0x097  Per-device tunables ---------------------------------- *
 * Holds values that today are hardcoded constants. Reserving the bytes now
 * means we can expose iOS-side knobs later without a firmware update.
 *   [0]    magic 0xD2
 *   [1]    schema version
 *   [2..3]    sig_motion_threshold (raw accel cut-off for MEDIUM sens)
 *   [4..5]    stabilize_timeout_ms
 *   [6..7]    motion_grace_ms
 *   [8..9]    cable_unplug_awake_ms
 *   [10..11]  lp_armed_pulse_period_ms
 *   [12]      lp_armed_pulse_intensity
 *   [13..14]  ble_adv_interval_min
 *   [15..16]  ble_adv_interval_max
 *   [17..18]  ble_conn_interval_min
 *   [19..20]  ble_conn_interval_max
 *   [21]      alarm_tone_selector
 *   [22..23]  reserved
 */
#define EEPROM_TUNABLES_ADDR                0x080
#define EEPROM_TUNABLES_LEN                 24
#define EEPROM_TUNABLES_MAGIC               0xD2
#define EEPROM_TUNABLES_VERSION             1

/* ---- 0x098..0x0AF  Custom nickname -------------------------------------- *
 * On-device user-supplied name (survives app reinstall). Layout:
 *   [0]    magic 0xD3
 *   [1]    length (0..20)
 *   [2..21]  UTF-8 bytes (not null-terminated)
 *   [22..23] reserved
 */
#define EEPROM_NICKNAME_ADDR                0x098
#define EEPROM_NICKNAME_LEN                 24
#define EEPROM_NICKNAME_MAGIC               0xD3
#define EEPROM_NICKNAME_MAX_BYTES           20

/* ---- 0x0B0..0x0C7  Fuel-gauge known-good snapshot ----------------------- *
 * Periodic checkpoint of learned BQ27427 values. Used to re-seed the gauge
 * if it loses its mind. Snapshot routine is deferred — reserve only.
 *   [0]    magic 0xD4
 *   [1]    schema version
 *   [2..5]    cc_gain  (float, 4 B)
 *   [6..9]    cap_gain (float, 4 B)
 *   [10..11]  qmax_cell0
 *   [12..13]  reserved_capacity
 *   [14..17]  snapshot_epoch_s2000
 *   [18..19]  crc16
 *   [20..23]  reserved
 */
#define EEPROM_FUEL_SNAPSHOT_ADDR           0x0B0
#define EEPROM_FUEL_SNAPSHOT_LEN            24
#define EEPROM_FUEL_SNAPSHOT_MAGIC          0xD4
#define EEPROM_FUEL_SNAPSHOT_VERSION        1

/* ---- 0x0C8..0x0D7  Loyalty rotation metadata ---------------------------- *
 *   [0]    magic 0xD5
 *   [1]    schema version
 *   [2..5]    last_claimed_epoch_s2000
 *   [6..7]    claim_counter (uint16 LE — increments on every successful CLAIM)
 *   [8..15]   prior_owner_fingerprint (first 8 bytes of SHA of prior token)
 */
#define EEPROM_LOYALTY_ROTATION_ADDR        0x0C8
#define EEPROM_LOYALTY_ROTATION_LEN         16
#define EEPROM_LOYALTY_ROTATION_MAGIC       0xD5
#define EEPROM_LOYALTY_ROTATION_VERSION     1

/* ---- 0x0D8..0x0E7  Hard-fault last snapshot ----------------------------- *
 * Written ONCE by HardFault_Handler, then the handler calls NVIC_SystemReset.
 * Layout:
 *   [0]    magic 0xD6
 *   [1]    valid flag (0 = uninit, 1 = present)
 *   [2]    schema version
 *   [3]    reserved
 *   [4..7]    PC at fault (from stacked frame)
 *   [8..11]   LR at fault
 *   [12..15]  xPSR at fault
 */
#define EEPROM_FAULT_SNAPSHOT_ADDR          0x0D8
#define EEPROM_FAULT_SNAPSHOT_LEN           16
#define EEPROM_FAULT_SNAPSHOT_MAGIC         0xD6
#define EEPROM_FAULT_SNAPSHOT_VERSION       1

/* ---- 0x0E8..0x12F  Reset event ring (72 B) ------------------------------ *
 * 8 most-recent reset events, append-on-boot.
 *   Header (4 B at 0x0E8):
 *     [0]  magic 0xD7
 *     [1]  schema version (1)
 *     [2]  write_index (0..7, next slot to overwrite)
 *     [3]  count (saturating, 0..8)
 *   Entries (8 x 9 B starting at 0x0EC):
 *     [0]  reset_cause_packed   (PAD/POR/SFT/WDG/LOCKUP, see PowerMgmt_GetResetCause)
 *     [1..4]  boot_count_at_event (uint32 LE)
 *     [5..8]  uptime_at_event_s   (uint32 LE — uptime of the PREVIOUS boot)
 */
#define EEPROM_RESET_RING_ADDR              0x0E8
#define EEPROM_RESET_RING_LEN               72
#define EEPROM_RESET_RING_HEADER_LEN        4
#define EEPROM_RESET_RING_MAGIC             0xD7
#define EEPROM_RESET_RING_VERSION           1
#define EEPROM_RESET_RING_ENTRIES           8
#define EEPROM_RESET_RING_ENTRY_LEN         9
#define EEPROM_RESET_RING_ENTRIES_ADDR      (EEPROM_RESET_RING_ADDR + EEPROM_RESET_RING_HEADER_LEN)

/* ---- 0x134..0x1B3  Generic KV scratch region (128 B) -------------------- *
 * iOS-owned bytes. Firmware exposes bounds-checked READ/WRITE BLE opcodes
 * (CMD_EE_READ 0xE5, CMD_EE_WRITE 0xE6) that only touch this region. The
 * layout INSIDE the region is iOS's problem — firmware is dumb storage.
 *
 * This is the meta-fix: post-ship, any new per-device app feature can claim
 * a few bytes here without a firmware update. The schema is iOS-side TLV.   */
#define EEPROM_KV_SCRATCH_ADDR              0x134
#define EEPROM_KV_SCRATCH_LEN               128

/* ---- 0x1B4..0x1BB  Motion ring header (8 B) ----------------------------- *
 * Relocated from 0x040 to make room for the diagnostics blocks above.
 * Layout unchanged; magic bumped to invalidate the old (relocated) header.   */
#define EEPROM_MOTION_HEADER_ADDR           0x1B4
#define EEPROM_MOTION_HEADER_SIZE           8

/* ---- 0x1BC..0x3FF  Motion ring data (580 B = 116 events) ---------------- */
#define EEPROM_MOTION_DATA_ADDR             0x1BC
#define EEPROM_MOTION_EVENT_SIZE            5      /* 4 epoch_s2000 + 1 type */
#define EEPROM_MOTION_DATA_LEN              (1024 - EEPROM_MOTION_DATA_ADDR)
#define EEPROM_MOTION_MAX_EVENTS            (EEPROM_MOTION_DATA_LEN / EEPROM_MOTION_EVENT_SIZE)
/* MAX_MOTION_EVENTS evaluates to 116. */

/* Bumped magic for the motion ring: old EEPROMs reload as blank because the
 * per-slot layout, capacity, AND address all changed in this commit.        */
#define EEPROM_MOTION_MAGIC                 0xA8   /* was 0xA7, then 0xA6 */

/* Convenience aliases used by older motion_logger.{c,h} call sites. Kept
 * for diff-minimising; new code should use the EEPROM_MOTION_* names.       */
#define MAX_MOTION_EVENTS                   EEPROM_MOTION_MAX_EVENTS

#endif /* INC_EEPROM_MAP_H_ */
