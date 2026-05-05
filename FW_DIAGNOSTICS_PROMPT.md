# WatchDogBT — Diagnostics (iOS implementation prompt)

## Goal

The firmware can no longer be field-updated reliably. The app is the only window
left into what's happening on-device. This document defines the on-demand
**Diagnostic** transfer that replaces the old auto-pushed gauge dump.

## What changes on the iOS side

1. **Remove the "Gauge Health" button.** The data it surfaced now lives inside
   the new Diagnostic view as the **Battery** section. There is no separate
   gauge-health code path.
2. **Repurpose the existing "Diagnostic" button** to:
   - send `CMD_REQUEST_DIAG` (0xF4) over `APPTOWD`,
   - wait for one `BATTERYDIAG`-channel notification (the characteristic name is
     unchanged for compatibility — payload format is new),
   - parse the TLV payload into sections,
   - render each section in its own collapsible/scrollable card: **System**,
     **Battery**, **BLE Link**, **Sensor**, **Power**, **Storage**.
3. The firmware **no longer pushes** diagnostic notifications automatically.
   Battery state used to update once per second; now you only see it when you
   ask. Trigger a fresh request when:
   - the user opens the Diagnostic view, and
   - the user taps a "Refresh" affordance on that view.
   Do not poll faster than once per second (firmware caches battery state
   internally on a 1 s tick — sending faster gives you the same numbers).

## Wire protocol

### Request

`APPTOWD` write, after the 4-byte loyalty token:

```
[t0, t1, t2, t3, 0xF4, section_mask?]
```

| Byte | Meaning |
|------|---------|
| 0xF4 | `CMD_REQUEST_DIAG` opcode |
| `section_mask` (optional, default 0xFF) | Bitmask of sections to send; `0xFF` = all. Bit positions match the section IDs (bit 0 = SYSTEM, bit 1 = BATTERY, … bit 5 = STORAGE). For the foreseeable future just send `0xFF` and parse what comes back. |

No timestamp tail — this opcode is not a settings write.

### Response

One notification on the `BATTERYDIAG` characteristic. The payload is a TLV
(type-length-value) blob; **iOS must parse sections by ID and length, never by
fixed offset**, because new fields will appear inside section payloads over time
and unknown section IDs may also appear.

```
Header (2 bytes):
  byte 0:  format_version  (currently 1; bump = breaking change to header layout)
  byte 1:  section_count   (how many sections follow)

Sections (repeated, section_count times):
  byte 0:  section_id      (1..0xFE; 0xFF reserved for future end-marker)
  byte 1:  section_len     (N, number of payload bytes that follow)
  bytes 2..N+1: section payload (N bytes, layout per section_id below)
```

All multi-byte integers are **little-endian** unless noted.

### Section IDs

| ID | Name | Always present in v1 |
|----|------|----------------------|
| `0x01` | SYSTEM | yes |
| `0x02` | BATTERY | yes |
| `0x03` | BLE | yes |
| `0x04` | SENSOR | yes |
| `0x05` | POWER | yes |
| `0x06` | STORAGE | yes |
| `0x07..0xFE` | reserved for future | — |

Sections may grow: any section's `section_len` may increase in future firmware,
with new fields appended at the end. **iOS must read exactly `section_len`
bytes per section** (not a hardcoded constant) and ignore trailing bytes it
doesn't recognise.

---

## Section payload layouts (v1)

### `0x01` SYSTEM (19 bytes today)

| Off | Type | Field | Notes |
|-----|------|-------|-------|
| 0 | u32 LE | `uptime_seconds` | Seconds since the most recent boot. |
| 4 | u32 LE | `boot_count` | EEPROM-persisted. Incremented once per boot. Watch for unexpected jumps while the device is sitting idle. |
| 8 | u8 | `reset_cause` | Latched `RCC->CSR` reset flags packed into a single byte at boot, then `__HAL_RCC_CLEAR_RESET_FLAGS()` is called so the next boot's bits are clean. Bit decoding: bit 0 = PAD (NRSTn pin), bit 1 = POR/BOR (cold boot), bit 2 = SFT (software reset, e.g. `CMD_RESET_DEVICE`), bit 3 = WDG (watchdog), bit 4 = LOCKUP (CPU hard-locked). Multiple bits can be set per boot; treat WDG/LOCKUP as red flags. |
| 9 | u8 | `fw_version_major` | Same triplet as bytes 16..18 of DEVICESTATUS. |
| 10 | u8 | `fw_version_main` | |
| 11 | u8 | `fw_version_v2` | |
| 12 | u8 | `init_bitmask` | Each bit = "this subsystem's `*_Init()` reached its end successfully": bit 0 I2C, bit 1 BQ27427 (battery), bit 2 LIS2DUX12 (accel), bit 3 EEPROM, bit 4 Loyalty, bit 5 MotionLogger, bit 6 BLE. **Show any zero bit as a red flag** in the UI. |
| 13 | u8 | `last_fault_marker` | 0 = clean. Non-zero = the previous boot ended in a HardFault and this byte was written by the fault handler before reset. (Firmware will populate in a later revision; show "—" if 0.) |
| 14 | u8 × 6 | reserved | All zero in v1. Ignore. |

### `0x02` BATTERY (51 bytes today)

This is the **existing** v11 BatteryDiagnostic payload, unchanged byte-for-byte.
If you already have a parser for the old auto-pushed BATTERYDIAG dump, **reuse
it verbatim** on this section's payload bytes.

| Off | Type | Field |
|-----|------|-------|
| 0 | u8 | `version` (= 11) |
| 1 | u8 | `soc_percent` (filtered) |
| 2 | u16 LE | `voltage_mV` |
| 4 | i16 LE | `current_mA` (negative = discharging) |
| 6 | u16 LE | `remaining_mAh` |
| 8 | u16 LE | `full_charge_mAh` |
| 10 | i16 LE | `temperature_0_1K` (÷10 then −273.15 for °C) |
| 12 | u16 LE | `flags_raw` |
| 14 | u16 LE | `control_status_raw` |
| 16 | u8 | `status_bits` (bit0 charging, 1 full, 2 low, 3 critical, 4 bat_detected, 5 qmax_learned, 6 res_learned, 7 itpor) |
| 17 | u8 | `soc_unfiltered` |
| 18 | u16 LE | `design_capacity_mAh` |
| 20 | u16 LE | `terminate_voltage_mV` |
| 22 | u16 LE | `taper_rate` |
| 24 | u16 LE | `op_config_raw` |
| 26 | i16 LE | `average_power_mW` |
| 28 | i8 | `board_offset` |
| 29 | u8 | `deadband_mA` |
| 30 | u8 × 16 | `calib_bytes` (Subclass 104 dump) |
| 46 | u8 | `init_fail_stage` |
| 47 | u8 | `init_completed` |
| 48 | u8 | `post_reset_fired` |
| 49 | u16 LE | `chem_id_read` |

### `0x03` BLE (14 bytes today)

| Off | Type | Field | Notes |
|-----|------|-------|-------|
| 0 | i8 | `current_rssi_dBm` | `0x7F` if not measured this build. Live RSSI on the active link. |
| 1 | u16 LE | `connection_count_since_boot` | Number of GAP connect events since boot. |
| 3 | u8 | `last_disconnect_reason` | HCI error code from the previous disconnect (`0x13` = remote user terminated, `0x08` = supervision timeout, `0x22` = LL response timeout, `0x3E` = failed to establish, etc.). `0x00` = no prior disconnect. |
| 4 | u16 LE | `mtu_negotiated` | ATT MTU agreed for this connection. |
| 6 | u16 LE | `connection_interval_units` | LL conn interval in 1.25 ms units. `0` if not measured this build. |
| 8 | u8 × 6 | reserved | Zero. |

### `0x04` SENSOR (18 bytes today)

| Off | Type | Field | Notes |
|-----|------|-------|-------|
| 0 | u8 | `cached_mlc_state` | 0x00 = STATIONARY_UPRIGHT, 0x04 = STATIONARY_NOT_UPRIGHT, 0x08 = IN_MOTION, 0x0C = SHAKEN. Anything else → "unknown". |
| 1 | u8 | `last_fsm_event` | 0 = none, otherwise FSM event code (impact / freefall). |
| 2 | u32 LE | `mlc_transitions_since_boot` | Counter of MLC state-byte changes. If zero after motion, UCF didn't load. |
| 6 | u32 LE | `int1_fires_since_boot` | Counter of PB15 interrupt fires (or DEEPSTOP wake equivalents). |
| 10 | u32 LE | `motion_events_logged_since_boot` | How many entries the logger has appended since boot (independent of total log count, which lives in STORAGE). |
| 14 | u8 × 4 | reserved | Zero. |

### `0x05` POWER (24 bytes today)

| Off | Type | Field | Notes |
|-----|------|-------|-------|
| 0 | u32 LE | `wakes_motion` | Wake events attributed to PB15 (accel INT1). |
| 4 | u32 LE | `wakes_cable` | Wake events attributed to PB4 (cable plug). |
| 8 | u32 LE | `wakes_debug` | Wake events attributed to PB5 (debug-hold). |
| 12 | u32 LE | `wakes_tick` | Tick / radio-driven wakes (everything else). |
| 16 | u32 LE | `time_in_lp_seconds` | Cumulative seconds spent in either low-power mode. Compute duty cycle as `time_in_lp_seconds / uptime_seconds` and show as % "asleep". |
| 20 | u8 | `current_power_state` | 0 active, 1 LP_IDLE, 2 LP_ARMED. |
| 21 | u8 × 3 | reserved | Zero. |

### `0x06` STORAGE (26 bytes today)

| Off | Type | Field | Notes |
|-----|------|-------|-------|
| 0 | u16 LE | `motion_log_count` | Current ring-buffer fill. |
| 2 | u16 LE | `motion_log_max` | Buffer capacity (`MAX_MOTION_EVENTS`, currently 169). |
| 4 | u8 | `loyalty_store_healthy` | 1 healthy, 0 store flagged unhealthy at boot — **all loyalty operations refused while 0**. Surface this prominently. |
| 5 | u8 | `loyalty_claimed` | 1 claimed, 0 unclaimed. |
| 6 | u32 LE | `i2c_errors_since_boot` | HAL-level I2C error/timeout counter, all chips combined. |
| 10 | u32 LE | `eeprom_fail_count` | Failed EEPROM reads/writes since boot. |
| 14 | u32 LE | `bq27427_fail_count` | Failed BQ27427 reads/writes since boot. |
| 18 | u32 LE | `lis2dux12_fail_count` | Failed LIS2DUX12 reads/writes since boot. |
| 22 | u8 × 4 | reserved | Zero. |

---

## Parsing recipe (Swift sketch)

```swift
struct DiagPayload {
    let formatVersion: UInt8
    let sections: [UInt8: Data]  // section_id → raw section bytes
}

enum DiagParseError: Error { case truncated, badHeader }

func parseDiag(_ data: Data) throws -> DiagPayload {
    guard data.count >= 2 else { throw DiagParseError.truncated }
    let version = data[0]
    let count = Int(data[1])
    var sections: [UInt8: Data] = [:]
    var idx = 2
    for _ in 0..<count {
        guard idx + 2 <= data.count else { throw DiagParseError.truncated }
        let id = data[idx]; idx += 1
        let len = Int(data[idx]); idx += 1
        guard idx + len <= data.count else { throw DiagParseError.truncated }
        sections[id] = data.subdata(in: idx..<(idx + len))
        idx += len
    }
    return DiagPayload(formatVersion: version, sections: sections)
}
```

Each section decoder reads its known fields and **ignores anything past the
last field it knows about** so a firmware-side append never breaks the app.

---

## UI guidance

- One screen, one tap, one notification. No streaming, no polling.
- Show **stale-time** ("captured 4 s ago") so the user knows when to refresh.
- Highlight in red:
  - any `init_bitmask` bit that's 0,
  - `last_fault_marker != 0`,
  - `loyalty_store_healthy == 0`,
  - any non-zero error counter (`i2c_errors`, `*_fail_count`),
  - `reset_cause` indicating WDG (bit 3) or LOCKUP (bit 4).
- Make raw values copyable (long-press) so users can paste them into a bug
  report. Especially `flags_raw`, `control_status_raw`, `reset_cause`,
  `last_disconnect_reason`, the calib bytes hex blob.
- Add an **"Export diagnostics"** button that dumps the full hex payload + the
  decoded sections to a share sheet. This is the single most useful
  field-debug affordance once firmware is frozen.

---

## What is *not* in v1 but may appear later

The firmware reserves headroom in every section and reserves section IDs
`0x07..0xFE`. iOS must therefore:

- treat unknown section IDs as **silently ignored** (don't error),
- treat trailing reserved bytes inside a known section as **silently ignored**,
- not assume `section_count` is fixed.

Likely future additions, in priority order: hardfault PC/LR snapshot, BLE
supervision-timeout count, per-state power-mode dwell histograms, motion-log
overflow flag, EEPROM CRC mismatch counter, last sleep entry/exit timestamps.
