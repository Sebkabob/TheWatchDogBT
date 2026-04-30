# iOS task: parse BatteryDiagnostic v3 (30-byte payload) and surface gauge config readback

## Context

We're tracking a bug where `AverageCurrent()` from the BQ27427 fuel gauge reports a stuck ~−10 mA reading even when the actual battery current is 104 mA (verified externally with a power profiler). To diagnose whether the gauge's config writes are actually sticking, whether SLEEP mode is really off, and whether calibration has been corrupted, the firmware now emits a richer **version 3** BatteryDiagnostic notification.

The wire format has grown from **18 bytes (v2)** to **30 bytes (v3)**. The first byte is still the version, so the app can keep handling older firmware (v2) and any future bumps from the same code path.

You **must not crash** when an older device sends a v2 (18-byte) payload, and you **must not** parse the new fields out of a v2 packet. Use the version byte as the discriminator.

---

## BatteryDiagnostic v3 — wire format (30 bytes, little-endian)

The STM32 is little-endian and writes a `__attribute__((packed))` struct directly to the GATT buffer, so multi-byte fields are little-endian on the wire. Fields are tightly packed — no padding.

| Off | Size | Type      | Field                  | Notes                                                           |
|----:|-----:|-----------|------------------------|-----------------------------------------------------------------|
|   0 |    1 | UInt8     | `version`              | Equals **3** for this layout. (v2 = 18 bytes total.)            |
|   1 |    1 | UInt8     | `socPercent`           | Filtered SOC, 0–100                                             |
|   2 |    2 | UInt16LE  | `voltageMV`            | Battery voltage, mV                                             |
|   4 |    2 | Int16LE   | `currentMA`            | Average current, mA. Negative = discharging                     |
|   6 |    2 | UInt16LE  | `remainingMAH`         | Remaining capacity, mAh                                         |
|   8 |    2 | UInt16LE  | `fullChargeMAH`        | Full charge capacity, mAh                                       |
|  10 |    2 | Int16LE   | `temperature_0_1K`     | 0.1 K units. °C = value/10 − 273.15                             |
|  12 |    2 | UInt16LE  | `flagsRaw`             | BQ27427 `Flags()` register, raw                                 |
|  14 |    2 | UInt16LE  | `controlStatusRaw`     | BQ27427 `CONTROL_STATUS` register, raw                          |
|  16 |    1 | UInt8     | `statusBits`           | Packed convenience flags — see bit table below                  |
|  17 |    1 | UInt8     | `socUnfiltered`        | Raw IT SOC, 0–100                                               |
|  18 |    2 | UInt16LE  | `designCapacityMAH`    | **NEW v3.** Gauge readback. **Expected: 300**                   |
|  20 |    2 | UInt16LE  | `terminateVoltageMV`   | **NEW v3.** Gauge readback. **Expected: 3000**                  |
|  22 |    2 | UInt16LE  | `taperRate`            | **NEW v3.** Gauge readback (0.1 h units). **Expected: 100**     |
|  24 |    2 | UInt16LE  | `opConfigRaw`          | **NEW v3.** OpConfig register raw. **Expected: 0x6458**         |
|  26 |    2 | Int16LE   | `averagePowerMW`       | **NEW v3.** `AveragePower()` at 0x18, mW (signed)               |
|  28 |    1 | Int8      | `boardOffset`          | **NEW v3.** Subclass 104, off 0. Signed counts. **Expected: 0** |
|  29 |    1 | UInt8     | `deadbandMA`           | **NEW v3.** Subclass 107, off 1, mA. **Expected: 5**            |

Total wire length: **30 bytes**. The firmware enforces this with a compile-time `_Static_assert(sizeof(...) == 30, ...)`, so this length is authoritative.

Parsing rule: when `version == 3`, require **length == 30** (or accept `>= 30` and ignore trailing bytes if you want to be future-proof).

### `statusBits` layout (LSB first)

| Bit | Meaning              | Source                            |
|----:|----------------------|-----------------------------------|
| 0   | `isCharging`         | `FLAG_CHG`                        |
| 1   | `isFull`             | `FLAG_FC`                         |
| 2   | `isLow`              | `FLAG_SOC1`                       |
| 3   | `isCritical`         | `FLAG_SOCF`                       |
| 4   | `batteryDetected`    | `FLAG_BAT_DET`                    |
| 5   | `qmaxLearned`        | `CTRL_STATUS` bit 9               |
| 6   | `resistanceLearned`  | `CTRL_STATUS` bit 8               |
| 7   | `itpor`              | `FLAG_ITPOR` (gauge lost config)  |

---

## v2 backward compatibility (18 bytes)

If `version == 2`, parse only bytes 0–17 (the original v2 layout — identical to the first 18 rows of the v3 table). The new fields (`designCapacityMAH` … `deadbandMA`) are absent. Existing app behaviour for v2 must remain unchanged.

If `version` is anything other than 2 or 3, log it, drop the packet, and **do not crash**. We may bump again.

---

## What to do in the iOS app

1. **Update the BatteryDiagnostic notification handler** to dispatch on the leading version byte:
   - `version == 2` → existing 18-byte parser (unchanged behaviour).
   - `version == 3` → new 30-byte parser populating both the v2 fields and the seven new fields.
   - Other → log and drop.

2. **Surface the new fields in whatever debug / diagnostic view currently shows BatteryDiagnostic data.** Pick a sensible spot — the existing battery debug screen, gauge diagnostics view, wherever `voltageMV` / `currentMA` already render. Exact UI placement is your call.

3. **Render expected vs. actual for the four gauge-config readback fields**, and visually flag any mismatch (red text, ⚠️ icon, whatever's idiomatic for the app). Mismatches are the entire reason this telemetry exists — they must not be subtle.

   | Field                  | Expected      | Display hint                                                                                              |
   |------------------------|---------------|-----------------------------------------------------------------------------------------------------------|
   | `designCapacityMAH`    | `300`         | `"300 mAh"` — flag if not 300                                                                             |
   | `terminateVoltageMV`   | `3000`        | `"3000 mV"` — flag if not 3000                                                                            |
   | `taperRate`            | `100`         | `"100 (0.1h)"` — flag if not 100                                                                          |
   | `opConfigRaw`          | `0x6458`      | render hex; flag if any bit differs. **Bit 5 (`0x0020`) is SLEEP — if set, the gauge will lie about current.** |

4. **`averagePowerMW`** — show alongside Voltage and Current. Negative values mean discharging. Useful sanity check: P should approximately equal V × I (sign-corrected).

5. **`boardOffset`** — show as a signed integer with a note: *"Expected: 0. Large absolute values (>5) indicate calibration corruption — current readings will be biased."*

6. **`deadbandMA`** — show as an integer mA with a note: *"Expected: 5 mA. Below this threshold the gauge reports zero current."*

7. The flag bits already exposed via `statusBits` continue to work as before — keep showing them. The new fields complement, they don't replace.

---

## Test plan

- Connect to a freshly-flashed device, subscribe to BatteryDiagnostic, and confirm the first notification has `version == 3` and length 30.
- Hex-dump the first packet so we can confirm wire offsets match the table above.
- Verify each of the four config-readback fields renders the expected value on a known-good device. If any of them is off, that's the bug we're hunting — the UI should make it impossible to miss.
- Pair with an older firmware build that still sends v2 (18 bytes); confirm the app parses it without crashing and without surfacing the v3-only fields.
- Confirm `averagePowerMW` updates each second and tracks `currentMA` × `voltageMV` directionally.

---

## Out of scope

- Don't touch the DEVICESTATUS characteristic — its format is unchanged.
- The BATTERYDIAG UUID is unchanged.
- Don't change subscription / connection logic — the only change is payload parsing and rendering.
