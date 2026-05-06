# WatchDogBT — Disconnect Sound (iOS implementation prompt)

## Goal

Firmware **V1.11.26** adds a new persisted user setting: **disconnectSoundDisabled**.
When the toggle is on, the three-tone descending chime that plays whenever the
BLE link drops is silenced. Today this chime fires every time the user closes
the app, walks out of range, or backgrounds the phone — which is loud and not
always wanted. The new toggle gives the user a way to keep the alarm audible
while killing only the disconnect chime.

This is independent of **Disable Alarm** (the existing bit 1 toggle). The two
flags do not affect each other:

- `Disable Alarm` ON, `Disable Disconnect Sound` OFF → disconnect chime plays,
  alarm/find-my/drain are all silent.
- `Disable Alarm` OFF, `Disable Disconnect Sound` ON → disconnect chime is
  silent, alarm/find-my/drain still play normally.
- Both ON → silent in every audible path.

## What changes on the iOS side

1. **Add a new Settings row labelled "Disable Disconnect Sound"** (or whatever
   localized phrasing matches the existing copy). Place it **directly above
   the existing "Disable Alarm" row** in the Settings list. The two are related
   ("disable this sound, disable that sound") and grouping them makes the
   relationship obvious; putting Disconnect Sound on top keeps Disable Alarm —
   the larger / more consequential toggle — in its current position.
2. Persist the toggle's UI state from what the firmware reports (see
   "DEVICESTATUS echo" below). Do not store it locally as the source of truth —
   the firmware is authoritative across reboots.
3. When the user flips the toggle, send a settings write with the new bit
   layout (see "Settings write" below). On the next DEVICESTATUS notification
   the firmware will echo the persisted value back; reconcile the UI from that.

## Wire protocol changes

The `deviceInfo` byte (`cmd_data[1]` of a settings write, and byte 13 of the
19-byte DEVICESTATUS notification) gains one more bit:

| Bit | Name | Meaning |
|-----|------|---------|
| 0 | `HIGH_PERF` | unchanged — 50 Hz status updates when set |
| 1 | `alarmDisabled` | unchanged — silences alarm / find-my / drain / one-shot tones |
| 2 | `disconnectSoundDisabled` | **new** — silences the disconnect chime only |
| 3..7 | reserved | always 0 — firmware masks them off on receive and on echo |

Both `alarmDisabled` and `disconnectSoundDisabled` are persisted in EEPROM by
firmware in their own records — iOS does not need to re-send them on every
connect; whatever the user last set is what the device will be in on the next
boot.

### Settings write (iOS → device, `APPTOWD` characteristic)

After the 4-byte loyalty token, the settings core is:

```
[deviceState, deviceInfo, alarm_duration_s?, led_brightness?, ts0..ts5]
```

`deviceInfo` byte:

```
bit 7 6 5 4 3 |  2                          |  1              |  0
       (0)    | disconnectSoundDisabled     | alarmDisabled   | HIGH_PERF
```

Examples (assuming HIGH_PERF off):

| State | `deviceInfo` value |
|-------|-------------------|
| neither flag set | `0x00` |
| only Disable Alarm | `0x02` |
| only Disable Disconnect Sound | `0x04` |
| both | `0x06` |

If the iOS app is currently sending `(deviceInfo & 0x03)` it must be widened to
`(deviceInfo & 0x07)` so bit 2 round-trips. **Older app builds that send only
bits 0..1 are still compatible** — bit 2 simply stays at whatever value the
firmware previously persisted (it is not cleared by an absent bit).

### DEVICESTATUS echo (device → iOS, byte 13 of the 19-byte payload)

The same byte 13 that already echoed bits 0..1 now echoes bit 2 too:

```
byte 13 = (HIGH_PERF ? 0x01 : 0)
        | (alarmDisabled ? 0x02 : 0)
        | (disconnectSoundDisabled ? 0x04 : 0)
```

Bits 3..7 of byte 13 remain reserved (always 0). Read bit 2 to drive the toggle
state in the new Settings row.

## Visual / UX notes

- Use the same toggle component already used for "Disable Alarm" — same row
  height, same accessory, same persistence semantics. The two rows should look
  identical apart from the label.
- No subtitle / explainer text is required, but if the existing Disable Alarm
  row has one, mirror that style: e.g.
  *"Silences the chime that plays when the device disconnects."*
- The toggle must **survive an app reinstall** — because the firmware is the
  source of truth, the new install will read the persisted state from the next
  DEVICESTATUS notification. Do not show a default "off" until that first
  notification arrives; show a brief skeleton / disabled state instead.

## Firmware version gate

This bit is only meaningful on firmware **≥ V1.11.26**. The firmware version
triplet sits in DEVICESTATUS bytes 16..18 (`MAJOR`, `MAIN`, `V2`) — if the
connected device reports a lower version, **hide the "Disable Disconnect Sound"
row entirely** so the user is not offered a setting that the device cannot
honor. (Older firmware will silently mask bit 2 off and the chime will keep
playing regardless of what iOS sends.)

## Test checklist

- [ ] Toggle ON → disconnect → no chime; reconnect → DEVICESTATUS byte 13
      bit 2 reads `1`.
- [ ] Toggle OFF → disconnect → three-tone descending chime plays; reconnect →
      byte 13 bit 2 reads `0`.
- [ ] Toggle ON, full power-cycle the device → reconnect → toggle is still ON
      (state survived the reboot).
- [ ] Disable Alarm ON, Disable Disconnect Sound OFF → disconnect chime plays
      (Disable Alarm does NOT silence the disconnect chime).
- [ ] Disable Alarm OFF, Disable Disconnect Sound ON → trigger an alarm —
      buzzer fires normally; disconnect afterward — no chime.
- [ ] Connecting to firmware < V1.11.26 → the new row is hidden.
- [ ] Settings list visual order is: …, **Disable Disconnect Sound**,
      **Disable Alarm**, … (Disconnect Sound directly above Disable Alarm).
