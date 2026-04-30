# iOS App Update — Battery Drain Mode

The WatchDogBT firmware now supports a **drain test** mode used to characterise
the battery for the BQ27427 fuel gauge. While active, the device:

- holds the RGB LED **white at maximum brightness** (255/255/255), and
- plays a **continuous 100 Hz buzzer tone** with no gaps.

The firmware **auto-stops** drain mode when the gauge reports `SOC ≤ 5 %`. The
app may also stop drain mode at any time by sending a stop command. Drain mode
is intended to be controlled from the new "Gauge Health" screen.

## What to add

1. A **"Start Drain"** / **"Stop Drain"** button on the Gauge Health view.
2. A confirmation dialog before starting drain (this is destructive — it
   intentionally runs the battery flat, with a loud tone and bright light).
3. Visual indication while drain is active (e.g., a red banner "Draining
   battery — auto-stop at 5 % SOC"). Use the existing `BatteryDiagnostic`
   notification stream to drive this — the firmware will stop drain on its
   own when SOC drops, and the LED/buzzer will go quiet, but the app should
   still display drain progress (current SOC) the whole time.

## BLE protocol — drain command

Drain is controlled by writing to the existing **APPTOWD** characteristic
(write-only, the same one the app uses for arm/disarm and find-my-device).

- Service UUID: `0x183E` (LockService — unchanged).
- APPTOWD characteristic UUID: unchanged (the existing write characteristic
  the app already uses).
- Drain command opcode: **`0xFC`** = `CMD_DRAIN_MODE`.

### Wire format (2 bytes)

| Offset | Type    | Meaning                              |
|--------|---------|--------------------------------------|
| 0      | uint8_t | `0xFC` (CMD_DRAIN_MODE)              |
| 1      | uint8_t | `0x01` = start drain, `0x00` = stop  |

Example payloads:

- Start drain: `[0xFC, 0x01]`
- Stop drain:  `[0xFC, 0x00]`

The firmware ignores any extra bytes after byte 1.

### Behavior

- On **start**: firmware sets RGB LED to white at 255 brightness and begins a
  continuous 100 Hz tone. Idempotent — sending start while already draining
  is a no-op.
- On **stop**: firmware silences the buzzer and turns the LED off, returning
  control to the regular state machine on the next loop iteration.
- **Auto-stop**: firmware monitors `BATTERY_GetSOC()` each main-loop tick.
  When SOC reaches 5 % or lower, it stops on its own. The `BatteryDiagnostic`
  notification will reflect the new SOC and (because drain is no longer
  asserting them) the LED/tone will end.

There is no separate "drain status" notification — the app should infer drain
progress from the existing `BatteryDiagnostic` characteristic, which already
delivers SOC, voltage, current, and temperature once per second.

## Swift example

```swift
enum LockServiceCommand {
    static let drainMode: UInt8 = 0xFC
}

func startDrain() {
    let payload: [UInt8] = [LockServiceCommand.drainMode, 0x01]
    apptowdCharacteristic.write(Data(payload), type: .withResponse)
}

func stopDrain() {
    let payload: [UInt8] = [LockServiceCommand.drainMode, 0x00]
    apptowdCharacteristic.write(Data(payload), type: .withResponse)
}
```

## UX guidance

- Treat drain as a **debug / power-user feature**. Hide behind a long-press,
  developer-flag, or a confirmation modal that explains what will happen.
- While drain is running, the device cannot be silenced except by:
  - sending `[0xFC, 0x00]` from the app, or
  - waiting for SOC to reach 5 % (auto-stop), or
  - removing the battery / power cycling.
- Recommend the user place the device somewhere the loud 100 Hz buzz won't
  bother anyone.
- If the BLE connection drops mid-drain, the device will keep draining until
  auto-stop. Reconnecting and sending stop is the only way to abort early.
- Consider showing estimated time-to-completion using
  `current_mA` and `remaining_mAh` from the `BatteryDiagnostic` payload:
  `hours_remaining ≈ remaining_mAh / abs(current_mA)`.

## Why the user would run this

After a fresh battery is installed (or after `ITPOR` is set), the BQ27427
fuel gauge needs a full discharge under reasonable load to learn `Qmax` and
update its resistance table. The drain mode provides a known, repeatable
discharge path to drive the gauge through that learning cycle. Once
`qmax_learned && res_learned` are both set in `status_bits`, the gauge has
converged and the drain test is no longer needed.
