# iOS investigation: BatteryDiagnostic notifications not arriving

## Symptom

The user reports the iOS app is no longer receiving BatteryDiagnostic data. The
firmware side has been re-audited end-to-end and is intact (see "Firmware proof"
below). Nothing has changed on the iOS side recently per the user, so either:

  (a) the iOS app never subscribed to this characteristic in the first place,
  (b) it subscribes but the parser silently drops the payload (e.g. version
      check, length check), or
  (c) the UUID / handle the app is looking for doesn't match what the firmware
      now exposes.

Your job: figure out which.

## What firmware currently exposes (authoritative)

Service: LockService, 16-bit UUID `0x183E` (primary).

Four characteristics, declared in this order in `lockservice_chars[]`:

| Idx | Name          | Properties      | UUID (128-bit, little-endian on the wire as stored) |
|-----|---------------|-----------------|-----------------------------------------------------|
| 0   | APPTOWD       | WRITE           | `20 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00`   |
| 1   | DEVICESTATUS  | NOTIFY + CCCD   | `00 09 00 00 00 00 00 00 00 00 00 00 00 00 00 00`   |
| 2   | MOTIONDATA    | (none)          | `01 33 00 00 00 00 00 00 00 00 00 00 00 00 00 00`   |
| 3   | BATTERYDIAG   | NOTIFY + CCCD   | `42 44 00 00 00 00 00 00 00 00 00 00 00 00 00 00`   |

The STM stack stores 128-bit UUIDs LSB-first; iOS's CoreBluetooth presents them
MSB-first. So the BATTERYDIAG UUID as iOS sees it is:

    00000000-0000-0000-0000-000000004442

(The other characteristics follow the same convention — APPTOWD ends in `0220`,
DEVICESTATUS ends in `0900`, MOTIONDATA ends in `3301`. If the app already
matches DEVICESTATUS correctly, it is using the same byte-reversal convention.)

## Wire format (BATTERYDIAG, version 2, 18 bytes, little-endian)

Defined in `lockservice_app.c:476–528` (`LOCKSERVICE_SendBatteryDiagnostic`).

```
offset  size  field                 notes
 0      u8    version               = 2  (was 1 previously — see migration note)
 1      u8    soc_percent           filtered SOC, 0..100
 2      u16   voltage_mV
 4      i16   current_mA            negative = discharging
 6      u16   remaining_mAh
 8      u16   full_charge_mAh
10      i16   temperature_0_1K      divide by 10, subtract 273.15 for °C
12      u16   flags_raw             BQ27427 Flags() register
14      u16   control_status_raw    BQ27427 CONTROL_STATUS register
16      u8    status_bits           packed flags, see below
17      u8    soc_unfiltered        raw IT SOC, 0..100
                                    *** v1 had a reserved 0x00 byte here ***
```

`status_bits` (LSB first):

```
bit 0  is_charging   (FLAG_CHG)
bit 1  is_full       (FLAG_FC)
bit 2  is_low        (FLAG_SOC1)
bit 3  is_critical   (FLAG_SOCF)
bit 4  bat_detected  (FLAG_BAT_DET)
bit 5  qmax_learned  (CTRL_STATUS bit 9)
bit 6  res_learned   (CTRL_STATUS bit 8)
bit 7  itpor         (FLAG_ITPOR)
```

A `_Static_assert` enforces `sizeof(payload) == 18` at compile time, so the
firmware cannot ship a different length.

## Send cadence

`Core/Src/main.c:206-213`:

```c
static uint32_t last_battery_check = 0;
if (HAL_GetTick() - last_battery_check > 1000) {
    last_battery_check = HAL_GetTick();
    if (!PowerMgmt_IsLowPower()) {
        BATTERY_UpdateState();
        LOCKSERVICE_SendBatteryDiagnostic();
    }
}
```

So: once per second while connected and not in low-power. The send function only
guards on `ConnectionHandle != 0xFFFF`; it does **not** check whether the
central enabled the CCCD before calling `aci_gatt_srv_notify`. That means if iOS
hasn't subscribed, the notify is emitted into the void (the controller may drop
it or buffer it, but iOS will not deliver it to the app delegate without an
active subscription).

## Firmware proof (so you don't have to re-derive it)

- Char declared with NOTIFY + CCCD: `STM32_BLE/App/lockservice.c:174-184`.
- Handle captured at init: `lockservice.c:401`
  (`LOCKSERVICE_Context.BatterydiagCharHdle = ...lockservice_chars[3]`).
- CCCD-write handler routes to enable/disable evt:
  `lockservice.c:281-297`.
- Notify dispatch: `lockservice.c:495-503` calls `aci_gatt_srv_notify` on
  `BatterydiagCharHdle + 1` (the value attribute).
- Build & call site: `lockservice_app.c:476-528`.
- Periodic trigger: `Core/Src/main.c:211`.

## What I need you to verify on the iOS side, in order

1. **Service & characteristic discovery.** Is the app discovering the
   BATTERYDIAG characteristic at all? Search for any reference to UUID
   `00000000-0000-0000-0000-000000004442` (or the bytes `4244` / `0x42 0x44`).
   - If discovery uses an explicit UUID list, confirm BATTERYDIAG is in the
     list. If it's not, that's the bug.
   - If discovery is unfiltered (`discoverCharacteristics(nil, …)`) confirm the
     resulting characteristic is being matched to a Swift property somewhere.

2. **Subscription.** After discovery, the app must call
   `peripheral.setNotifyValue(true, for: batteryDiagChar)`. Confirm this is
   actually invoked for BATTERYDIAG and that `peripheral(_:didUpdateNotificationStateFor:error:)`
   reports `isNotifying == true` with `error == nil`. If subscription is never
   attempted, that is the bug — the firmware sends notifies but iOS will not
   surface them without the CCCD write.

3. **Delegate callback.** Check that
   `peripheral(_:didUpdateValueFor:error:)` has a branch matching the
   BATTERYDIAG characteristic. If it dispatches by UUID and the UUID constant
   is wrong/missing, the payload will be received but ignored.

4. **Parser.**
   - Length check: if the parser requires `data.count == 16` (the old v1 size),
     v2 (18 bytes) will be rejected. The parser must accept `>= 18` or
     specifically `== 18` for v2.
   - Version check: does the app gate on `version == 1`? It must accept
     `version == 2`. The byte at offset 17 changed meaning (was reserved/0,
     now `soc_unfiltered` 0..100). A v1 parser that ignored offset 17 will
     still work for the v2 fields it knows about, but a strict v1 parser
     (e.g. asserting offset 17 == 0) will reject every v2 packet.
   - Endianness: payload is little-endian. iOS on Apple silicon is little-endian
     too, so a direct memory load works, but if the parser uses `Data.withUnsafeBytes`
     + manual byte assembly, double-check it isn't doing big-endian shifts.

5. **Connection / pairing.** Advertising uses `HCI_ADV_FILTER_ACCEPT_LIST_CONNECT`
   — only bonded devices can connect. If iOS lost its bonding (e.g. the device
   was forgotten in iOS Bluetooth settings), the connection itself will fail
   long before notifications matter. Confirm the app is reaching a connected
   state and is receiving DEVICESTATUS notifies (those use the same path; if
   DEVICESTATUS works and BATTERYDIAG doesn't, the problem is BATTERYDIAG-specific
   per the steps above).

## Report back

Tell me concretely which of (1)–(5) is broken, with file/line citations on the
iOS side. If it's a parser/length/version mismatch, propose the fix. If
BATTERYDIAG simply isn't wired up on iOS at all, list every place a new
characteristic needs to be added (UUID constant, discovery list, subscription
call, delegate switch, parser, model object) so the user can decide whether
to do it themselves or ask you to.

Do **not** modify firmware files. The firmware is correct; the bug is on the
iOS side or in the iOS↔firmware contract.
