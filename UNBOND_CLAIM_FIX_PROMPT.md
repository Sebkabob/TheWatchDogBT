# Firmware fix: post-UNBOND CLAIM rejected

## The bug

After a successful UNBOND from the iOS app, the firmware refuses the next CLAIM unless the USB-C reset window happens to be open. User-visible symptom: tap "Forget Device" → tap to re-pair from the same iPhone → iOS shows "Not your device!".

## Repro

1. Phone has paired (claimed) device. EEPROM `0x10` = `[0xA7, t0..t3, crc]`.
2. User taps "Forget Device".
3. iOS sends `[0xC0, t0..t3]`. Firmware verifies, calls `Loyalty_Wipe()` (EEPROM becomes `[0xCE, 0xFF, 0xFF, 0xFF, 0xFF, crc]`), replies `[0xE4, 0x01]`, disconnects.
4. User taps to re-pair. iOS `BondManager` was cleared by `clearLocalStateAfterUnpair`, so `isFirstClaim` is true and iOS sends `[0xC1, t0..t3]` (CLAIM, not VERIFY).
5. Firmware: `Loyalty_StoreUnhealthy()` false ✓, `data_length >= 5` ✓, `Loyalty_IsResetWindowOpen()` **false** (no cable edge), `Loyalty_IsClaimed()` **false** (just wiped). Falls through to `RESP_REJECT`.

The wipe is correct — `s_claimed = false` and the EEPROM record is properly cleared. The bug is in the CLAIM dispatcher: there's no branch for "unclaimed AND outside reset window", so it conservatively rejects a legitimate fresh claim on an unowned device.

## Where

`STM32_BLE/App/lockservice_app.c::LOCKSERVICE_Notification`, the `CMD_CLAIM_DEVICE` branch (around lines 300–331). Currently:

```c
if (first_byte == CMD_CLAIM_DEVICE) {
    if (data_length < 1 + LOYALTY_TOKEN_LEN) { /* REJECT */ }

    if (Loyalty_IsResetWindowOpen()) {
        // Claim/overwrite via Loyalty_Claim → CLAIM_OK or REJECT
        break;
    }

    if (Loyalty_IsClaimed() && Loyalty_Verify(&received_data[1])) {
        // RESP_VERIFY_OK
        break;
    }
    Loyalty_SendResponse(RESP_REJECT, 0x01, 1);  // ← bug: hits this on unclaimed device
    break;
}
```

The reset-window gate exists to stop a stranger from overwriting an *already-claimed* device's token. There's no security reason to gate a fresh claim on an *unowned* device — UNBOND deliberately put it in that state.

## Fix (option 1 from the diagnosis chat)

Add a third successful branch: if `!Loyalty_IsClaimed()`, the device is unowned, so accept the CLAIM unconditionally. Restructure the handler to roughly:

```c
if (first_byte == CMD_CLAIM_DEVICE) {
    if (data_length < 1 + LOYALTY_TOKEN_LEN) {
        Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
        break;
    }

    // Reset window open: USB-C-gated overwrite of an existing claim
    // (handles iOS Keychain wipe / different-phone takeover).
    if (Loyalty_IsResetWindowOpen()) {
        if (Loyalty_Claim(&received_data[1])) {
            Loyalty_SendResponse(RESP_CLAIM_OK, 0x01, 0);
        } else {
            Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
        }
        break;
    }

    // Unowned device (factory-fresh or post-UNBOND): anyone may claim.
    // No security gate needed — there's no existing claim to protect.
    if (!Loyalty_IsClaimed()) {
        if (Loyalty_Claim(&received_data[1])) {
            Loyalty_SendResponse(RESP_CLAIM_OK, 0x01, 0);
        } else {
            Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
        }
        break;
    }

    // Device IS claimed; legitimate owner re-establishing a session
    // (their phone still has the token but BondManager was cleared).
    if (Loyalty_Verify(&received_data[1])) {
        Loyalty_SendResponse(RESP_VERIFY_OK, 0x01, 0);
        break;
    }

    Loyalty_SendResponse(RESP_REJECT, 0x01, 1);
    break;
}
```

Update the inline comment block around lines 306–310 — the existing wording ("USB-C reset window is the only path that's allowed to overwrite (or write a fresh) token") is wrong after this change. The window is only required to **overwrite** an existing claim, not to write a fresh one on an unclaimed device.

## Don't touch

- `loyalty.c` / `loyalty.h` — the store layer is correct.
- `CMD_UNBOND_DEVICE`, `CMD_VERIFY_OWNER` branches — they're correct.
- The 30-second cable-hold recovery hatch in `main.c` — unrelated, still needed for the lost-phone case.

## Docs

Update `FW_LOYALTY_TOKEN_PROMPT.md` to reflect the new dispatcher table (CLAIM on `!s_claimed` no longer requires the USB-C window). Keep the security reasoning explicit: the reset window remains the only path that can **overwrite** a stored token.

The CLAUDE.md description of the loyalty token already says CLAIM is "idempotent when the incoming token matches" — leave that line alone, but if you find any wording that implies CLAIM on an unowned device requires the reset window, fix it.

## Version bump

Per `CLAUDE.md`, this is a `main`-branch commit:
- Recompute `MAIN = git rev-list --count --first-parent main` (then +1 for the commit you're about to make).
- `V2 = 0` (cascading reset on `MAIN` bump).
- Update `Core/Inc/firmware_version.h` (`FW_VERSION_MAIN`, `FW_VERSION_V2`, `FW_VERSION_STRING`) and the `Current: V…` + reconciled-sha line in `CLAUDE.md` in the same commit.
- Commit message: `version bump to Vx.y.z`.

## After this lands

The iOS side has its own Claude Code instance running — it has a coordinated prompt at `WatchDog_iOS/UNBOND_CLAIM_FIX_PROMPT.md`. No iOS code change is required for the fix itself; that prompt covers stale-comment cleanup and end-to-end validation.
