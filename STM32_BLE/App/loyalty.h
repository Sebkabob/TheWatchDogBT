/***************************************************************************
 * loyalty.h
 * created by Sebastian Forenza 2026
 *
 * Application-layer loyalty token store: 4-byte EEPROM-backed ownership
 * marker, used in lieu of BLE pairing/bonding. See FW_LOYALTY_TOKEN_PROMPT.md
 * for the full design.
 ***************************************************************************/

#ifndef LOYALTY_H
#define LOYALTY_H

#include <stdint.h>
#include <stdbool.h>
#include "eeprom_map.h"

/* Loyalty record address + length live in eeprom_map.h
 * (EEPROM_LOYALTY_ADDR / EEPROM_LOYALTY_LEN). */

#define EEPROM_LOYALTY_CLAIMED      0xA7
#define EEPROM_LOYALTY_CLEARED      0xCE

// DEBUG ONLY: flash with =1 once to wipe, then back to =0 and reflash.
// Useful when stale test data leaves a unit "already claimed" and the app
// can't unbond it. Leave at 0 in normal builds.
#define LOYALTY_WIPE_ON_BOOT        (0)

// USB-C-gated reset window: while open, CLAIM unconditionally overwrites the
// stored token (handles iOS Keychain wipe after app reinstall). Length is a
// compromise between giving the user time to open the app and tap Pair vs.
// limiting the exploitable window for a stranger in BLE range during a charge.
#define LOYALTY_RESET_WINDOW_MS     10000u

void Loyalty_Init(void);
bool Loyalty_IsClaimed(void);

// True iff Loyalty_Init couldn't read the EEPROM. While set, the dispatcher
// MUST refuse all loyalty operations (including CLAIM) — otherwise a
// transient I2C glitch at boot would let any phone hijack a claimed device.
bool Loyalty_StoreUnhealthy(void);

// False if no token is stored or the incoming token doesn't match.
bool Loyalty_Verify(const uint8_t *incoming_token);

bool Loyalty_Claim(const uint8_t *token);

// Wipe on verified UNBOND or the cable-hold recovery hatch.
bool Loyalty_Wipe(void);

// Reset-window control. The window is RAM-only; only the token persists in
// EEPROM. Start is called on a debounced VBUS rising edge (and at boot if
// VBUS is already high); Cancel on the falling edge. IsOpen drives the CLAIM
// dispatcher's overwrite-vs-match decision and lazily expires the timer.
void Loyalty_StartResetWindow(void);
void Loyalty_CancelResetWindow(void);
bool Loyalty_IsResetWindowOpen(void);

#endif /* LOYALTY_H */
