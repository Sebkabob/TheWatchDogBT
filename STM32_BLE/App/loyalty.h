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

// Single 6-byte record at 0x10..0x15 (status + token + CRC). Sits inside
// the reserved device-info region (0x00..0x3F), clear of the BD-address
// bytes (0x00..0x06) and well below the motion-log header at 0x40.
#define EEPROM_LOYALTY_ADDR         0x10
#define EEPROM_LOYALTY_LEN          6     // status(1) + token(4) + crc(1)

#define EEPROM_LOYALTY_CLAIMED      0xA7
#define EEPROM_LOYALTY_CLEARED      0xCE

// DEBUG ONLY: flash with =1 once to wipe, then back to =0 and reflash.
// Useful when stale test data leaves a unit "already claimed" and the app
// can't unbond it. Leave at 0 in normal builds.
#define LOYALTY_WIPE_ON_BOOT        (0)

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

#endif /* LOYALTY_H */
