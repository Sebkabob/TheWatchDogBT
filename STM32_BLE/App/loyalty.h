/**
 ******************************************************************************
 * @file    loyalty.h
 * @brief   Application-layer loyalty token store: 4-byte EEPROM-backed
 *          ownership marker, used in lieu of BLE pairing/bonding.
 *          See FW_LOYALTY_TOKEN_PROMPT.md for design notes.
 ******************************************************************************
 */

#ifndef LOYALTY_H
#define LOYALTY_H

#include <stdint.h>
#include <stdbool.h>

/* EEPROM layout (single record, single source of truth):
 *   0x10  status byte: 0xA7=CLAIMED, 0xCE=CLEARED, anything else=BLANK
 *   0x11..0x14  4-byte token (only meaningful when status==CLAIMED)
 *   0x15  CRC8 over status+token (only meaningful when status==CLAIMED)
 *
 * Sits inside the reserved device-info region (0x00..0x3F) — clear of the
 * BD-address bytes (0x00..0x06) and well below the motion-log header at 0x40. */
#define EEPROM_LOYALTY_ADDR         0x10
#define EEPROM_LOYALTY_LEN          6     /* status(1) + token(4) + crc(1) */

#define EEPROM_LOYALTY_CLAIMED      0xA7
#define EEPROM_LOYALTY_CLEARED      0xCE

/* DEBUG ONLY: Set to 1, flash once, then set back to 0 and flash again to
 * leave the unit in a clean state with no loyalty token in EEPROM. Useful
 * when stale data from prior testing has the firmware stuck "already claimed"
 * and you can't unbond from the app. Leave at 0 in normal builds. */
#define LOYALTY_WIPE_ON_BOOT        (0)

/* Initialise from EEPROM. Call once at boot, after I2C is up. */
void Loyalty_Init(void);

/* True iff a loyalty token is currently stored. */
bool Loyalty_IsClaimed(void);

/* True iff Loyalty_Init could not read the EEPROM at boot. While set, the
 * dispatcher must refuse all loyalty operations (including CLAIM) — otherwise
 * a transient I2C glitch at boot would let any phone hijack a claimed device. */
bool Loyalty_StoreUnhealthy(void);

/* Compare a 4-byte incoming token with the stored token.
 * Returns false if no token is stored. */
bool Loyalty_Verify(const uint8_t *incoming_token);

/* Persist a fresh 4-byte token. Called only on a successful CLAIM. */
bool Loyalty_Claim(const uint8_t *token);

/* Wipe the EEPROM region. Called on verified UNBOND or hardware reset hatch. */
bool Loyalty_Wipe(void);

#endif /* LOYALTY_H */
