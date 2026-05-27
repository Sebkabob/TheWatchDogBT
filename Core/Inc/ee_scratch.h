/***************************************************************************
 * ee_scratch.h
 * created by Sebastian Forenza 2026
 *
 * Bounds-checked accessors for the 128-byte iOS-owned scratch region at
 * EEPROM_KV_SCRATCH_ADDR. Firmware treats the region as opaque storage —
 * the layout (TLV, fixed offsets, whatever) is owned entirely by the iOS
 * app. Both Read and Write reject any (offset, length) pair that would
 * touch a byte outside the region.
 *
 * This module exists so we can ship a "frozen" firmware that still lets
 * the app add per-device persistent state forever without firmware
 * updates. New app features that need a few bytes per device just claim
 * an offset range in the scratch region and start using it.
 ***************************************************************************/

#ifndef INC_EE_SCRATCH_H_
#define INC_EE_SCRATCH_H_

#include <stdint.h>
#include <stdbool.h>
#include "eeprom_map.h"

/* Result codes for EeScratch_Read / EeScratch_Write. Mapped 1:1 to the
 * RESP_EE_REJECT reason byte sent back over BLE on failure. */
typedef enum {
    EE_SCRATCH_OK         = 0,
    EE_SCRATCH_OUT_OF_BOUNDS = 1,
    EE_SCRATCH_I2C_FAIL   = 2,
} EeScratchStatus_t;

/* Read `length` bytes from offset `offset` (relative to the start of the
 * scratch region) into `dst`. Returns EE_SCRATCH_OK on success.
 * Brackets its own EEPROM power on/off. */
EeScratchStatus_t EeScratch_Read(uint8_t offset, uint8_t length, uint8_t *dst);

/* Write `length` bytes from `src` to offset `offset` in the scratch region.
 * Returns EE_SCRATCH_OK on success. The 16-byte page boundary inside the
 * M24C08 is handled by the m24cxx driver. */
EeScratchStatus_t EeScratch_Write(uint8_t offset, uint8_t length, const uint8_t *src);

#endif /* INC_EE_SCRATCH_H_ */
