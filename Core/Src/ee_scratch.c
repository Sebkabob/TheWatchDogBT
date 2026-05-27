/***************************************************************************
 * ee_scratch.c
 * created by Sebastian Forenza 2026
 *
 * Implementation of the iOS-owned EEPROM scratch region. See ee_scratch.h.
 ***************************************************************************/

#include "main.h"            /* I2C_HandleTypeDef + HAL types for m24cxx.h */
#include "ee_scratch.h"
#include "power_management.h"

#define M24CXX_MODEL 0
#include "m24cxx.h"

extern I2C_HandleTypeDef hi2c1;

static M24CXX_HandleTypeDef s_scratch_eeprom;

/***************************************************************************
 * bounds_ok — single check that the (offset, length) pair stays inside the
 *   reserved region. Treating zero-length requests as invalid lets callers
 *   reject malformed BLE writes without a separate code path.
 ***************************************************************************/
static bool bounds_ok(uint8_t offset, uint8_t length)
{
    if (length == 0)                                      return false;
    if (offset >= EEPROM_KV_SCRATCH_LEN)                  return false;
    if ((uint32_t)offset + length > EEPROM_KV_SCRATCH_LEN) return false;
    return true;
}

EeScratchStatus_t EeScratch_Read(uint8_t offset, uint8_t length, uint8_t *dst)
{
    if (dst == NULL)             return EE_SCRATCH_OUT_OF_BOUNDS;
    if (!bounds_ok(offset, length)) return EE_SCRATCH_OUT_OF_BOUNDS;

    EeScratchStatus_t status = EE_SCRATCH_OK;

    PowerMgmt_EEPROM_PowerOn();

    if (m24cxx_init(&s_scratch_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        status = EE_SCRATCH_I2C_FAIL;
        goto done;
    }

    uint32_t addr = (uint32_t)EEPROM_KV_SCRATCH_ADDR + offset;
    if (m24cxx_read(&s_scratch_eeprom, addr, dst, length) != M24CXX_Ok) {
        status = EE_SCRATCH_I2C_FAIL;
    }

done:
    PowerMgmt_EEPROM_PowerOff();
    return status;
}

EeScratchStatus_t EeScratch_Write(uint8_t offset, uint8_t length, const uint8_t *src)
{
    if (src == NULL)             return EE_SCRATCH_OUT_OF_BOUNDS;
    if (!bounds_ok(offset, length)) return EE_SCRATCH_OUT_OF_BOUNDS;

    EeScratchStatus_t status = EE_SCRATCH_OK;

    PowerMgmt_EEPROM_PowerOn();

    if (m24cxx_init(&s_scratch_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        status = EE_SCRATCH_I2C_FAIL;
        goto done;
    }

    uint32_t addr = (uint32_t)EEPROM_KV_SCRATCH_ADDR + offset;
    // m24cxx_write splits across the 16-byte page boundary internally.
    if (m24cxx_write(&s_scratch_eeprom, addr, (uint8_t *)src, length) != M24CXX_Ok) {
        status = EE_SCRATCH_I2C_FAIL;
    }

done:
    PowerMgmt_EEPROM_PowerOff();
    return status;
}
