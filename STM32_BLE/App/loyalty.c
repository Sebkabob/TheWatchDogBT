/**
 ******************************************************************************
 * @file    loyalty.c
 * @brief   Application-layer loyalty token store. EEPROM-backed 4-byte
 *          ownership marker; replaces BLE pairing/bonding for the prototype.
 *
 * Layout (single record at 0x10..0x15):
 *   [0]    status: 0xA7=CLAIMED, 0xCE=CLEARED, anything else=BLANK
 *   [1..4] 4-byte token (valid only when status == CLAIMED && CRC matches)
 *   [5]    CRC8 over [0..4]
 *
 * EEPROM map (M24C08 = 1024 B):
 *   0x00..0x06  BD address: magic(1) + addr(6)            (motion_logger.h)
 *   0x07..0x0F  reserved
 *   0x10..0x15  loyalty record (this file)
 *   0x16..0x3F  reserved (unused inside the device-info block)
 *   0x40..0x47  motion-log header                         (motion_logger.h)
 *   0x48..0x3FF motion-log event data
 ******************************************************************************
 */

#include "loyalty.h"
#include "lockservice_app.h"
#include "main.h"
#include "app_common.h"
#include "power_management.h"
#include "motion_logger.h"   /* EEPROM_I2C_ADDRESS */

#define M24CXX_MODEL 0
#include "m24cxx.h"

#include <string.h>

static M24CXX_HandleTypeDef s_eeprom;
extern I2C_HandleTypeDef    hi2c1;

static bool    s_claimed   = false;
static bool    s_unhealthy = false;
static uint8_t s_token[LOYALTY_TOKEN_LEN] = {0};

/* CRC-8/CCITT (poly 0x07, init 0x00). Cheap byte-wise loop, no table. */
static uint8_t loyalty_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* Build a 6-byte record in dst from status + token. */
static void loyalty_pack(uint8_t status, const uint8_t *token, uint8_t dst[EEPROM_LOYALTY_LEN])
{
    dst[0] = status;
    if (token) {
        memcpy(&dst[1], token, LOYALTY_TOKEN_LEN);
    } else {
        memset(&dst[1], 0xFF, LOYALTY_TOKEN_LEN);
    }
    dst[5] = loyalty_crc8(dst, 5);
}

/* Write+readback an EEPROM region with up to 3 retries. Caller has already
 * powered on the EEPROM and called m24cxx_init. m24cxx_write ACK-polls until
 * the flash-commit cycle completes, so no extra HAL_Delay is needed. */
static bool loyalty_write_verify(uint16_t addr, const uint8_t *src, uint16_t len)
{
    uint8_t readback[EEPROM_LOYALTY_LEN];
    if (len > sizeof(readback)) {
        return false;
    }
    for (int attempt = 0; attempt < 3; attempt++) {
        if (m24cxx_write(&s_eeprom, addr, (uint8_t *)src, len) != M24CXX_Ok) {
            APP_DBG_MSG("Loyalty: write attempt %d @0x%02X failed\n", attempt, addr);
            continue;
        }
        if (m24cxx_read(&s_eeprom, addr, readback, len) != M24CXX_Ok) {
            APP_DBG_MSG("Loyalty: readback attempt %d @0x%02X failed\n", attempt, addr);
            continue;
        }
        if (memcmp(readback, src, len) == 0) {
            return true;
        }
        APP_DBG_MSG("Loyalty: readback mismatch attempt %d @0x%02X\n", attempt, addr);
    }
    return false;
}

void Loyalty_Init(void)
{
    s_claimed   = false;
    s_unhealthy = false;
    memset(s_token, 0, sizeof(s_token));

#if LOYALTY_WIPE_ON_BOOT
    APP_DBG_MSG("Loyalty: LOYALTY_WIPE_ON_BOOT=1 - wiping EEPROM at boot\n");
    (void)Loyalty_Wipe();
    APP_DBG_MSG("Loyalty: post-wipe complete, proceeding with normal init\n");
#endif

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        APP_DBG_MSG("Loyalty: m24cxx_init failed - store UNHEALTHY\n");
        s_unhealthy = true;
        PowerMgmt_EEPROM_PowerOff();
        return;
    }

    uint8_t buf[EEPROM_LOYALTY_LEN] = {0};
    if (m24cxx_read(&s_eeprom, EEPROM_LOYALTY_ADDR, buf, EEPROM_LOYALTY_LEN) != M24CXX_Ok) {
        APP_DBG_MSG("Loyalty: EEPROM read failed - store UNHEALTHY\n");
        s_unhealthy = true;
        PowerMgmt_EEPROM_PowerOff();
        return;
    }

    APP_DBG_MSG("Loyalty: EEPROM[0x%02X..]=%02X %02X %02X %02X %02X %02X\n",
                EEPROM_LOYALTY_ADDR,
                buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

    if (buf[0] == EEPROM_LOYALTY_CLEARED) {
        APP_DBG_MSG("Loyalty: status=CLEARED - unclaimed\n");
    } else if (buf[0] == EEPROM_LOYALTY_CLAIMED) {
        uint8_t expected = loyalty_crc8(buf, 5);
        if (expected == buf[5]) {
            memcpy(s_token, &buf[1], LOYALTY_TOKEN_LEN);
            s_claimed = true;
            APP_DBG_MSG("Loyalty: token loaded (claimed, CRC ok)\n");
        } else {
            /* CRC mismatch on a CLAIMED record. Two possibilities:
             *   (a) Legacy format from pre-CRC firmware: status=0xA7, token at
             *       [1..4], buf[5] is whatever (often 0x00 from the old write
             *       path that did `buf[5] = 0x00`). Migrate in place by
             *       rewriting the record with the correct CRC.
             *   (b) Genuine bit-rot on a new-format record. Indistinguishable
             *       from (a) at this point — if the token bytes are corrupt,
             *       VERIFY will fail on the next connect and the user re-claims,
             *       same outcome as without migration.
             * We attempt the rewrite once. If it persists, treat the device as
             * claimed. If the write fails, drop to UNHEALTHY (safer than
             * accepting an in-RAM claim we couldn't persist). */
            APP_DBG_MSG("Loyalty: CRC mismatch (got 0x%02X exp 0x%02X) - migrating legacy record\n",
                        buf[5], expected);

            uint8_t migrated[EEPROM_LOYALTY_LEN];
            loyalty_pack(EEPROM_LOYALTY_CLAIMED, &buf[1], migrated);

            if (loyalty_write_verify(EEPROM_LOYALTY_ADDR, migrated, EEPROM_LOYALTY_LEN)) {
                memcpy(s_token, &buf[1], LOYALTY_TOKEN_LEN);
                s_claimed = true;
                APP_DBG_MSG("Loyalty: legacy record migrated (CRC=0x%02X persisted)\n",
                            migrated[5]);
            } else {
                APP_DBG_MSG("Loyalty: legacy migration write failed - store UNHEALTHY\n");
                s_unhealthy = true;
            }
        }
    } else {
        APP_DBG_MSG("Loyalty: status=0x%02X - blank/unowned\n", buf[0]);
    }

    PowerMgmt_EEPROM_PowerOff();
}

bool Loyalty_IsClaimed(void)
{
    return s_claimed;
}

bool Loyalty_StoreUnhealthy(void)
{
    return s_unhealthy;
}

bool Loyalty_Verify(const uint8_t *incoming_token)
{
    if (!s_claimed || incoming_token == NULL) {
        return false;
    }
    return (memcmp(s_token, incoming_token, LOYALTY_TOKEN_LEN) == 0);
}

bool Loyalty_Claim(const uint8_t *token)
{
    if (token == NULL || s_unhealthy) {
        return false;
    }

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        APP_DBG_MSG("Loyalty: claim - m24cxx_init failed\n");
        PowerMgmt_EEPROM_PowerOff();
        return false;
    }

    uint8_t record[EEPROM_LOYALTY_LEN];
    loyalty_pack(EEPROM_LOYALTY_CLAIMED, token, record);

    bool ok = loyalty_write_verify(EEPROM_LOYALTY_ADDR, record, EEPROM_LOYALTY_LEN);
    PowerMgmt_EEPROM_PowerOff();

    if (!ok) {
        APP_DBG_MSG("Loyalty: CLAIM_FAILED after 3 attempts\n");
        return false;
    }

    memcpy(s_token, token, LOYALTY_TOKEN_LEN);
    s_claimed = true;
    APP_DBG_MSG("Loyalty: claimed (EEPROM verified)\n");
    return true;
}

bool Loyalty_Wipe(void)
{
    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        APP_DBG_MSG("Loyalty: wipe - m24cxx_init failed\n");
        PowerMgmt_EEPROM_PowerOff();
        memset(s_token, 0, sizeof(s_token));
        s_claimed = false;
        return false;
    }

    uint8_t record[EEPROM_LOYALTY_LEN];
    loyalty_pack(EEPROM_LOYALTY_CLEARED, NULL, record);

    bool ok = loyalty_write_verify(EEPROM_LOYALTY_ADDR, record, EEPROM_LOYALTY_LEN);
    PowerMgmt_EEPROM_PowerOff();

    /* Always clear in-RAM state so the current session sees the device as
     * unclaimed even if EEPROM persistence is broken. */
    memset(s_token, 0, sizeof(s_token));
    s_claimed = false;

    APP_DBG_MSG("Loyalty_Wipe: persisted=%d\n", (int)ok);
    return ok;
}
