/**
 ******************************************************************************
 * @file    loyalty.c
 * @brief   Application-layer loyalty token store. EEPROM-backed 4-byte
 *          ownership marker; replaces BLE pairing/bonding for the prototype.
 *
 * --- M24C08 driver / EEPROM-layout investigation (recorded for posterity) ---
 *
 * Driver: Drivers/M24C08/m24cxx.c
 *   - m24cxx_write(): per-page HAL_I2C_Mem_Write followed by i2c_wait()
 *     (file-static, line ~34) which busy-polls HAL_I2C_IsDeviceReady until
 *     the chip ACKs again. Returns M24CXX_Ok ONLY after the internal flash-
 *     commit cycle has actually completed — i.e. ACK polling is honoured.
 *   - m24cxx_read(): a straight HAL_I2C_Mem_Read; no busy-poll, no delay.
 *   - There is NO exposed wait/busy-poll helper.
 *
 * Proven write pattern (app_ble.c BLE_Init lines 346-348, 364-366): naked
 * m24cxx_write with no extra HAL_Delay and no readback. Works in production
 * for the BD-address persistence path, so driver timing is not the bottleneck.
 *
 * EEPROM map (M24C08 = 1024 B):
 *   0x00..0x06  BD address: magic(1) + addr(6)            (motion_logger.h)
 *   0x07..0x0F  reserved
 *   0x10..0x15  loyalty token: magic(1) + token(4) + rsvd (this file)
 *   0x16..0x17  reserved
 *   0x18        loyalty CLEARED sentinel  (NEW — this file)
 *   0x19..0x3F  reserved (unused inside the device-info block)
 *   0x40..0x47  motion-log header                         (motion_logger.h)
 *   0x48..0x3FF motion-log event data
 *
 * Note: the prompt suggested 0x40 for the cleared sentinel, but 0x40 is
 * occupied by the motion-log header. 0x18 sits inside the reserved 64-byte
 * device-info block, well clear of the loyalty token bytes (0x10..0x15) and
 * far below the 0x40 motion-log boundary.
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

/* Mirror motion_logger.c's pattern: a file-local handle and the shared I2C bus. */
static M24CXX_HandleTypeDef s_eeprom;
extern I2C_HandleTypeDef    hi2c1;

static bool    s_claimed = false;
static uint8_t s_token[LOYALTY_TOKEN_LEN] = {0};

void Loyalty_Init(void)
{
    s_claimed = false;
    memset(s_token, 0, sizeof(s_token));

#if LOYALTY_WIPE_ON_BOOT
    APP_DBG_MSG("Loyalty: LOYALTY_WIPE_ON_BOOT=1 - wiping EEPROM at boot\n");
    /* Run the full robust wipe (3 retries, readback) before normal init. */
    (void)Loyalty_Wipe();
    APP_DBG_MSG("Loyalty: post-wipe complete, proceeding with normal init\n");
#endif

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        APP_DBG_MSG("Loyalty: m24cxx_init failed\n");
        PowerMgmt_EEPROM_PowerOff();
        return;
    }

    /* 1. Check the "explicitly cleared" sentinel first. Takes precedence
     *    over the magic byte — if UNBOND landed this byte we treat the
     *    device as unclaimed regardless of what the legacy magic region
     *    says. */
    uint8_t cleared = 0;
    if (m24cxx_read(&s_eeprom, EEPROM_LOYALTY_CLEARED_ADDR, &cleared, 1) == M24CXX_Ok) {
        APP_DBG_MSG("Loyalty: cleared sentinel @0x%02X = 0x%02X\n",
                    EEPROM_LOYALTY_CLEARED_ADDR, cleared);
        if (cleared == EEPROM_LOYALTY_CLEARED_VALUE) {
            APP_DBG_MSG("Loyalty: cleared sentinel set - device is unclaimed (overrides magic byte)\n");
            PowerMgmt_EEPROM_PowerOff();
            return;
        }
    } else {
        APP_DBG_MSG("Loyalty: cleared sentinel read failed; falling through to magic byte\n");
    }

    /* 2. Sentinel not set — read the loyalty area normally. */
    uint8_t buf[EEPROM_LOYALTY_LEN] = {0};
    if (m24cxx_read(&s_eeprom, EEPROM_LOYALTY_ADDR, buf, EEPROM_LOYALTY_LEN) == M24CXX_Ok) {
        APP_DBG_MSG("Loyalty: EEPROM[0x%02X..]=%02X %02X %02X %02X %02X %02X\n",
                    EEPROM_LOYALTY_ADDR,
                    buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

        if (buf[0] == EEPROM_LOYALTY_MAGIC) {
            memcpy(s_token, &buf[1], LOYALTY_TOKEN_LEN);
            s_claimed = true;
            APP_DBG_MSG("Loyalty: token loaded (claimed)\n");
        } else {
            APP_DBG_MSG("Loyalty: no token (magic=0x%02X != 0xA7) - unowned\n", buf[0]);
        }
    } else {
        APP_DBG_MSG("Loyalty: EEPROM read failed; assuming no token\n");
    }

    PowerMgmt_EEPROM_PowerOff();
}

bool Loyalty_IsClaimed(void)
{
    return s_claimed;
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
    if (token == NULL) {
        return false;
    }

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    bool    write_ok  = false;
    bool    verify_ok = false;
    uint8_t readback[EEPROM_LOYALTY_LEN];

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        APP_DBG_MSG("Loyalty: claim - m24cxx_init failed\n");
        PowerMgmt_EEPROM_PowerOff();
        return false;
    }

    /* Step 1: clear the "explicitly cleared" sentinel (anything other than
     * 0xCE). Otherwise Loyalty_Init would override the new claim on next
     * boot. Best-effort — if this fails the magic byte still wins, but we
     * log it loudly. */
    {
        uint8_t unset = 0xFF;
        for (int attempt = 0; attempt < 3; attempt++) {
            if (m24cxx_write(&s_eeprom, EEPROM_LOYALTY_CLEARED_ADDR, &unset, 1) != M24CXX_Ok) {
                APP_DBG_MSG("Loyalty_Claim: sentinel unset attempt %d - write failed\n", attempt);
                HAL_Delay(5);
                continue;
            }
            HAL_Delay(10);
            uint8_t rb = 0;
            if (m24cxx_read(&s_eeprom, EEPROM_LOYALTY_CLEARED_ADDR, &rb, 1) == M24CXX_Ok
                && rb != EEPROM_LOYALTY_CLEARED_VALUE) {
                APP_DBG_MSG("Loyalty_Claim: sentinel unset OK (0x%02X)\n", rb);
                break;
            }
            APP_DBG_MSG("Loyalty_Claim: sentinel unset readback still 0x%02X\n", rb);
            HAL_Delay(5);
        }
    }

    /* Step 2: write the magic byte + token. */
    uint8_t buf[EEPROM_LOYALTY_LEN] = {0};
    buf[0] = EEPROM_LOYALTY_MAGIC;
    memcpy(&buf[1], token, LOYALTY_TOKEN_LEN);
    buf[5] = 0x00;

    for (int attempt = 0; attempt < 3; attempt++) {
        if (m24cxx_write(&s_eeprom, EEPROM_LOYALTY_ADDR, buf, EEPROM_LOYALTY_LEN) != M24CXX_Ok) {
            APP_DBG_MSG("Loyalty: claim attempt %d - m24cxx_write returned non-Ok\n", attempt);
            HAL_Delay(5);
            continue;
        }
        write_ok = true;
        /* m24cxx_write already ACK-polls until the internal flash-commit
         * cycle completes; the extra 10 ms is belt-and-braces against
         * marginal supplies during the M24C08's ~5-10 ms flash cycle. */
        HAL_Delay(10);

        memset(readback, 0x00, sizeof(readback));
        if (m24cxx_read(&s_eeprom, EEPROM_LOYALTY_ADDR, readback, EEPROM_LOYALTY_LEN) != M24CXX_Ok) {
            APP_DBG_MSG("Loyalty: claim attempt %d - readback m24cxx_read failed\n", attempt);
            HAL_Delay(5);
            continue;
        }
        if (memcmp(readback, buf, EEPROM_LOYALTY_LEN) == 0) {
            verify_ok = true;
            APP_DBG_MSG("Loyalty_Claim: post-write EEPROM[0x%02X..]=%02X %02X %02X %02X %02X %02X\n",
                        EEPROM_LOYALTY_ADDR,
                        readback[0], readback[1], readback[2], readback[3], readback[4], readback[5]);
            break;
        }
        APP_DBG_MSG("Loyalty: claim attempt %d - readback mismatch\n", attempt);
        HAL_Delay(5);
    }

    PowerMgmt_EEPROM_PowerOff();

    if (verify_ok) {
        memcpy(s_token, token, LOYALTY_TOKEN_LEN);
        s_claimed = true;
        APP_DBG_MSG("Loyalty: claimed (EEPROM verified)\n");
        return true;
    }

    APP_DBG_MSG("Loyalty: CLAIM_FAILED - write_ok=%d verify_ok=%d after 3 attempts\n",
                (int)write_ok, (int)verify_ok);
    /* Do NOT update s_claimed/s_token on failure - the caller will REJECT
     * the iOS request, the user will retry, and we'll try again with a fresh
     * write. Better to fail loudly than to claim a token that won't survive
     * a reboot. */
    return false;
}

bool Loyalty_Wipe(void)
{
    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);  /* power-up settling */

    bool sentinel_ok = false;
    bool magic_ok    = false;

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        APP_DBG_MSG("Loyalty: wipe - m24cxx_init failed\n");
        PowerMgmt_EEPROM_PowerOff();
        memset(s_token, 0, sizeof(s_token));
        s_claimed = false;
        return false;
    }

    /* Layer A: write the "explicitly cleared" sentinel at the dedicated
     * address. This is the primary signal — even if the magic-byte wipe
     * fails, this sentinel makes the device functionally unclaimed on the
     * next boot. */
    {
        uint8_t sentinel = EEPROM_LOYALTY_CLEARED_VALUE;
        for (int attempt = 0; attempt < 3; attempt++) {
            if (m24cxx_write(&s_eeprom, EEPROM_LOYALTY_CLEARED_ADDR, &sentinel, 1) != M24CXX_Ok) {
                APP_DBG_MSG("Loyalty_Wipe: sentinel write attempt %d failed\n", attempt);
                HAL_Delay(5);
                continue;
            }
            HAL_Delay(10);
            uint8_t rb = 0;
            if (m24cxx_read(&s_eeprom, EEPROM_LOYALTY_CLEARED_ADDR, &rb, 1) == M24CXX_Ok
                && rb == EEPROM_LOYALTY_CLEARED_VALUE) {
                sentinel_ok = true;
                APP_DBG_MSG("Loyalty_Wipe: sentinel persisted (0x%02X @0x%02X)\n",
                            rb, EEPROM_LOYALTY_CLEARED_ADDR);
                break;
            }
            APP_DBG_MSG("Loyalty_Wipe: sentinel readback mismatch (got 0x%02X)\n", rb);
            HAL_Delay(5);
        }
    }

    /* Layer B: also wipe the legacy magic byte / token region to 0xFF. */
    {
        uint8_t wipe[EEPROM_LOYALTY_LEN];
        memset(wipe, 0xFF, sizeof(wipe));
        uint8_t readback[EEPROM_LOYALTY_LEN];

        for (int attempt = 0; attempt < 3; attempt++) {
            if (m24cxx_write(&s_eeprom, EEPROM_LOYALTY_ADDR, wipe, EEPROM_LOYALTY_LEN) != M24CXX_Ok) {
                APP_DBG_MSG("Loyalty_Wipe: magic-region attempt %d - write failed\n", attempt);
                HAL_Delay(5);
                continue;
            }
            HAL_Delay(10);
            memset(readback, 0, sizeof(readback));
            if (m24cxx_read(&s_eeprom, EEPROM_LOYALTY_ADDR, readback, EEPROM_LOYALTY_LEN) != M24CXX_Ok) {
                APP_DBG_MSG("Loyalty_Wipe: magic-region attempt %d - readback failed\n", attempt);
                HAL_Delay(5);
                continue;
            }
            if (memcmp(readback, wipe, EEPROM_LOYALTY_LEN) == 0) {
                magic_ok = true;
                APP_DBG_MSG("Loyalty_Wipe: magic region wiped: %02X %02X %02X %02X %02X %02X\n",
                            readback[0], readback[1], readback[2], readback[3], readback[4], readback[5]);
                break;
            }
            APP_DBG_MSG("Loyalty_Wipe: magic-region readback mismatch: %02X %02X %02X %02X %02X %02X\n",
                        readback[0], readback[1], readback[2], readback[3], readback[4], readback[5]);
            HAL_Delay(5);
        }
    }

    PowerMgmt_EEPROM_PowerOff();

    /* Always clear in-RAM state so the current session sees the device as
     * unclaimed even if EEPROM persistence is broken. The sentinel (Layer A)
     * handles persistence across reboots. */
    memset(s_token, 0, sizeof(s_token));
    s_claimed = false;

    APP_DBG_MSG("Loyalty_Wipe: sentinel_ok=%d magic_ok=%d\n",
                (int)sentinel_ok, (int)magic_ok);

    /* Success if at least the sentinel landed — that alone is enough to
     * mark the device unclaimed on next boot. */
    return sentinel_ok;
}
