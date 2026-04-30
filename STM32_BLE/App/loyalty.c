/**
 ******************************************************************************
 * @file    loyalty.c
 * @brief   Application-layer loyalty token store. EEPROM-backed 4-byte
 *          ownership marker; replaces BLE pairing/bonding for the prototype.
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

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
        uint8_t buf[EEPROM_LOYALTY_LEN] = {0};
        if (m24cxx_read(&s_eeprom, EEPROM_LOYALTY_ADDR, buf, EEPROM_LOYALTY_LEN) == M24CXX_Ok) {
            /* DIAGNOSTIC: log every byte so we can verify what's actually in
             * EEPROM across boots. If the unit is reportedly stuck "already
             * claimed" after an unbond, this line tells us whether the EEPROM
             * was wiped (magic=0xFF) or still has stale data (magic=0xA7). */
            APP_DBG_MSG("Loyalty: EEPROM[0x%02X..]=%02X %02X %02X %02X %02X %02X\n",
                        EEPROM_LOYALTY_ADDR,
                        buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

            if (buf[0] == EEPROM_LOYALTY_MAGIC) {
                memcpy(s_token, &buf[1], LOYALTY_TOKEN_LEN);
                s_claimed = true;
                APP_DBG_MSG("Loyalty: token loaded (claimed)\n");
            } else {
                s_claimed = false;
                APP_DBG_MSG("Loyalty: no token (magic=0x%02X != 0xA7) - unowned\n", buf[0]);
            }
        } else {
            APP_DBG_MSG("Loyalty: EEPROM read failed; assuming no token\n");
        }
    } else {
        APP_DBG_MSG("Loyalty: m24cxx_init failed\n");
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

    uint8_t buf[EEPROM_LOYALTY_LEN] = {0};
    buf[0] = EEPROM_LOYALTY_MAGIC;
    memcpy(&buf[1], token, LOYALTY_TOKEN_LEN);
    buf[5] = 0x00;

    bool    write_ok  = false;
    bool    verify_ok = false;
    uint8_t readback[EEPROM_LOYALTY_LEN];

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
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
    } else {
        APP_DBG_MSG("Loyalty: claim - m24cxx_init failed\n");
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

    uint8_t wipe[EEPROM_LOYALTY_LEN];
    memset(wipe, 0xFF, sizeof(wipe));

    bool    write_ok  = false;
    bool    verify_ok = false;
    uint8_t readback[EEPROM_LOYALTY_LEN];

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
        for (int attempt = 0; attempt < 3; attempt++) {
            if (m24cxx_write(&s_eeprom, EEPROM_LOYALTY_ADDR, wipe, EEPROM_LOYALTY_LEN) != M24CXX_Ok) {
                APP_DBG_MSG("Loyalty: wipe attempt %d - m24cxx_write returned non-Ok\n", attempt);
                HAL_Delay(5);
                continue;
            }
            write_ok = true;
            /* M24C08 internal flash-commit cycle is up to ~10 ms after the
             * I2C transaction completes. m24cxx_write already ACK-polls the
             * chip back to readiness, but we keep a small extra wait as a
             * safety margin against marginal VDD. */
            HAL_Delay(10);

            memset(readback, 0x00, sizeof(readback));
            if (m24cxx_read(&s_eeprom, EEPROM_LOYALTY_ADDR, readback, EEPROM_LOYALTY_LEN) != M24CXX_Ok) {
                APP_DBG_MSG("Loyalty: wipe attempt %d - readback m24cxx_read failed\n", attempt);
                HAL_Delay(5);
                continue;
            }
            if (memcmp(readback, wipe, EEPROM_LOYALTY_LEN) == 0) {
                verify_ok = true;
                APP_DBG_MSG("Loyalty_Wipe: post-wipe EEPROM[0x%02X..]=%02X %02X %02X %02X %02X %02X\n",
                            EEPROM_LOYALTY_ADDR,
                            readback[0], readback[1], readback[2], readback[3], readback[4], readback[5]);
                break;
            }
            APP_DBG_MSG("Loyalty: wipe attempt %d - readback mismatch (magic=0x%02X, expected 0xFF)\n",
                        attempt, readback[0]);
            HAL_Delay(5);
        }
    } else {
        APP_DBG_MSG("Loyalty: wipe - m24cxx_init failed\n");
    }

    PowerMgmt_EEPROM_PowerOff();

    /* Clear in-RAM state regardless of EEPROM persistence outcome. The user
     * just unbonded - they expect this device to be unowned for the rest of
     * this session. If EEPROM persistence failed, log loudly so the next
     * debug session catches it; the worst-case fallback is the cable-hold
     * reset hatch. */
    memset(s_token, 0, sizeof(s_token));
    s_claimed = false;

    if (verify_ok) {
        APP_DBG_MSG("Loyalty: wiped (EEPROM verified)\n");
        return true;
    } else if (write_ok) {
        APP_DBG_MSG("Loyalty: WIPE_PARTIAL - m24cxx_write OK but readback mismatch (3 attempts)\n");
        return false;
    } else {
        APP_DBG_MSG("Loyalty: WIPE_FAILED - m24cxx_write never succeeded (3 attempts)\n");
        return false;
    }
}
