/***************************************************************************
 * loyalty.c
 * created by Sebastian Forenza 2026
 *
 * Application-layer loyalty token store. EEPROM-backed 4-byte ownership
 * marker; stands in for BLE pairing/bonding for the prototype.
 *
 * Record layout (single 6-byte record at 0x10..0x15):
 *   [0]    status: 0xA7=CLAIMED, 0xCE=CLEARED, anything else=BLANK
 *   [1..4] 4-byte token (only valid when CLAIMED + CRC matches)
 *   [5]    CRC8 over [0..4]
 *
 * EEPROM map (M24C08, 1024 B):
 *   0x00..0x06   BD address: magic(1) + addr(6)
 *   0x07..0x0F   reserved
 *   0x10..0x15   loyalty record (this file)
 *   0x16..0x17   reserved
 *   0x18..0x19   alarm-duration record (sound.{c,h})
 *   0x1A..0x1B   led-brightness record (lights.{c,h})
 *   0x1C..0x1D   alarm-disabled record (sound.{c,h})
 *   0x1E..0x20   device-settings record (state_machine.{c,h}) —
 *                deviceState (sans ARMED) + deviceInfo HIGH_PERF
 *   0x20..0x23   boot-count uint32 LE (power_management.{c,h})
 *   0x24..0x25   ble-tx-power record (power_management.{c,h}) —
 *                magic + NORMAL/HIGH enum
 *   0x26..0x3F   reserved
 *   0x40..0x47   motion-log header (motion_logger.h)
 *   0x48..0x3FF  motion-log event data
 ***************************************************************************/

#include "loyalty.h"
#include "lockservice_app.h"
#include "main.h"
#include "app_common.h"
#include "power_management.h"
#include "motion_logger.h"   // EEPROM_I2C_ADDRESS

#define M24CXX_MODEL 0
#include "m24cxx.h"

#include <string.h>

static M24CXX_HandleTypeDef s_eeprom;
extern I2C_HandleTypeDef    hi2c1;

static bool    s_claimed   = false;
static bool    s_unhealthy = false;
static uint8_t s_token[LOYALTY_TOKEN_LEN] = {0};

// Reset-window state. 0 = closed (LOCKED); non-zero = HAL_GetTick deadline at
// which the window expires (UNLOCKED). Compared with signed-tick arithmetic so
// the 49.7-day wraparound is benign.
static volatile uint32_t s_unlock_expiry_ms = 0;

// CRC-8/CCITT (poly 0x07, init 0x00). Byte-wise loop, no table.
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

// Build a 6-byte record from status + token (NULL token → token bytes 0xFF).
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

/***************************************************************************
 * loyalty_write_verify — write+readback with up to 3 retries
 *   Caller has already powered EEPROM and called m24cxx_init. m24cxx_write
 *   ACK-polls until the flash-commit cycle completes, so no extra delay.
 ***************************************************************************/
static bool loyalty_write_verify(uint16_t addr, const uint8_t *src, uint16_t len)
{
    uint8_t readback[EEPROM_LOYALTY_LEN];
    if (len > sizeof(readback)) {
        return false;
    }
    for (int attempt = 0; attempt < 3; attempt++) {
        if (m24cxx_write(&s_eeprom, addr, (uint8_t *)src, len) != M24CXX_Ok) {
            continue;
        }
        if (m24cxx_read(&s_eeprom, addr, readback, len) != M24CXX_Ok) {
            continue;
        }
        if (memcmp(readback, src, len) == 0) {
            return true;
        }
    }
    return false;
}

/***************************************************************************
 * Loyalty_Init — read EEPROM, set s_claimed/s_unhealthy, migrate legacy
 *   Legacy records (pre-CRC) had status=0xA7 with buf[5]=0x00. We rewrite
 *   them in place once with the correct CRC. If the rewrite fails the
 *   store goes UNHEALTHY (safer than accepting an in-RAM claim we couldn't
 *   persist).
 *
 *   If VBUS is already high at boot we open the reset window here so
 *   "factory pair" and "reset old device" both reduce to the same gesture
 *   (plug into USB-C, then tap Pair within 10 s).
 ***************************************************************************/
void Loyalty_Init(void)
{
    s_claimed         = false;
    s_unhealthy       = false;
    s_unlock_expiry_ms = 0;
    memset(s_token, 0, sizeof(s_token));

#if LOYALTY_WIPE_ON_BOOT
    (void)Loyalty_Wipe();
#endif

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        s_unhealthy = true;
        PowerMgmt_EEPROM_PowerOff();
        return;
    }

    uint8_t buf[EEPROM_LOYALTY_LEN] = {0};
    if (m24cxx_read(&s_eeprom, EEPROM_LOYALTY_ADDR, buf, EEPROM_LOYALTY_LEN) != M24CXX_Ok) {
        s_unhealthy = true;
        PowerMgmt_EEPROM_PowerOff();
        return;
    }

    if (buf[0] == EEPROM_LOYALTY_CLAIMED) {
        uint8_t expected = loyalty_crc8(buf, 5);
        if (expected == buf[5]) {
            memcpy(s_token, &buf[1], LOYALTY_TOKEN_LEN);
            s_claimed = true;
        } else {
            // Legacy pre-CRC record (status=0xA7, crc=0x00). Rewrite once
            // with the correct CRC; if the rewrite fails, mark UNHEALTHY
            // rather than accepting an in-RAM claim we couldn't persist.
            uint8_t migrated[EEPROM_LOYALTY_LEN];
            loyalty_pack(EEPROM_LOYALTY_CLAIMED, &buf[1], migrated);

            if (loyalty_write_verify(EEPROM_LOYALTY_ADDR, migrated, EEPROM_LOYALTY_LEN)) {
                memcpy(s_token, &buf[1], LOYALTY_TOKEN_LEN);
                s_claimed = true;
            } else {
                s_unhealthy = true;
            }
        }
    }

    PowerMgmt_EEPROM_PowerOff();

    if (IS_CABLE_PLUGGED()) {
        Loyalty_StartResetWindow();
    }
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

/***************************************************************************
 * Loyalty_Claim — persist a fresh token and mark the device claimed
 ***************************************************************************/
bool Loyalty_Claim(const uint8_t *token)
{
    if (token == NULL || s_unhealthy) {
        return false;
    }

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        PowerMgmt_EEPROM_PowerOff();
        return false;
    }

    uint8_t record[EEPROM_LOYALTY_LEN];
    loyalty_pack(EEPROM_LOYALTY_CLAIMED, token, record);

    bool ok = loyalty_write_verify(EEPROM_LOYALTY_ADDR, record, EEPROM_LOYALTY_LEN);
    PowerMgmt_EEPROM_PowerOff();

    if (!ok) {
        return false;
    }

    memcpy(s_token, token, LOYALTY_TOKEN_LEN);
    s_claimed = true;
    return true;
}

/***************************************************************************
 * Loyalty_Wipe — clear the EEPROM record (UNBOND or recovery hatch)
 *   Always clears in-RAM state, even if EEPROM persistence failed, so the
 *   current session sees the device as unclaimed.
 ***************************************************************************/
bool Loyalty_Wipe(void)
{
    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        PowerMgmt_EEPROM_PowerOff();
        memset(s_token, 0, sizeof(s_token));
        s_claimed = false;
        return false;
    }

    uint8_t record[EEPROM_LOYALTY_LEN];
    loyalty_pack(EEPROM_LOYALTY_CLEARED, NULL, record);

    bool ok = loyalty_write_verify(EEPROM_LOYALTY_ADDR, record, EEPROM_LOYALTY_LEN);
    PowerMgmt_EEPROM_PowerOff();

    memset(s_token, 0, sizeof(s_token));
    s_claimed = false;

    return ok;
}

/***************************************************************************
 * Loyalty_StartResetWindow — open the USB-C-gated CLAIM-overwrite window
 *   Caller is the cable-plug edge handler (or Loyalty_Init at boot if VBUS
 *   is already high). Re-arming an already-open window is intentional for
 *   the boot path; the cable-plug handler debounces edges so legitimate
 *   re-arms only happen on true unplug→replug.
 ***************************************************************************/
void Loyalty_StartResetWindow(void)
{
    uint32_t deadline = HAL_GetTick() + LOYALTY_RESET_WINDOW_MS;
    // 0 is reserved for "closed". On the astronomically unlikely tick that
    // wraps to exactly 0 here, nudge by 1 ms so IsOpen still treats us as
    // open.
    if (deadline == 0) deadline = 1;
    s_unlock_expiry_ms = deadline;
}

void Loyalty_CancelResetWindow(void)
{
    s_unlock_expiry_ms = 0;
}

bool Loyalty_IsResetWindowOpen(void)
{
    uint32_t expiry = s_unlock_expiry_ms;
    if (expiry == 0) {
        return false;
    }
    if ((int32_t)(expiry - HAL_GetTick()) > 0) {
        return true;
    }
    s_unlock_expiry_ms = 0;
    return false;
}
