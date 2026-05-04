/***************************************************************************
 * alarm_duration.c
 * created by Sebastian Forenza 2026
 *
 * EEPROM-backed alarm post-motion duration (0..30 s). Cached in RAM after
 * Init so the state machine reads it without I2C traffic. A blank EEPROM
 * (wrong magic) is treated as a factory reset and seeded with the default.
 ***************************************************************************/

#include "alarm_duration.h"
#include "main.h"
#include "power_management.h"
#include "motion_logger.h"   // EEPROM_I2C_ADDRESS

#define M24CXX_MODEL 0
#include "m24cxx.h"

static M24CXX_HandleTypeDef s_eeprom;
extern I2C_HandleTypeDef    hi2c1;

static uint8_t s_duration_s = ALARM_DURATION_DEFAULT_S;

static uint8_t clamp_duration(uint8_t v)
{
    return (v > ALARM_DURATION_MAX_S) ? ALARM_DURATION_MAX_S : v;
}

void AlarmDuration_Init(void)
{
    s_duration_s = ALARM_DURATION_DEFAULT_S;

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        PowerMgmt_EEPROM_PowerOff();
        return;
    }

    uint8_t buf[EEPROM_ALARM_DURATION_LEN] = {0};
    if (m24cxx_read(&s_eeprom, EEPROM_ALARM_DURATION_ADDR, buf,
                    EEPROM_ALARM_DURATION_LEN) == M24CXX_Ok) {
        if (buf[0] == EEPROM_ALARM_DURATION_MAGIC) {
            s_duration_s = clamp_duration(buf[1]);
        } else {
            // Blank EEPROM or wrong magic — seed the default so subsequent
            // reads return a valid record. Failure here is non-fatal: we
            // already have the default in RAM.
            uint8_t fresh[EEPROM_ALARM_DURATION_LEN];
            fresh[0] = EEPROM_ALARM_DURATION_MAGIC;
            fresh[1] = ALARM_DURATION_DEFAULT_S;
            (void)m24cxx_write(&s_eeprom, EEPROM_ALARM_DURATION_ADDR, fresh,
                               EEPROM_ALARM_DURATION_LEN);
        }
    }

    PowerMgmt_EEPROM_PowerOff();
}

uint8_t AlarmDuration_Get(void)
{
    return s_duration_s;
}

uint8_t AlarmDuration_Set(uint8_t seconds)
{
    uint8_t clamped = clamp_duration(seconds);
    if (clamped == s_duration_s) {
        return clamped;
    }

    s_duration_s = clamped;

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
        uint8_t buf[EEPROM_ALARM_DURATION_LEN];
        buf[0] = EEPROM_ALARM_DURATION_MAGIC;
        buf[1] = clamped;
        (void)m24cxx_write(&s_eeprom, EEPROM_ALARM_DURATION_ADDR, buf,
                           EEPROM_ALARM_DURATION_LEN);
    }

    PowerMgmt_EEPROM_PowerOff();
    return clamped;
}
