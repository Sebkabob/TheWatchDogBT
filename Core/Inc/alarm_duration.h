/***************************************************************************
 * alarm_duration.h
 * created by Sebastian Forenza 2026
 *
 * Persisted alarm post-motion duration: how many seconds the alarm keeps
 * sounding after motion stops while ALARM_ACTIVE. Range 0..30 s, default 10.
 *
 * EEPROM record (2 bytes at 0x18, inside the reserved 0x16..0x3F region):
 *   [0]  magic = 0xC3
 *   [1]  duration in seconds (clamped to 0..30 on Init/Set)
 ***************************************************************************/

#ifndef ALARM_DURATION_H
#define ALARM_DURATION_H

#include <stdint.h>

#define ALARM_DURATION_DEFAULT_S    10u
#define ALARM_DURATION_MAX_S        30u

#define EEPROM_ALARM_DURATION_ADDR  0x18
#define EEPROM_ALARM_DURATION_LEN   2
#define EEPROM_ALARM_DURATION_MAGIC 0xC3

void    AlarmDuration_Init(void);
uint8_t AlarmDuration_Get(void);

// Clamps to [0, 30] and persists. Returns the clamped value actually stored.
// Skips the EEPROM write when the value is unchanged to avoid wear.
uint8_t AlarmDuration_Set(uint8_t seconds);

#endif /* ALARM_DURATION_H */
