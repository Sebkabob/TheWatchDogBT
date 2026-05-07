/***************************************************************************
 * sound.h
 * created by Sebastian Forenza 2026
 *
 * Magnetic-buzzer API. PB0 = TIM16_CH1 hardware PWM (frequency via ARR,
 * 50 % duty via CCR = ARR/2). TIM2 is reserved for LEDs.
 ***************************************************************************/

#ifndef INC_SOUND_H_
#define INC_SOUND_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t frequency_hz;   // 0 = silent rest
    uint16_t duration_ms;
    uint16_t delay_ms;       // gap AFTER this note before the next
} Note_t;

// Stop PWM and clamp PB0 LOW. Call once after MX_GPIO_Init / MX_TIM16_Init.
void BUZZER_Init(void);

// Blocking helpers — boot tones, connection chimes.
void BUZZER_Tone(uint32_t frequency_hz, uint32_t duration_ms);
void firstBootTone(void);
// Three-tone disconnect chime. Self-gates on DisconnectSoundDisabled_Get(),
// so callers can invoke unconditionally.
void SOUND_Disconnected(void);

// Non-blocking sequence playback.
void BUZZER_PlaySequence(const Note_t* sequence, uint8_t num_notes, uint8_t loop);
void BUZZER_Update(void);
void BUZZER_Stop(void);
uint8_t BUZZER_IsPlaying(void);
uint8_t BUZZER_IsToneActive(void);
uint32_t BUZZER_GetSequenceDuration(const Note_t* sequence, uint8_t num_notes);

void BUZZER_StartCalmAlarm(void);
void BUZZER_StartNormalAlarm(void);
uint32_t BUZZER_GetCalmAlarmDuration(void);
uint32_t BUZZER_GetNormalAlarmDuration(void);

void BUZZER_StartLaCucaracha(void);
uint32_t BUZZER_GetLaCucarachaDuration(void);

void BUZZER_StartFindMe(void);

// Single-frequency tone that loops forever until BUZZER_Stop() (drain mode).
void BUZZER_StartContinuousTone(uint16_t frequency_hz);

/***************************************************************************
 * Persisted alarm settings — both EEPROM-backed, both apply to the buzzer
 * (the alarm subsystem owned by this file).
 *
 *   alarm_duration_seconds (0..30, default 10) — how long the alarm keeps
 *     sounding after motion stops while ALARM_ACTIVE. Stored at EEPROM 0x18.
 *
 *   alarm_disabled (default false) — when true, BUZZER_SetFrequency forces
 *     every non-zero request to 0, so no caller (alarm, find-my, drain
 *     mode, one-shot tones) can drive TIM16. Stored at EEPROM 0x1C.
 *
 * Init for both is invoked once from main() after Loyalty_Init().
 ***************************************************************************/

#define ALARM_DURATION_DEFAULT_S    10u
#define ALARM_DURATION_MAX_S        30u
#define EEPROM_ALARM_DURATION_ADDR  0x18
#define EEPROM_ALARM_DURATION_LEN   2
#define EEPROM_ALARM_DURATION_MAGIC 0xC3

void    AlarmDuration_Init(void);
uint8_t AlarmDuration_Get(void);
// Clamps to [0, 30] and persists. Returns the stored value. Skips the
// EEPROM write when unchanged.
uint8_t AlarmDuration_Set(uint8_t seconds);

#define EEPROM_ALARM_DISABLED_ADDR  0x1C
#define EEPROM_ALARM_DISABLED_LEN   2
#define EEPROM_ALARM_DISABLED_MAGIC 0xC5

void AlarmDisabled_Init(void);
bool AlarmDisabled_Get(void);
// Persists if changed. When transitioning to true, calls BUZZER_Stop()
// before persisting so any in-flight tone is killed immediately.
bool AlarmDisabled_Set(bool disabled);

/***************************************************************************
 * disconnect_sound_disabled (default false) — when true, SOUND_Disconnected()
 *   becomes a no-op so the three-tone descending chime that fires on BLE
 *   disconnect is silent. Independent of alarm_disabled — the user may want
 *   the alarm audible but skip the chime when leaving the app. Stored at
 *   EEPROM 0x1E. Init runs once from main() after AlarmDisabled_Init().
 ***************************************************************************/

#define EEPROM_DISCONNECT_SOUND_DISABLED_ADDR  0x1E
#define EEPROM_DISCONNECT_SOUND_DISABLED_LEN   2
#define EEPROM_DISCONNECT_SOUND_DISABLED_MAGIC 0xC7

void DisconnectSoundDisabled_Init(void);
bool DisconnectSoundDisabled_Get(void);
bool DisconnectSoundDisabled_Set(bool disabled);

#endif /* INC_SOUND_H_ */
