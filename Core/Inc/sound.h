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

#endif /* INC_SOUND_H_ */
