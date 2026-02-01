/*
 * sound.h
 *
 *  Created on: Oct 29, 2025
 *      Author: sebkabob
 */

#ifndef INC_SOUND_H_
#define INC_SOUND_H_

#include <stdint.h>

extern TIM_HandleTypeDef htim2;

/* Note structure for sequences */
typedef struct {
    uint16_t frequency_hz;  // 0 = silence/rest
    uint16_t duration_ms;   // How long to play this note
    uint16_t delay_ms;      // Delay AFTER this note before next one
} Note_t;

/* Legacy blocking functions (for boot tones, etc) */
void BUZZER_Tone(uint32_t frequency_hz, uint32_t duration_ms);
void firstBootTone(void);
void SOUND_Disconnected(void);

/* Non-blocking buzzer API */
void BUZZER_PlaySequence(const Note_t* sequence, uint8_t num_notes, uint8_t loop);
void BUZZER_Update(void);  // Call in main loop
void BUZZER_Stop(void);
uint8_t BUZZER_IsPlaying(void);
uint32_t BUZZER_GetSequenceDuration(const Note_t* sequence, uint8_t num_notes);

/* Alarm sequences - start playing the alarm pattern */
void BUZZER_StartCalmAlarm(void);
void BUZZER_StartNormalAlarm(void);
void BUZZER_StartLoudAlarm(void);
uint32_t BUZZER_GetCalmAlarmDuration(void);
uint32_t BUZZER_GetNormalAlarmDuration(void);
uint32_t BUZZER_GetLoudAlarmDuration(void);

/* Fun melody */
void BUZZER_StartLaCucaracha(void);
uint32_t BUZZER_GetLaCucarachaDuration(void);

#endif /* INC_SOUND_H_ */
