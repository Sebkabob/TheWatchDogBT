/*
 * sound.h
 *
 *  Created on: Oct 29, 2025
 *      Author: sebkabob
 *
 *  Buzzer uses TIM16_CH1 hardware PWM on PB0.
 *  Frequency set via ARR, 50% duty via CCR = ARR/2.
 *  TIM2 is left exclusively for LED PWM (CH2, CH3, CH4).
 */

#ifndef INC_SOUND_H_
#define INC_SOUND_H_

#include <stdint.h>

/* Note structure for sequences */
typedef struct {
    uint16_t frequency_hz;  // 0 = silence/rest
    uint16_t duration_ms;   // How long to play this note
    uint16_t delay_ms;      // Delay AFTER this note before next one
} Note_t;

/**
 * @brief  Initialise the buzzer subsystem.
 *         Call once after MX_GPIO_Init() and MX_TIM16_Init() in main().
 */
void BUZZER_Init(void);

/* Legacy blocking functions (for boot tones, etc.) */
void BUZZER_Tone(uint32_t frequency_hz, uint32_t duration_ms);
void firstBootTone(void);
void SOUND_Disconnected(void);

/* Non-blocking buzzer API */
void BUZZER_PlaySequence(const Note_t* sequence, uint8_t num_notes, uint8_t loop);
void BUZZER_Update(void);  // Call in main loop
void BUZZER_Stop(void);
uint8_t BUZZER_IsPlaying(void);
uint8_t BUZZER_IsToneActive(void);
uint32_t BUZZER_GetSequenceDuration(const Note_t* sequence, uint8_t num_notes);

/* Alarm sequences */
void BUZZER_StartCalmAlarm(void);
void BUZZER_StartNormalAlarm(void);
uint32_t BUZZER_GetCalmAlarmDuration(void);
uint32_t BUZZER_GetNormalAlarmDuration(void);

/* Fun melody */
void BUZZER_StartLaCucaracha(void);
uint32_t BUZZER_GetLaCucarachaDuration(void);

/* Find My Device ping */
void BUZZER_StartFindMe(void);

/* Continuous single-frequency tone (used by drain mode). Loops indefinitely
 * until BUZZER_Stop() is called. */
void BUZZER_StartContinuousTone(uint16_t frequency_hz);

#endif /* INC_SOUND_H_ */
