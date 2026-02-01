/***************************************************************************
 * sound.c
 * created by Sebastian Forenza 2026
 *
 * Functions in charge of interfacing with the
 * onboard magnetic transducers
 ***************************************************************************/

#include "main.h"
#include "sound.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/***************************************************************************
 * ALARM PATTERN DEFINITIONS
 * Define your alarm sequences here - easy to modify!
 ***************************************************************************/

// Calm alarm - gentle beeps
static const Note_t CALM_ALARM_PATTERN[] = {
    {415, 20, 15},   // 415Hz for 20ms, then 15ms pause
    {349, 20, 15},   // 349Hz for 20ms, then 15ms pause
};

// Normal alarm - medium intensity
static const Note_t NORMAL_ALARM_PATTERN[] = {
    {1047, 300, 15},  // High C for 300ms
    {880,  300, 15},  // A for 300ms
    {1047, 300, 15},  // High C for 300ms
    {880,  300, 15},  // A for 300ms
};

// Loud alarm - high intensity
static const Note_t LOUD_ALARM_PATTERN[] = {
    {2186, 300, 15},  // Very high frequency
    {3520, 300, 15},  // Even higher
    {2186, 300, 15},  // Very high frequency
    {3520, 300, 15},  // Even higher
};

// La Cucaracha - fun test melody
// Tempo: 120 BPM (Quarter note = 500ms, Eighth note = 250ms)
static const Note_t LA_CUCARACHA_PATTERN[] = {
    // Phrase 1: "La cu-ca-ra-cha, la cu-ca-ra-cha"
    {523, 125, 25},   // C5 - eighth note (pickup)
    {523, 125, 25},   // C5 - eighth note
    {523, 125, 25},   // C5 - eighth note
    {698, 250, 25},   // F5 - quarter note
    {880, 250, 200},  // A5 - quarter note (slight pause before phrase 2)

    // Phrase 1: "La cu-ca-ra-cha, la cu-ca-ra-cha"
    {523, 125, 25},   // C5 - eighth note (pickup)
    {523, 125, 25},   // C5 - eighth note
    {523, 125, 25},   // C5 - eighth note
    {698, 250, 25},   // F5 - quarter note
    {880, 250, 200},  // A5 - quarter note (slight pause before phrase 2)

    // Phrase 2: "Ya no pue-de ca-mi-nar"
    {698, 125, 25},   // F5 - "Ya"
    {698, 125, 25},   // F5 - "no"
    {659, 125, 25},   // E5 - "pue"
    {659, 125, 25},   // E5 - "de"
    {587, 125, 25},   // D5 - "ca"
    {587, 125, 25},   // D5 - "mi"
    {523, 500, 500},  // C5 - "nar" (quarter note)
};

/***************************************************************************
 * BUZZER STATE MACHINE
 ***************************************************************************/

typedef struct {
    const Note_t* sequence;      // Pointer to current sequence
    uint8_t num_notes;           // Number of notes in sequence
    uint8_t current_note;        // Current note index
    uint8_t loop;                // 1 = loop, 0 = play once
    uint8_t is_playing;          // 1 = active, 0 = stopped
    uint8_t in_delay;            // 1 = in delay after note, 0 = playing note
    uint32_t state_start_time;   // When current state (note/delay) started
} BuzzerState_t;

static BuzzerState_t buzzer_state = {0};

/***************************************************************************
 * PRIVATE HELPER FUNCTIONS
 ***************************************************************************/

/**
 * @brief Set PWM frequency for buzzer (non-blocking)
 * @param frequency_hz Frequency in Hz, 0 = stop/silence
 */
static void BUZZER_SetFrequency(uint32_t frequency_hz) {
    if (frequency_hz == 0) {
        // Stop PWM for silence
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        return;
    }

    // Calculate period for this frequency
    uint32_t timer_clock = 64000000 / 64;  // 1MHz after prescaler
    uint32_t period = timer_clock / frequency_hz / 2;

    // Limit period to reasonable bounds
    if (period > 65535) period = 65535;
    if (period < 2) period = 2;

    // Stop the timer to reconfigure
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    // Set new period and 50% duty cycle
    __HAL_TIM_SET_AUTORELOAD(&htim2, period - 1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, period / 2);

    // Start PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

/***************************************************************************
 * PUBLIC NON-BLOCKING API
 ***************************************************************************/

/**
 * @brief Start playing a note sequence
 * @param sequence Pointer to array of Note_t structures
 * @param num_notes Number of notes in the sequence
 * @param loop 1 = loop forever, 0 = play once
 */
void BUZZER_PlaySequence(const Note_t* sequence, uint8_t num_notes, uint8_t loop) {
    if (sequence == NULL || num_notes == 0) {
        return;
    }

    buzzer_state.sequence = sequence;
    buzzer_state.num_notes = num_notes;
    buzzer_state.current_note = 0;
    buzzer_state.loop = loop;
    buzzer_state.is_playing = 1;
    buzzer_state.in_delay = 0;
    buzzer_state.state_start_time = HAL_GetTick();

    // Start playing first note
    BUZZER_SetFrequency(sequence[0].frequency_hz);
}

/**
 * @brief Update buzzer state machine - call this in main loop!
 */
void BUZZER_Update(void) {
    if (!buzzer_state.is_playing) {
        return;
    }

    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed = current_time - buzzer_state.state_start_time;
    const Note_t* current = &buzzer_state.sequence[buzzer_state.current_note];

    if (!buzzer_state.in_delay) {
        // Currently playing a note - check if duration expired
        if (elapsed >= current->duration_ms) {
            // Note finished, enter delay phase
            BUZZER_SetFrequency(0);  // Silence
            buzzer_state.in_delay = 1;
            buzzer_state.state_start_time = current_time;
        }
    } else {
        // Currently in delay after note - check if delay expired
        if (elapsed >= current->delay_ms) {
            // Delay finished, move to next note
            buzzer_state.current_note++;

            // Check if we've reached the end of sequence
            if (buzzer_state.current_note >= buzzer_state.num_notes) {
                if (buzzer_state.loop) {
                    // Loop back to start
                    buzzer_state.current_note = 0;
                } else {
                    // Sequence finished, stop
                    BUZZER_Stop();
                    return;
                }
            }

            // Start playing next note
            const Note_t* next = &buzzer_state.sequence[buzzer_state.current_note];
            BUZZER_SetFrequency(next->frequency_hz);
            buzzer_state.in_delay = 0;
            buzzer_state.state_start_time = current_time;
        }
    }
}

/**
 * @brief Stop buzzer immediately
 */
void BUZZER_Stop(void) {
    buzzer_state.is_playing = 0;
    BUZZER_SetFrequency(0);

    // Restore original timer period for LEDs (999)
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_SET_AUTORELOAD(&htim2, 999);
}

/**
 * @brief Check if buzzer is currently playing
 * @return 1 if playing, 0 if stopped
 */
uint8_t BUZZER_IsPlaying(void) {
    return buzzer_state.is_playing;
}

/**
 * @brief Calculate total duration of a note sequence
 * @param sequence Pointer to note array
 * @param num_notes Number of notes in sequence
 * @return Total duration in milliseconds
 */
uint32_t BUZZER_GetSequenceDuration(const Note_t* sequence, uint8_t num_notes) {
    uint32_t total_duration = 0;

    for (uint8_t i = 0; i < num_notes; i++) {
        total_duration += sequence[i].duration_ms;
        total_duration += sequence[i].delay_ms;
    }

    return total_duration;
}

/***************************************************************************
 * ALARM SEQUENCE STARTERS WITH DURATION GETTERS
 ***************************************************************************/

void BUZZER_StartCalmAlarm(void) {
    BUZZER_PlaySequence(CALM_ALARM_PATTERN,
                        sizeof(CALM_ALARM_PATTERN) / sizeof(Note_t),
                        1);  // Loop
}

uint32_t BUZZER_GetCalmAlarmDuration(void) {
    return BUZZER_GetSequenceDuration(CALM_ALARM_PATTERN,
                                      sizeof(CALM_ALARM_PATTERN) / sizeof(Note_t));
}

void BUZZER_StartNormalAlarm(void) {
    BUZZER_PlaySequence(NORMAL_ALARM_PATTERN,
                        sizeof(NORMAL_ALARM_PATTERN) / sizeof(Note_t),
                        1);  // Loop
}

uint32_t BUZZER_GetNormalAlarmDuration(void) {
    return BUZZER_GetSequenceDuration(NORMAL_ALARM_PATTERN,
                                      sizeof(NORMAL_ALARM_PATTERN) / sizeof(Note_t));
}

void BUZZER_StartLoudAlarm(void) {
    BUZZER_PlaySequence(LOUD_ALARM_PATTERN,
                        sizeof(LOUD_ALARM_PATTERN) / sizeof(Note_t),
                        1);  // Loop
}

uint32_t BUZZER_GetLoudAlarmDuration(void) {
    return BUZZER_GetSequenceDuration(LOUD_ALARM_PATTERN,
                                      sizeof(LOUD_ALARM_PATTERN) / sizeof(Note_t));
}

void BUZZER_StartLaCucaracha(void) {
    BUZZER_PlaySequence(LA_CUCARACHA_PATTERN,
                        sizeof(LA_CUCARACHA_PATTERN) / sizeof(Note_t),
                        1);  // Loop
}

uint32_t BUZZER_GetLaCucarachaDuration(void) {
    return BUZZER_GetSequenceDuration(LA_CUCARACHA_PATTERN,
                                      sizeof(LA_CUCARACHA_PATTERN) / sizeof(Note_t));
}

/***************************************************************************
 * LEGACY BLOCKING FUNCTIONS (kept for boot tones, etc)
 ***************************************************************************/

void BUZZER_Tone(uint32_t frequency_hz, uint32_t duration_ms) {
    if (frequency_hz == 0 || duration_ms == 0) return;

    // Calculate the period needed for this frequency
    uint32_t timer_clock = 64000000 / 64;  // 1MHz
    uint32_t period = timer_clock / frequency_hz / 2;

    // Limit period to reasonable bounds
    if (period > 65535) period = 65535;
    if (period < 2) period = 2;

    // Stop the timer to reconfigure
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    // Set new period for this frequency
    __HAL_TIM_SET_AUTORELOAD(&htim2, period - 1);

    // Set 50% duty cycle for square wave
    uint32_t pulse = period / 2;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);

    // Start PWM on buzzer channel
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // Play for specified duration (BLOCKING!)
    HAL_Delay(duration_ms);

    // Stop the buzzer
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    // Restore original period for LED (999)
    __HAL_TIM_SET_AUTORELOAD(&htim2, 999);
}

void firstBootTone(void) {
    BUZZER_Tone(160, 10);
    HAL_Delay(15);
    BUZZER_Tone(230, 20);
    HAL_Delay(15);
    BUZZER_Tone(292, 30);
}

void SOUND_Disconnected(void) {
    BUZZER_Tone(380, 10);
    HAL_Delay(10);
    BUZZER_Tone(280, 12);
    HAL_Delay(10);
    BUZZER_Tone(100, 15);
}
