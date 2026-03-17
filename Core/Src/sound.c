/***************************************************************************
 * sound.c
 * created by Sebastian Forenza 2026
 *
 * Functions in charge of interfacing with the
 * onboard magnetic transducers
 *
 * BUZZER HARDWARE:
 *   PB6 -> TIM2_CH1 (AF4) -> N-channel MOSFET gate
 *   Gate HIGH = MOSFET ON  (current flows through buzzer)
 *   Gate LOW  = MOSFET OFF (no current)
 *
 * CRITICAL BOOT SEQUENCE:
 *   MX_TIM2_Init() calls HAL_TIM_MspPostInit() which configures PB6
 *   as AF push-pull for TIM2_CH1.  At that moment the timer isn't
 *   counting yet, so the OC1 output sits at its idle level — which
 *   is HIGH for PWM1 + OCPOLARITY_HIGH.  This turns the N-channel
 *   MOSFET on immediately and sends DC through the buzzer.
 *
 *   FIX: Call BUZZER_Init() immediately after MX_TIM2_Init() in main().
 *   It forces CCR1=0 and starts CH1 so the output goes LOW before
 *   anything else happens.
 ***************************************************************************/

#include "main.h"
#include "sound.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/***************************************************************************
 * PRIVATE DEFINES
 ***************************************************************************/
#define LED_ARR  999
#define BUZZER_TIMER_CLK  (64000000UL / 64)  /* 1 MHz after PSC=63 */

/***************************************************************************
 * ALARM PATTERN DEFINITIONS
 ***************************************************************************/

static const Note_t DRAIN_PATTERN[] = {
    {20, 57600, 0},
};

static const Note_t CALM_ALARM_PATTERN[] = {
    {415, 20, 15},
    {349, 20, 15},
};

static const Note_t NORMAL_ALARM_PATTERN[] = {
    {1047, 150, 15},
    {880,  150, 15},
    {1047, 150, 15},
    {880,  150, 15},
    {1047, 150, 15},
    {880,  150, 15},
    {1047, 150, 15},
    {880,  150, 15},
};

static const Note_t LOUD_ALARM_PATTERN[] = {
    {2186, 300, 15},
    {3520, 300, 15},
    {2186, 300, 15},
    {3520, 300, 15},
};

static const Note_t LA_CUCARACHA_PATTERN[] = {
    {523, 125, 25},
    {523, 125, 25},
    {523, 125, 25},
    {698, 250, 25},
    {880, 250, 200},
    {523, 125, 25},
    {523, 125, 25},
    {523, 125, 25},
    {698, 250, 25},
    {880, 250, 200},
    {698, 125, 25},
    {698, 125, 25},
    {659, 125, 25},
    {659, 125, 25},
    {587, 125, 25},
    {587, 125, 25},
    {523, 500, 500},
};

/***************************************************************************
 * BUZZER STATE MACHINE
 ***************************************************************************/

typedef struct {
    const Note_t* sequence;
    uint8_t  num_notes;
    uint8_t  current_note;
    uint8_t  loop;
    uint8_t  is_playing;
    uint8_t  in_delay;
    uint32_t state_start_time;
} BuzzerState_t;

static BuzzerState_t buzzer_state = {0};
static uint8_t ch1_started = 0;

/***************************************************************************
 * BUZZER_Init — CALL IMMEDIATELY AFTER MX_TIM2_Init() IN main()
 ***************************************************************************/

/**
 * @brief  Safe-start the buzzer channel.
 *
 *         MX_TIM2_Init → HAL_TIM_MspPostInit puts PB6 in AF mode
 *         with the OC1 output idling HIGH (N-ch MOSFET on!).
 *
 *         This function forces CCR1=0 and starts CH1 + the timer
 *         base so the PWM output immediately goes LOW.
 *
 *         MUST be called right after MX_TIM2_Init() in main(),
 *         before any HAL_Delay or other code that takes time.
 */
void BUZZER_Init(void)
{
    /* Force compare value to 0 BEFORE starting the channel.
     * PWM1 + OCPOLARITY_HIGH: output HIGH while cnt < CCR.
     * CCR=0 means output is NEVER HIGH → always LOW → MOSFET OFF. */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

    /* Make sure the counter is at 0 */
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* Start the timer base (needed for ALL channels incl. LEDs) */
    HAL_TIM_Base_Start(&htim2);

    /* Start CH1 output — pin goes LOW immediately */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    ch1_started = 1;
}

/***************************************************************************
 * PRIVATE: Ensure CH1 is running
 ***************************************************************************/
static void BUZZER_EnsureCH1Started(void)
{
    if (!ch1_started) {
        BUZZER_Init();
    }
}

/***************************************************************************
 * PRIVATE: Set buzzer frequency
 ***************************************************************************/
static void BUZZER_SetFrequency(uint32_t frequency_hz)
{
    BUZZER_EnsureCH1Started();

    if (frequency_hz == 0) {
        /* Silence: CCR1=0 → output always LOW → MOSFET OFF */
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_AUTORELOAD(&htim2, LED_ARR);
        return;
    }

    uint32_t period = BUZZER_TIMER_CLK / frequency_hz;
    if (period > 65535) period = 65535;
    if (period < 2)     period = 2;

    uint32_t new_arr = period - 1;
    uint32_t old_arr = __HAL_TIM_GET_AUTORELOAD(&htim2);

    /* Rescale LED CCR values so brightness doesn't jump */
    if (old_arr > 0 && new_arr != old_arr) {
        uint32_t ccr3 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
        uint32_t ccr4 = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,
                              (ccr3 * new_arr) / old_arr);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,
                              (ccr4 * new_arr) / old_arr);
    }

    __HAL_TIM_SET_AUTORELOAD(&htim2, new_arr);

    /* 50% duty on CH1 → square wave on MOSFET gate */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, period / 2);
}

/***************************************************************************
 * PUBLIC NON-BLOCKING API
 ***************************************************************************/

void BUZZER_PlaySequence(const Note_t* sequence, uint8_t num_notes, uint8_t loop)
{
    if (sequence == NULL || num_notes == 0) return;

    buzzer_state.sequence     = sequence;
    buzzer_state.num_notes    = num_notes;
    buzzer_state.current_note = 0;
    buzzer_state.loop         = loop;
    buzzer_state.is_playing   = 1;
    buzzer_state.in_delay     = 0;
    buzzer_state.state_start_time = HAL_GetTick();

    BUZZER_SetFrequency(sequence[0].frequency_hz);
}

void BUZZER_Update(void)
{
    if (!buzzer_state.is_playing) return;

    uint32_t now     = HAL_GetTick();
    uint32_t elapsed = now - buzzer_state.state_start_time;
    const Note_t* cur = &buzzer_state.sequence[buzzer_state.current_note];

    if (!buzzer_state.in_delay) {
        if (elapsed >= cur->duration_ms) {
            BUZZER_SetFrequency(0);
            buzzer_state.in_delay = 1;
            buzzer_state.state_start_time = now;
        }
    } else {
        if (elapsed >= cur->delay_ms) {
            buzzer_state.current_note++;

            if (buzzer_state.current_note >= buzzer_state.num_notes) {
                if (buzzer_state.loop) {
                    buzzer_state.current_note = 0;
                } else {
                    BUZZER_Stop();
                    return;
                }
            }

            const Note_t* nxt = &buzzer_state.sequence[buzzer_state.current_note];
            BUZZER_SetFrequency(nxt->frequency_hz);
            buzzer_state.in_delay = 0;
            buzzer_state.state_start_time = now;
        }
    }
}

void BUZZER_Stop(void)
{
    buzzer_state.is_playing = 0;
    BUZZER_SetFrequency(0);
}

uint8_t BUZZER_IsPlaying(void)
{
    return buzzer_state.is_playing;
}

uint32_t BUZZER_GetSequenceDuration(const Note_t* sequence, uint8_t num_notes)
{
    uint32_t total = 0;
    for (uint8_t i = 0; i < num_notes; i++) {
        total += sequence[i].duration_ms;
        total += sequence[i].delay_ms;
    }
    return total;
}

/***************************************************************************
 * ALARM SEQUENCE STARTERS
 ***************************************************************************/

void BUZZER_Drain(void)
{
    BUZZER_PlaySequence(DRAIN_PATTERN,
        sizeof(DRAIN_PATTERN) / sizeof(Note_t), 1);
}

void BUZZER_StartCalmAlarm(void)
{
    BUZZER_PlaySequence(CALM_ALARM_PATTERN,
        sizeof(CALM_ALARM_PATTERN) / sizeof(Note_t), 1);
}

uint32_t BUZZER_GetCalmAlarmDuration(void)
{
    return BUZZER_GetSequenceDuration(CALM_ALARM_PATTERN,
        sizeof(CALM_ALARM_PATTERN) / sizeof(Note_t));
}

void BUZZER_StartNormalAlarm(void)
{
    BUZZER_PlaySequence(NORMAL_ALARM_PATTERN,
        sizeof(NORMAL_ALARM_PATTERN) / sizeof(Note_t), 1);
}

uint32_t BUZZER_GetNormalAlarmDuration(void)
{
    return BUZZER_GetSequenceDuration(NORMAL_ALARM_PATTERN,
        sizeof(NORMAL_ALARM_PATTERN) / sizeof(Note_t));
}

void BUZZER_StartLoudAlarm(void)
{
    BUZZER_PlaySequence(LOUD_ALARM_PATTERN,
        sizeof(LOUD_ALARM_PATTERN) / sizeof(Note_t), 1);
}

uint32_t BUZZER_GetLoudAlarmDuration(void)
{
    return BUZZER_GetSequenceDuration(LOUD_ALARM_PATTERN,
        sizeof(LOUD_ALARM_PATTERN) / sizeof(Note_t));
}

void BUZZER_StartLaCucaracha(void)
{
    BUZZER_PlaySequence(LA_CUCARACHA_PATTERN,
        sizeof(LA_CUCARACHA_PATTERN) / sizeof(Note_t), 1);
}

uint32_t BUZZER_GetLaCucarachaDuration(void)
{
    return BUZZER_GetSequenceDuration(LA_CUCARACHA_PATTERN,
        sizeof(LA_CUCARACHA_PATTERN) / sizeof(Note_t));
}

/***************************************************************************
 * LEGACY BLOCKING FUNCTIONS (boot tones, etc.)
 ***************************************************************************/

void BUZZER_Tone(uint32_t frequency_hz, uint32_t duration_ms)
{
    if (frequency_hz == 0 || duration_ms == 0) return;

    BUZZER_SetFrequency(frequency_hz);
    HAL_Delay(duration_ms);
    BUZZER_SetFrequency(0);
}

void firstBootTone(void)
{
    BUZZER_Tone(160, 10);
    HAL_Delay(15);
    BUZZER_Tone(230, 20);
    HAL_Delay(15);
    BUZZER_Tone(292, 30);
}

void SOUND_Disconnected(void)
{
    BUZZER_Tone(380, 10);
    HAL_Delay(10);
    BUZZER_Tone(280, 12);
    HAL_Delay(10);
    BUZZER_Tone(100, 15);
}
