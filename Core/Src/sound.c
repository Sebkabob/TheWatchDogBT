/***************************************************************************
 * sound.c
 * created by Sebastian Forenza 2026
 *
 * Functions in charge of interfacing with the onboard magnetic transducers
 *
 * V2 PCB BUZZER HARDWARE:
 *   PB0 = TIM16_CH1 alternate-function output (hardware PWM).
 *   Frequency is set by changing TIM16 ARR.
 *   50% duty cycle via CCR = (ARR+1)/2.
 *   Gate HIGH = N-channel MOSFET ON (current through buzzer).
 *
 *   TIM16 clock = 32 MHz HSE.
 *   With prescaler 31 the TIM16 tick is 1 MHz (1 µs).
 *   For frequency F: period = 1 000 000 / F, ARR = period - 1.
 ***************************************************************************/

#include "main.h"
#include "sound.h"
#include <stdint.h>
#include <string.h>

/***************************************************************************
 * PRIVATE DEFINES
 ***************************************************************************/
/* TIM16 input clock after PSC = 31 → 1 MHz */
#define BUZZER_TIMER_CLK  (1000000UL)

/***************************************************************************
 * EXTERN — TIM16 handle (declared in main.c, initialised by MX_TIM16_Init)
 ***************************************************************************/
extern TIM_HandleTypeDef htim16;

/***************************************************************************
 * ALARM PATTERN DEFINITIONS
 ***************************************************************************/

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

/* 3-tone ascending chirp × 3 repetitions — Apple "Find My" style ping */
static const Note_t FIND_MY_PATTERN[] = {
    /* Rep 1 */
    {987, 120, 50},
    {987, 120, 300},
    {987, 240, 50},
    {987, 120, 50},
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

/* Flag: is TIM16 currently generating a tone? */
static volatile uint8_t buzzer_tone_active = 0;

/***************************************************************************
 * PRIVATE: Start / stop the hardware PWM on PB0 via TIM16_CH1
 ***************************************************************************/

/**
 * @brief  Set buzzer frequency (0 = silence).
 *         Reconfigures TIM16 ARR and CCR for 50% duty HW PWM.
 */
static void BUZZER_SetFrequency(uint32_t frequency_hz)
{
    if (frequency_hz == 0) {
        /* Force output LOW (CCR=0 → always low in PWM1 mode), then stop */
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
        HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
        buzzer_tone_active = 0;
        return;
    }

    /* Full period in 1 µs ticks */
    uint32_t period = BUZZER_TIMER_CLK / frequency_hz;
    if (period < 2)  period = 2;
    if (period > 65535) period = 65535;

    __HAL_TIM_SET_AUTORELOAD(&htim16, period - 1);
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, period / 2);
    __HAL_TIM_SET_COUNTER(&htim16, 0);

    if (!buzzer_tone_active) {
        HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
        buzzer_tone_active = 1;
    }
}

/***************************************************************************
 * BUZZER_Init — call once in main() after MX_GPIO_Init / MX_TIM16_Init
 ***************************************************************************/
void BUZZER_Init(void)
{
    /* Ensure PWM is stopped (pin goes to idle state = LOW) */
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
    buzzer_tone_active = 0;
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

uint8_t BUZZER_IsToneActive(void)
{
    return buzzer_state.is_playing && !buzzer_state.in_delay;
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

void BUZZER_StartFindMe(void)
{
    BUZZER_PlaySequence(FIND_MY_PATTERN,
        sizeof(FIND_MY_PATTERN) / sizeof(Note_t), 0);
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
