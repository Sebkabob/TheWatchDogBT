/***************************************************************************
 * sound.c
 * created by Sebastian Forenza 2026
 *
 * Magnetic-buzzer driver. PB0 = TIM16_CH1 hardware PWM into an N-channel
 * MOSFET. ARR sets frequency; CCR = (ARR+1)/2 for 50 % duty.
 *
 * TIM16 input clock = 32 MHz HSE; with prescaler 31 the tick is 1 MHz, so
 * for tone frequency F: ARR = (1 000 000 / F) - 1.
 *
 * The pin is held LOW as a regular GPIO when idle and only switched to AF
 * during an active tone — keeps mains-frequency noise from coupling
 * through the gate when the buzzer should be silent.
 ***************************************************************************/

#include "main.h"
#include "sound.h"
#include "power_management.h"
#include "eeprom_map.h"      // EEPROM_I2C_ADDRESS + persistent record offsets
#include <stdint.h>
#include <string.h>

#define M24CXX_MODEL 0
#include "m24cxx.h"

#define BUZZER_TIMER_CLK  (1000000UL)   // TIM16 tick after PSC=31

extern TIM_HandleTypeDef htim16;

static const Note_t CALM_ALARM_PATTERN[] = {
    {415, 20, 15},
    {349, 20, 15},
};

// Eight-note alternating C6/A5 pattern that loops in exactly 1000 ms
// (8 × 125 ms = 110 ms tone + 15 ms gap per note). Locking the loop
// period to a clean 1 s cadence makes the alarm-duration countdown
// land on tone-boundary transitions rather than mid-note, which is
// what you want when the user picks "alarm stops 3 s after motion ends".
static const Note_t NORMAL_ALARM_PATTERN[] = {
    {1047, 110, 15},
    {880,  110, 15},
    {1047, 110, 15},
    {880,  110, 15},
    {1047, 110, 15},
    {880,  110, 15},
    {1047, 110, 15},
    {880,  110, 15},
};

// Apple "Find My" style ping — 3-tone ascending chirp.
static const Note_t FIND_MY_PATTERN[] = {
    {987, 120, 50},
    {987, 120, 300},
    {987, 240, 50},
    {987, 120, 50},
};

/***************************************************************************
 * SUPER_LOUD_ALARM_PATTERN — ear-piercing warble in the buzzer's resonant
 *   band. The magnetic transducer hits peak SPL near 4 kHz; both tones
 *   sit within ±200 Hz of that peak so neither half of the cycle drops
 *   below the loudest output. Alternating every 75 ms (~6.7 Hz warble)
 *   produces a psychoacoustically rough wail that's far harder to tune
 *   out than a steady 4 kHz tone at the same SPL — the auditory system
 *   keeps re-orienting to the changing pitch instead of habituating.
 *   delay_ms = 0 keeps the cycle continuous; the few-ms silence inherent
 *   in the BUZZER_Update note-to-note transition is below perception.
 ***************************************************************************/
static const Note_t SUPER_LOUD_ALARM_PATTERN[] = {
    {4200, 75, 0},
    {3800, 75, 0},
};

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
static volatile uint8_t buzzer_tone_active = 0;

// PB0 idle = GPIO LOW (MOSFET gate hard-off, no leakage).
static void BUZZER_PinClampLow(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = BUZZ_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, BUZZ_Pin, GPIO_PIN_RESET);
}

// PB0 active = AF2 (TIM16_CH1) for hardware PWM.
static void BUZZER_PinEnableAF(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = BUZZ_Pin;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM16;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/***************************************************************************
 * BUZZER_SetFrequency — retune TIM16 (frequency_hz = 0 silences the buzzer)
 *   When AlarmDisabled_Get() is set, every non-zero request is forced to
 *   zero. This is the single chokepoint to TIM16, so the gate here makes
 *   it physically impossible for any caller — alarm, find-my, drain mode,
 *   one-shot tones — to drive the buzzer while the flag is on.
 ***************************************************************************/
static void BUZZER_SetFrequency(uint32_t frequency_hz)
{
    if (frequency_hz != 0 && AlarmDisabled_Get()) {
        frequency_hz = 0;
    }

    if (frequency_hz == 0) {
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
        HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
        BUZZER_PinClampLow();
        buzzer_tone_active = 0;
        return;
    }

    uint32_t period = BUZZER_TIMER_CLK / frequency_hz;
    if (period < 2)  period = 2;
    if (period > 65535) period = 65535;

    __HAL_TIM_SET_AUTORELOAD(&htim16, period - 1);
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, period / 2);
    __HAL_TIM_SET_COUNTER(&htim16, 0);

    if (!buzzer_tone_active) {
        BUZZER_PinEnableAF();
        HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
        buzzer_tone_active = 1;
    }
}

/***************************************************************************
 * BUZZER_Init — call once after MX_GPIO_Init() / MX_TIM16_Init()
 *   Stops PWM and clamps the gate LOW so no current flows until a tone.
 ***************************************************************************/
void BUZZER_Init(void)
{
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
    BUZZER_PinClampLow();
    buzzer_tone_active = 0;
}

/***************************************************************************
 * BUZZER_PlaySequence — start a non-blocking note sequence
 *   loop=1 restarts at index 0 forever; loop=0 stops after the last note.
 ***************************************************************************/
void BUZZER_PlaySequence(const Note_t* sequence, uint8_t num_notes, uint8_t loop)
{
    if (sequence == NULL || num_notes == 0) return;

    // Defence-in-depth: refuse to even arm the sequencer when disabled, so
    // BUZZER_Update never has a queue to tick through.
    if (AlarmDisabled_Get()) {
        BUZZER_Stop();
        return;
    }

    buzzer_state.sequence     = sequence;
    buzzer_state.num_notes    = num_notes;
    buzzer_state.current_note = 0;
    buzzer_state.loop         = loop;
    buzzer_state.is_playing   = 1;
    buzzer_state.in_delay     = 0;
    buzzer_state.state_start_time = HAL_GetTick();

    BUZZER_SetFrequency(sequence[0].frequency_hz);
}

/***************************************************************************
 * BUZZER_Update — drive the playing-sequence state machine. Call from main.
 ***************************************************************************/
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

void BUZZER_StartCalmAlarm(void)
{
    BUZZER_PlaySequence(CALM_ALARM_PATTERN,
        sizeof(CALM_ALARM_PATTERN) / sizeof(Note_t), 1);
}

void BUZZER_StartNormalAlarm(void)
{
    BUZZER_PlaySequence(NORMAL_ALARM_PATTERN,
        sizeof(NORMAL_ALARM_PATTERN) / sizeof(Note_t), 1);
}

void BUZZER_StartSuperLoudAlarm(void)
{
    BUZZER_PlaySequence(SUPER_LOUD_ALARM_PATTERN,
        sizeof(SUPER_LOUD_ALARM_PATTERN) / sizeof(Note_t), 1);
}

void BUZZER_StartFindMe(void)
{
    BUZZER_PlaySequence(FIND_MY_PATTERN,
        sizeof(FIND_MY_PATTERN) / sizeof(Note_t), 0);
}

// Single-note "sequence" used by the continuous-tone driver (drain mode).
// The state machine loops it forever (loop=1) so the tone never breaks.
static Note_t continuous_tone_note = {0, 60000, 0};

void BUZZER_StartContinuousTone(uint16_t frequency_hz)
{
    if (frequency_hz == 0) {
        BUZZER_Stop();
        return;
    }
    continuous_tone_note.frequency_hz = frequency_hz;
    continuous_tone_note.duration_ms  = 60000;
    continuous_tone_note.delay_ms     = 0;
    BUZZER_PlaySequence(&continuous_tone_note, 1, 1);
}

/***************************************************************************
 * Blocking helpers — boot tones / connection chimes
 ***************************************************************************/

void BUZZER_Tone(uint32_t frequency_hz, uint32_t duration_ms)
{
    if (frequency_hz == 0 || duration_ms == 0) return;
    if (AlarmDisabled_Get()) return;

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
    if (DisconnectSoundDisabled_Get()) return;

    BUZZER_Tone(380, 10);
    HAL_Delay(10);
    BUZZER_Tone(280, 12);
    HAL_Delay(10);
    BUZZER_Tone(100, 15);
}

/***************************************************************************
 * Persisted alarm settings — all EEPROM-backed.
 *   alarm_duration_seconds     (0..30, default 10)
 *   alarm_disabled             (default false)
 *   disconnect_sound_disabled  (default false)
 ***************************************************************************/

extern I2C_HandleTypeDef hi2c1;

static M24CXX_HandleTypeDef s_sound_eeprom;
static uint8_t s_alarm_duration_s         = ALARM_DURATION_DEFAULT_S;
static bool    s_alarm_disabled           = false;
static bool    s_disconnect_sound_disabled = false;

static uint8_t alarm_duration_clamp(uint8_t v)
{
    return (v > ALARM_DURATION_MAX_S) ? ALARM_DURATION_MAX_S : v;
}

void AlarmDuration_Init(void)
{
    s_alarm_duration_s = ALARM_DURATION_DEFAULT_S;

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_sound_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        PowerMgmt_EEPROM_PowerOff();
        return;
    }

    uint8_t buf[EEPROM_ALARM_DURATION_LEN] = {0};
    if (m24cxx_read(&s_sound_eeprom, EEPROM_ALARM_DURATION_ADDR, buf,
                    EEPROM_ALARM_DURATION_LEN) == M24CXX_Ok) {
        if (buf[0] == EEPROM_ALARM_DURATION_MAGIC) {
            s_alarm_duration_s = alarm_duration_clamp(buf[1]);
        } else {
            uint8_t fresh[EEPROM_ALARM_DURATION_LEN];
            fresh[0] = EEPROM_ALARM_DURATION_MAGIC;
            fresh[1] = ALARM_DURATION_DEFAULT_S;
            (void)m24cxx_write(&s_sound_eeprom, EEPROM_ALARM_DURATION_ADDR,
                               fresh, EEPROM_ALARM_DURATION_LEN);
        }
    }

    PowerMgmt_EEPROM_PowerOff();
}

uint8_t AlarmDuration_Get(void)
{
    return s_alarm_duration_s;
}

uint8_t AlarmDuration_Set(uint8_t seconds)
{
    uint8_t clamped = alarm_duration_clamp(seconds);
    if (clamped == s_alarm_duration_s) {
        return clamped;
    }

    s_alarm_duration_s = clamped;

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_sound_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
        uint8_t buf[EEPROM_ALARM_DURATION_LEN];
        buf[0] = EEPROM_ALARM_DURATION_MAGIC;
        buf[1] = clamped;
        (void)m24cxx_write(&s_sound_eeprom, EEPROM_ALARM_DURATION_ADDR, buf,
                           EEPROM_ALARM_DURATION_LEN);
    }

    PowerMgmt_EEPROM_PowerOff();
    return clamped;
}

void AlarmDisabled_Init(void)
{
    s_alarm_disabled = false;

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_sound_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        PowerMgmt_EEPROM_PowerOff();
        return;
    }

    uint8_t buf[EEPROM_ALARM_DISABLED_LEN] = {0};
    if (m24cxx_read(&s_sound_eeprom, EEPROM_ALARM_DISABLED_ADDR, buf,
                    EEPROM_ALARM_DISABLED_LEN) == M24CXX_Ok) {
        if (buf[0] == EEPROM_ALARM_DISABLED_MAGIC) {
            s_alarm_disabled = (buf[1] != 0);
        } else {
            uint8_t fresh[EEPROM_ALARM_DISABLED_LEN];
            fresh[0] = EEPROM_ALARM_DISABLED_MAGIC;
            fresh[1] = 0;
            (void)m24cxx_write(&s_sound_eeprom, EEPROM_ALARM_DISABLED_ADDR,
                               fresh, EEPROM_ALARM_DISABLED_LEN);
        }
    }

    PowerMgmt_EEPROM_PowerOff();
}

bool AlarmDisabled_Get(void)
{
    return s_alarm_disabled;
}

bool AlarmDisabled_Set(bool disabled)
{
    if (disabled == s_alarm_disabled) {
        return s_alarm_disabled;
    }

    s_alarm_disabled = disabled;

    // Update the cache before kicking the buzzer down so any racing
    // BUZZER_Update tick sees the new state.
    if (disabled) {
        BUZZER_Stop();
    }

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_sound_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
        uint8_t buf[EEPROM_ALARM_DISABLED_LEN];
        buf[0] = EEPROM_ALARM_DISABLED_MAGIC;
        buf[1] = disabled ? 1 : 0;
        (void)m24cxx_write(&s_sound_eeprom, EEPROM_ALARM_DISABLED_ADDR, buf,
                           EEPROM_ALARM_DISABLED_LEN);
    }

    PowerMgmt_EEPROM_PowerOff();
    return s_alarm_disabled;
}

void DisconnectSoundDisabled_Init(void)
{
    s_disconnect_sound_disabled = false;

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_sound_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        PowerMgmt_EEPROM_PowerOff();
        return;
    }

    uint8_t buf[EEPROM_DISCONNECT_SOUND_DISABLED_LEN] = {0};
    if (m24cxx_read(&s_sound_eeprom, EEPROM_DISCONNECT_SOUND_DISABLED_ADDR, buf,
                    EEPROM_DISCONNECT_SOUND_DISABLED_LEN) == M24CXX_Ok) {
        if (buf[0] == EEPROM_DISCONNECT_SOUND_DISABLED_MAGIC) {
            s_disconnect_sound_disabled = (buf[1] != 0);
        } else {
            uint8_t fresh[EEPROM_DISCONNECT_SOUND_DISABLED_LEN];
            fresh[0] = EEPROM_DISCONNECT_SOUND_DISABLED_MAGIC;
            fresh[1] = 0;
            (void)m24cxx_write(&s_sound_eeprom,
                               EEPROM_DISCONNECT_SOUND_DISABLED_ADDR,
                               fresh, EEPROM_DISCONNECT_SOUND_DISABLED_LEN);
        }
    }

    PowerMgmt_EEPROM_PowerOff();
}

bool DisconnectSoundDisabled_Get(void)
{
    return s_disconnect_sound_disabled;
}

bool DisconnectSoundDisabled_Set(bool disabled)
{
    if (disabled == s_disconnect_sound_disabled) {
        return s_disconnect_sound_disabled;
    }

    s_disconnect_sound_disabled = disabled;

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_sound_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
        uint8_t buf[EEPROM_DISCONNECT_SOUND_DISABLED_LEN];
        buf[0] = EEPROM_DISCONNECT_SOUND_DISABLED_MAGIC;
        buf[1] = disabled ? 1 : 0;
        (void)m24cxx_write(&s_sound_eeprom,
                           EEPROM_DISCONNECT_SOUND_DISABLED_ADDR, buf,
                           EEPROM_DISCONNECT_SOUND_DISABLED_LEN);
    }

    PowerMgmt_EEPROM_PowerOff();
    return s_disconnect_sound_disabled;
}
