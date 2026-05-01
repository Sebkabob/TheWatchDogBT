/***************************************************************************
 * lights.c
 * created by Sebastian Forenza 2026
 *
 * Onboard RGB LED control. All channels are driven by hardware PWM on TIM2.
 *
 *   LED1 (Red)   = PB3  → TIM2_CH4
 *   LED2 (Green) = PB2  → TIM2_CH3
 *   LED3 (Blue)  = PB7  → TIM2_CH2
 *
 * LEDs are active-LOW. PWM range is inverted: CCR=999 → fully OFF,
 * CCR=0 → fully ON. Animation routines (Rainbow/Armed/Alarm/Pulse) are
 * non-blocking and rely on a 500 ms idle timeout to reset their internal
 * state when the caller stops invoking them.
 ***************************************************************************/

#include "main.h"
#include "lights.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

extern TIM_HandleTypeDef htim2;

// Set by LED_Pulse_Reset(); LED_Pulse clears it on next call.
static volatile uint8_t pulse_force_reset = 0;

static void StartRedPWM(void)   { HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); }
static void StartGreenPWM(void) { HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); }
static void StartBluePWM(void)  { HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); }

static void StopRedPWM(void)    { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 999); HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); }
static void StopGreenPWM(void)  { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 999); HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3); }
static void StopBluePWM(void)   { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 999); HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); }

static inline void SetRed(uint32_t pwm)   { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm); }
static inline void SetGreen(uint32_t pwm) { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm); }
static inline void SetBlue(uint32_t pwm)  { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm); }

/***************************************************************************
 * LED_Rainbow — non-blocking color-cycle animation
 *   Cycles 6 hue phases with linear ramping. State auto-resets after a
 *   500 ms idle window so back-to-back calls from different states don't
 *   stitch onto a half-finished animation.
 ***************************************************************************/
void LED_Rainbow(int ms_delay, uint8_t intensity)
{
    static uint8_t color_phase = 0;
    static uint8_t fade_step = 0;
    static uint32_t last_update = 0;
    static uint32_t last_call = 0;

    uint32_t current_time = HAL_GetTick();

    #define RAINBOW_RESET_TIMEOUT 500
    if ((current_time - last_call) > RAINBOW_RESET_TIMEOUT) {
        color_phase = 0;
        fade_step = 0;
        last_update = current_time;
    }
    last_call = current_time;

    StartRedPWM();
    StartGreenPWM();
    StartBluePWM();

    if ((current_time - last_update) < (uint32_t)ms_delay) {
        return;
    }
    last_update = current_time;

    uint16_t red = 0, green = 0, blue = 0;

    switch(color_phase) {
        case 0: red = 255; green = fade_step; blue = 0; break;
        case 1: red = 255 - fade_step; green = 255; blue = 0; break;
        case 2: red = 0; green = 255; blue = fade_step; break;
        case 3: red = 0; green = 255 - fade_step; blue = 255; break;
        case 4: red = fade_step; green = 0; blue = 255; break;
        case 5: red = 255; green = 0; blue = 255 - fade_step; break;
    }

    red   = (red   * intensity) / 255;
    green = (green * intensity) / 255;
    blue  = (blue  * intensity) / 255;

    // Convert RGB 0..255 to active-low PWM 0..999
    SetRed(  999 - ((red   * 999) / 255));
    SetGreen(999 - ((green * 999) / 255));
    SetBlue( 999 - ((blue  * 999) / 255));

    if (fade_step < 250) {
        fade_step += 5;
    } else {
        fade_step = 0;
        color_phase = (color_phase + 1) % 6;
    }
}

/***************************************************************************
 * LED_Armed — slow red breathing pulse for armed/locked states
 ***************************************************************************/
void LED_Armed(int ms_delay, uint8_t intensity)
{
    StartRedPWM();
    StopGreenPWM();
    StopBluePWM();

    static uint16_t pulse_value = 0;
    static uint8_t pulse_direction = 1;
    static uint32_t last_update = 0;
    static uint32_t last_call = 0;

    uint32_t current_time = HAL_GetTick();

    #define ARMED_RESET_TIMEOUT 500
    if ((current_time - last_call) > ARMED_RESET_TIMEOUT) {
        pulse_value = 0;
        pulse_direction = 1;
        last_update = current_time;
    }
    last_call = current_time;

    if ((current_time - last_update) < (uint32_t)ms_delay) {
        return;
    }
    last_update = current_time;

    if (pulse_direction) {
        pulse_value += 3;
        if (pulse_value >= 255) { pulse_value = 255; pulse_direction = 0; }
    } else {
        if (pulse_value >= 3) pulse_value -= 3; else pulse_value = 0;
        if (pulse_value == 0) pulse_direction = 1;
    }

    uint16_t scaled_brightness = (pulse_value * intensity) / 255;
    SetRed(999 - ((scaled_brightness * 999) / 255));
}

void LED_Off(void)
{
    SetRed(999);
    SetGreen(999);
    SetBlue(999);

    StopRedPWM();
    StopGreenPWM();
    StopBluePWM();
}

/***************************************************************************
 * LED_Solid — set a static color. No internal state; safe to call repeatedly.
 ***************************************************************************/
void LED_Solid(uint8_t r, uint8_t g, uint8_t b, uint8_t intensity)
{
    if (r > 0) { StartRedPWM(); }   else { StopRedPWM(); }
    if (g > 0) { StartGreenPWM(); } else { StopGreenPWM(); }
    if (b > 0) { StartBluePWM(); }  else { StopBluePWM(); }

    if (r > 0) { uint16_t sr = (r * intensity) / 255; SetRed(  999 - ((sr * 999) / 255)); }
    if (g > 0) { uint16_t sg = (g * intensity) / 255; SetGreen(999 - ((sg * 999) / 255)); }
    if (b > 0) { uint16_t sb = (b * intensity) / 255; SetBlue( 999 - ((sb * 999) / 255)); }
}

/***************************************************************************
 * LED_Alarm — flash custom color at <flash_interval_ms> cadence (non-blocking)
 *   Re-arms whenever the requested colour changes or after a 500 ms idle.
 ***************************************************************************/
void LED_Alarm(int flash_interval_ms, uint8_t red, uint8_t green, uint8_t blue, uint8_t intensity)
{
    static uint8_t led_state = 0;
    static uint32_t last_toggle_time = 0;
    static uint32_t last_call = 0;
    static uint8_t initialized_alarm = 0;
    static uint8_t last_r = 0, last_g = 0, last_b = 0;

    uint32_t current_time = HAL_GetTick();

    #define ALARM_RESET_TIMEOUT 500
    if ((current_time - last_call) > ALARM_RESET_TIMEOUT) {
        led_state = 0;
        last_toggle_time = current_time;
        initialized_alarm = 0;
        last_r = last_g = last_b = 0;
    }
    last_call = current_time;

    if (!initialized_alarm || last_r != red || last_g != green || last_b != blue) {
        if (red > 0)   { StartRedPWM(); }   else { StopRedPWM(); }
        if (green > 0) { StartGreenPWM(); } else { StopGreenPWM(); }
        if (blue > 0)  { StartBluePWM(); }  else { StopBluePWM(); }

        initialized_alarm = 1;
        last_toggle_time = current_time;
        led_state = 1;
        last_r = red; last_g = green; last_b = blue;
    }

    if ((current_time - last_toggle_time) >= (uint32_t)flash_interval_ms) {
        led_state = !led_state;
        last_toggle_time = current_time;
    }

    if (led_state) {
        if (red > 0)   { uint16_t sr = (red   * intensity) / 255; SetRed(  999 - ((sr * 999) / 255)); }
        if (green > 0) { uint16_t sg = (green * intensity) / 255; SetGreen(999 - ((sg * 999) / 255)); }
        if (blue > 0)  { uint16_t sb = (blue  * intensity) / 255; SetBlue( 999 - ((sb * 999) / 255)); }
    } else {
        if (red > 0)   SetRed(999);
        if (green > 0) SetGreen(999);
        if (blue > 0)  SetBlue(999);
    }
}

/***************************************************************************
 * LED_Pulse — soft breathing pulse over <duration> ms
 *   <duration> covers a full 0→255→0 sweep. LED_Pulse_Reset() forces the
 *   next call to start from dark even if PULSE_RESET_TIMEOUT hasn't elapsed.
 ***************************************************************************/
void LED_Pulse(int duration, uint8_t r, uint8_t g, uint8_t b, uint8_t intensity)
{
    static uint16_t pulse_value = 0;
    static uint8_t direction = 1;
    static uint32_t last_update = 0;
    static uint32_t last_call = 0;
    static uint8_t initialized_pulse = 0;
    static uint8_t last_r = 0, last_g = 0, last_b = 0;

    uint32_t now = HAL_GetTick();

    if (duration < 20) duration = 20;

    #define PULSE_RESET_TIMEOUT 500
    if (pulse_force_reset || (now - last_call) > PULSE_RESET_TIMEOUT) {
        pulse_value = 0;
        direction = 1;
        initialized_pulse = 0;
        last_update = now;
        last_r = last_g = last_b = 0;
        pulse_force_reset = 0;
    }
    last_call = now;

    if (!initialized_pulse || last_r != r || last_g != g || last_b != b) {
        if (r > 0) { StartRedPWM(); }   else { StopRedPWM(); }
        if (g > 0) { StartGreenPWM(); } else { StopGreenPWM(); }
        if (b > 0) { StartBluePWM(); }  else { StopBluePWM(); }

        initialized_pulse = 1;
        last_r = r; last_g = g; last_b = b;
    }

    uint16_t step_delay = duration / 512;
    if (step_delay < 1) step_delay = 1;

    if ((now - last_update) < step_delay) {
        return;
    }
    last_update = now;

    if (direction) {
        pulse_value++;
        if (pulse_value >= 255) { pulse_value = 255; direction = 0; }
    } else {
        if (pulse_value > 0) pulse_value--;
        if (pulse_value == 0) direction = 1;
    }

    if (r > 0) { uint16_t pr = (r * pulse_value * intensity) / (255 * 255); SetRed(  999 - ((pr * 999) / 255)); }
    if (g > 0) { uint16_t pg = (g * pulse_value * intensity) / (255 * 255); SetGreen(999 - ((pg * 999) / 255)); }
    if (b > 0) { uint16_t pb = (b * pulse_value * intensity) / (255 * 255); SetBlue( 999 - ((pb * 999) / 255)); }
}

void LED_Pulse_Reset(void)
{
    pulse_force_reset = 1;
}

/***************************************************************************
 * Cable plug-in transition
 *   Phase 1 (500 ms): fade current LED colour to black
 *   Phase 2 (250 ms): hold off
 *   Phase 3        : LED_Pulse_Reset() so the charging pulse starts dark
 ***************************************************************************/

#define PLUG_IN_FADE_MS  500u
#define PLUG_IN_GAP_MS   250u

typedef enum {
    PLUG_IN_IDLE = 0,
    PLUG_IN_FADE,
    PLUG_IN_GAP,
} PlugInPhase_t;

static PlugInPhase_t plugInPhase = PLUG_IN_IDLE;
static uint32_t plugInPhaseStart  = 0;
static uint16_t plugInStartCCR_R = 999;
static uint16_t plugInStartCCR_G = 999;
static uint16_t plugInStartCCR_B = 999;

void LED_PlugIn_Start(void)
{
    // Snapshot whatever the LEDs are showing right now. Channels that were
    // stopped have CCR=999 (StopXxxPWM sets it before stopping), so the fade
    // is a no-op for those — exactly what we want.
    plugInStartCCR_R = (uint16_t)htim2.Instance->CCR4;
    plugInStartCCR_G = (uint16_t)htim2.Instance->CCR3;
    plugInStartCCR_B = (uint16_t)htim2.Instance->CCR2;

    StartRedPWM();
    StartGreenPWM();
    StartBluePWM();

    SetRed(plugInStartCCR_R);
    SetGreen(plugInStartCCR_G);
    SetBlue(plugInStartCCR_B);

    plugInPhaseStart = HAL_GetTick();
    plugInPhase = PLUG_IN_FADE;
}

bool LED_PlugIn_InProgress(void)
{
    return plugInPhase != PLUG_IN_IDLE;
}

/***************************************************************************
 * LED_PlugIn_Tick — drive the plug-in fade/hold state machine (non-blocking)
 ***************************************************************************/
void LED_PlugIn_Tick(void)
{
    if (plugInPhase == PLUG_IN_IDLE) return;

    uint32_t elapsed = HAL_GetTick() - plugInPhaseStart;

    if (plugInPhase == PLUG_IN_FADE) {
        if (elapsed >= PLUG_IN_FADE_MS) {
            LED_Off();
            plugInPhase = PLUG_IN_GAP;
            plugInPhaseStart = HAL_GetTick();
            return;
        }

        uint32_t frac = (elapsed * 1000u) / PLUG_IN_FADE_MS;  // 0..999

        uint32_t r = plugInStartCCR_R + ((999u - plugInStartCCR_R) * frac) / 1000u;
        uint32_t g = plugInStartCCR_G + ((999u - plugInStartCCR_G) * frac) / 1000u;
        uint32_t b = plugInStartCCR_B + ((999u - plugInStartCCR_B) * frac) / 1000u;

        if (r > 999u) r = 999u;
        if (g > 999u) g = 999u;
        if (b > 999u) b = 999u;

        SetRed(r);
        SetGreen(g);
        SetBlue(b);
        return;
    }

    if (plugInPhase == PLUG_IN_GAP) {
        if (elapsed >= PLUG_IN_GAP_MS) {
            plugInPhase = PLUG_IN_IDLE;
            // Force the next LED_Pulse() to start from dark even if the pulse
            // was running less than PULSE_RESET_TIMEOUT ago.
            LED_Pulse_Reset();
            return;
        }
        return;
    }
}

/***************************************************************************
 * Cable unplug transition
 *   Phase 1 (100 ms): fade current LED colour to black
 *   Phase 2        : hand back to caller; natural LED routine resumes
 ***************************************************************************/

#define PLUG_OUT_FADE_MS 100u

static uint8_t  plugOutActive       = 0;
static uint32_t plugOutStartTick    = 0;
static uint16_t plugOutStartCCR_R   = 999;
static uint16_t plugOutStartCCR_G   = 999;
static uint16_t plugOutStartCCR_B   = 999;

void LED_PlugOut_Start(void)
{
    plugOutStartCCR_R = (uint16_t)htim2.Instance->CCR4;
    plugOutStartCCR_G = (uint16_t)htim2.Instance->CCR3;
    plugOutStartCCR_B = (uint16_t)htim2.Instance->CCR2;

    StartRedPWM();
    StartGreenPWM();
    StartBluePWM();

    SetRed(plugOutStartCCR_R);
    SetGreen(plugOutStartCCR_G);
    SetBlue(plugOutStartCCR_B);

    plugOutStartTick = HAL_GetTick();
    plugOutActive = 1;
}

bool LED_PlugOut_InProgress(void)
{
    return plugOutActive != 0;
}

void LED_PlugOut_Tick(void)
{
    if (!plugOutActive) return;

    uint32_t elapsed = HAL_GetTick() - plugOutStartTick;

    if (elapsed >= PLUG_OUT_FADE_MS) {
        LED_Off();
        plugOutActive = 0;
        return;
    }

    uint32_t frac = (elapsed * 1000u) / PLUG_OUT_FADE_MS;  // 0..999

    uint32_t r = plugOutStartCCR_R + ((999u - plugOutStartCCR_R) * frac) / 1000u;
    uint32_t g = plugOutStartCCR_G + ((999u - plugOutStartCCR_G) * frac) / 1000u;
    uint32_t b = plugOutStartCCR_B + ((999u - plugOutStartCCR_B) * frac) / 1000u;

    if (r > 999u) r = 999u;
    if (g > 999u) g = 999u;
    if (b > 999u) b = 999u;

    SetRed(r);
    SetGreen(g);
    SetBlue(b);
}
