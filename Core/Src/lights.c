/***************************************************************************
 * lights.c
 * created by Sebastian Forenza 2026
 *
 * Functions in charge of interfacing with the onboard LEDs
 *
 * NEW PCB pin mapping:
 *   LED1 (Red)   = PB3  -> TIM2_CH4  (hardware PWM, active LOW)
 *   LED2 (Green) = PB2  -> TIM2_CH3  (hardware PWM, active LOW)
 *   LED3 (Blue)  = PB1  -> GPIO soft-PWM via SysTick (active LOW)
 *
 * All LEDs are active-LOW: driving the pin LOW turns the LED ON.
 * For hardware PWM: CCR = 999 -> fully OFF, CCR = 0 -> fully ON.
 * For software PWM: GPIO RESET (low) = ON, SET (high) = OFF.
 ***************************************************************************/

#include "main.h"
#include "lights.h"
#include "sound.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim2;

/* =========================================================================
 * SOFTWARE PWM FOR LED3 (BLUE) ON PB1
 *
 * A simple 8-bit resolution PWM driven from SysTick at 1 kHz.
 * PWM frequency ≈ 1000 / 256 ≈ ~3.9 Hz per full cycle BUT we only
 * need perceived brightness so we run a fast counter that wraps at
 * a smaller period.  Using period = 100 gives ~10 Hz flicker-free
 * at 1 kHz tick rate (100 ticks per cycle = 10 cycles/sec ... still
 * visible).  Better: period = 20 gives 50 Hz, good enough.
 *
 * We use a period of 20 ticks (50 Hz PWM).  Duty 0-20 maps from
 * the requested 0-999 range to keep the interface consistent with
 * the hardware PWM channels.
 * ========================================================================= */

#define SOFT_PWM_PERIOD  20   /* 1 kHz / 20 = 50 Hz PWM — flicker-free */

static volatile uint16_t soft_pwm_blue_duty = SOFT_PWM_PERIOD; /* 0=full ON, PERIOD=full OFF (active low) */
static volatile uint8_t  soft_pwm_counter   = 0;
static volatile uint8_t  soft_pwm_enabled   = 0;

/**
 * @brief Initialise soft-PWM (call once at startup)
 */
void LED_SoftPWM_Init(void)
{
    /* Make sure PB1 is push-pull output, driven HIGH (LED off) */
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    soft_pwm_blue_duty = SOFT_PWM_PERIOD;  /* OFF */
    soft_pwm_counter   = 0;
    soft_pwm_enabled   = 0;
}

/**
 * @brief Call from SysTick_Handler every 1 ms
 */
void LED_SoftPWM_Tick(void)
{
    if (!soft_pwm_enabled) {
        return;
    }

    soft_pwm_counter++;
    if (soft_pwm_counter >= SOFT_PWM_PERIOD) {
        soft_pwm_counter = 0;
    }

    /* Active-LOW: pin LOW = LED on.
     * duty == 0           -> always HIGH (off)
     * duty == SOFT_PWM_PERIOD -> always LOW  (full on)  ... wait, inverted.
     *
     * Let's define: soft_pwm_blue_duty = how many ticks out of PERIOD the LED is ON.
     *   0              = LED fully OFF
     *   SOFT_PWM_PERIOD = LED fully ON
     */
    if (soft_pwm_counter < soft_pwm_blue_duty) {
        /* LED ON (active low) */
        LED3_GPIO_Port->BRR = LED3_Pin;   /* fast reset = LOW = ON */
    } else {
        /* LED OFF */
        LED3_GPIO_Port->BSRR = LED3_Pin;  /* fast set = HIGH = OFF */
    }
}

/**
 * @brief Set blue LED brightness using soft PWM
 * @param pwm_val 0-999 range matching hardware PWM convention
 *                999 = OFF (active low), 0 = full ON
 */
static void LED3_SetBrightness(uint32_t pwm_val)
{
    /* Convert from 0-999 (active-low HW convention) to duty ticks.
     * pwm_val 999 = OFF  -> duty = 0
     * pwm_val 0   = ON   -> duty = SOFT_PWM_PERIOD
     */
    if (pwm_val >= 999) {
        soft_pwm_blue_duty = 0;
    } else {
        /* Map: brightness = 999 - pwm_val (0..999), then scale to 0..PERIOD */
        uint32_t brightness = 999 - pwm_val;
        soft_pwm_blue_duty = (uint16_t)((brightness * SOFT_PWM_PERIOD) / 999);
    }
}

static void LED3_Enable(void)
{
    soft_pwm_enabled = 1;
}

static void LED3_Disable(void)
{
    soft_pwm_enabled = 0;
    soft_pwm_blue_duty = 0;
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET); /* OFF */
}

/* =========================================================================
 * HELPER: start / stop hardware PWM channels
 * ========================================================================= */

static void StartRedPWM(void)   { HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); }
static void StartGreenPWM(void) { HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); }
static void StartBlueSoft(void) { LED3_Enable(); }

static void StopRedPWM(void)    { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 999); HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); }
static void StopGreenPWM(void)  { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 999); HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3); }
static void StopBlueSoft(void)  { LED3_Disable(); }

static inline void SetRed(uint32_t pwm)   { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm); }
static inline void SetGreen(uint32_t pwm) { __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm); }
static inline void SetBlue(uint32_t pwm)  { LED3_SetBrightness(pwm); }

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

void LED_Rainbow(int ms_delay, uint8_t intensity)
{
    static uint8_t color_phase = 0;
    static uint8_t fade_step = 0;
    static uint32_t last_update = 0;
    static uint32_t last_call = 0;

    if (intensity > 255) intensity = 255;

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
    StartBlueSoft();

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

    /* Convert RGB 0-255 to active-low PWM 0-999 */
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

void LED_Armed(int ms_delay, uint8_t intensity)
{
    StartRedPWM();
    StopGreenPWM();
    StopBlueSoft();

    static uint16_t pulse_value = 0;
    static uint8_t pulse_direction = 1;
    static uint32_t last_update = 0;
    static uint32_t last_call = 0;

    if (intensity > 255) intensity = 255;

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
    StopBlueSoft();
}

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
        if (blue > 0)  { StartBlueSoft(); }  else { StopBlueSoft(); }

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
        if (intensity > 255) intensity = 255;
        if (red > 0)   { uint16_t sr = (red   * intensity) / 255; SetRed(  999 - ((sr * 999) / 255)); }
        if (green > 0) { uint16_t sg = (green * intensity) / 255; SetGreen(999 - ((sg * 999) / 255)); }
        if (blue > 0)  { uint16_t sb = (blue  * intensity) / 255; SetBlue( 999 - ((sb * 999) / 255)); }
    } else {
        if (red > 0)   SetRed(999);
        if (green > 0) SetGreen(999);
        if (blue > 0)  SetBlue(999);
    }
}

void LED_Pulse(int duration, uint8_t r, uint8_t g, uint8_t b, uint8_t intensity)
{
    static uint16_t pulse_value = 0;
    static uint8_t direction = 1;
    static uint32_t last_update = 0;
    static uint32_t last_call = 0;
    static uint8_t initialized_pulse = 0;
    static uint8_t last_r = 0, last_g = 0, last_b = 0;

    uint32_t now = HAL_GetTick();

    if (intensity > 255) intensity = 255;
    if (duration < 20) duration = 20;

    #define PULSE_RESET_TIMEOUT 500
    if ((now - last_call) > PULSE_RESET_TIMEOUT) {
        pulse_value = 0;
        direction = 1;
        initialized_pulse = 0;
        last_update = now;
        last_r = last_g = last_b = 0;
    }
    last_call = now;

    if (!initialized_pulse || last_r != r || last_g != g || last_b != b) {
        if (r > 0) { StartRedPWM(); }   else { StopRedPWM(); }
        if (g > 0) { StartGreenPWM(); } else { StopGreenPWM(); }
        if (b > 0) { StartBlueSoft(); }  else { StopBlueSoft(); }

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
