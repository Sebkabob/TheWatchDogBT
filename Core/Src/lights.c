/***************************************************************************
 * lights.c
 * created by __Sebastian__ __Forenza__ 2026
 *
 * Functions in charge of interfacing with the
 * __onboard__ LEDs
 ***************************************************************************/

#include "main.h"
#include "lights.h"
#include "sound.h"
#include "lockservice_app.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim16;
extern uint8_t deviceState;
extern uint8_t deviceInfo;

void rainbowLED(int ms_delay)
{
    // Static variables to maintain rainbow state
    static uint8_t color_phase = 0;  // 0-5 for the 6 color transitions
    static uint16_t fade_step = 0;   // 0-255 within each phase
    static uint32_t last_update = 0;

    // Check if enough time has passed since last update
    uint32_t current_time = HAL_GetTick();
    if ((current_time - last_update) < ms_delay) {
        return;
    }
    last_update = current_time;

    // Start PWM on all LED channels if not already running
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  // LED1 - Red
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); // LED2 - Green
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // LED3 - Blue

    // RGB values (0-255)
    uint16_t red = 0, green = 0, blue = 0;

    // Calculate RGB values based on current phase and fade step
    // Transitions: R->Y->G->C->B->M->R
    switch(color_phase) {
        case 0: // Red (255,0,0) -> Yellow (255,255,0) - fade in green
            red = 255;
            green = fade_step;
            blue = 0;
            break;

        case 1: // Yellow (255,255,0) -> Green (0,255,0) - fade out red
            red = 255 - fade_step;
            green = 255;
            blue = 0;
            break;

        case 2: // Green (0,255,0) -> Cyan (0,255,255) - fade in blue
            red = 0;
            green = 255;
            blue = fade_step;
            break;

        case 3: // Cyan (0,255,255) -> Blue (0,0,255) - fade out green
            red = 0;
            green = 255 - fade_step;
            blue = 255;
            break;

        case 4: // Blue (0,0,255) -> Magenta (255,0,255) - fade in red
            red = fade_step;
            green = 0;
            blue = 255;
            break;

        case 5: // Magenta (255,0,255) -> Red (255,0,0) - fade out blue
            red = 255;
            green = 0;
            blue = 255 - fade_step;
            break;
    }

    // Convert RGB (0-255) to PWM duty cycle (0-999)
    // For active LOW LEDs: higher PWM value = dimmer LED
    // So we invert: 0 RGB = 999 PWM (off), 255 RGB = 0 PWM (full brightness)
    uint32_t red_pwm = 999 - ((red * 999) / 255);
    uint32_t green_pwm = 999 - ((green * 999) / 255);
    uint32_t blue_pwm = 999 - ((blue * 999) / 255);

    // Set PWM duty cycles
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, red_pwm);    // LED1 - Red
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, green_pwm); // LED2 - Green
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, blue_pwm);   // LED3 - Blue

    // Increment fade step
    fade_step += 5;  // Adjust this value to control fade speed

    if (fade_step >= 255) {
        fade_step = 0;
        color_phase = (color_phase + 1) % 6;  // Move to next color phase
    }
}

void armedLED(int ms_delay)
{
    // Static variables to maintain fade state for red LED
    static uint16_t pulse_value = 0;
    static uint8_t pulse_direction = 1;  // 1 = increasing (fade in), 0 = decreasing (fade out)
    static uint32_t last_update = 0;
    static uint32_t last_call = 0;

    uint32_t current_time = HAL_GetTick();

    // Reset if function hasn't been called for more than 500ms
    #define ARMED_RESET_TIMEOUT 500
    if ((current_time - last_call) > ARMED_RESET_TIMEOUT) {
        pulse_value = 0;
        pulse_direction = 1;
        last_update = current_time;
    }

    last_call = current_time;

    // Check if enough time has passed since last update
    if ((current_time - last_update) < ms_delay) {
        return;
    }

    last_update = current_time;

    // Start PWM on LED1 (red) if not already running
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  // LED1 - Red

    // Turn off green and blue LEDs (set to full PWM = off for active low)
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); // LED2 - Green
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // LED3 - Blue
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 999); // Green OFF
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 999);  // Blue OFF

    // Update pulse value (0-255)
    if (pulse_direction) {
        pulse_value += 3;  // Adjust for fade speed
        if (pulse_value >= 255) {
            pulse_value = 255;
            pulse_direction = 0;  // Start fading out
        }
    } else {
        if (pulse_value >= 3) {
            pulse_value -= 3;
        } else {
            pulse_value = 0;
        }

        if (pulse_value == 0) {
            pulse_direction = 1;  // Start fading in
        }
    }

    // Convert brightness (0-255) to PWM (0-999)
    // Active LOW: 0 brightness = 999 PWM (off), 255 brightness = 0 PWM (full on)
    uint32_t red_pwm = 999 - ((pulse_value * 999) / 255);

    // Set red LED PWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, red_pwm);
}

void testLED(int ms_delay)
{
    // Static variables to maintain pulse state
    static uint32_t pulse_value = 0;
    static int pulse_direction = 1; // 1 = increasing, 0 = decreasing
    static uint8_t pulse_step = 20;
    static uint32_t last_update = 0;  // Track last update time

    // Check if enough time has passed since last update
    uint32_t current_time = HAL_GetTick();
    if ((current_time - last_update) < ms_delay) {
        return;  // Not enough time has passed, exit early
    }

    // Update the timestamp
    last_update = current_time;

    // Start PWM if not already running
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // Define pulse range
    uint32_t min_pulse = 0;
    uint32_t max_pulse = htim2.Init.Period - 100; // Leave some headroom

    // Update pulse value
    if (pulse_direction) {
        pulse_value += pulse_step;
        if (pulse_value >= max_pulse) {
            pulse_direction = 0; // Start decreasing
        }
    } else {
        pulse_value -= pulse_step;
        if (pulse_value <= min_pulse) {
            pulse_direction = 1; // Start increasing
        }
    }

    // For active low LED: invert the PWM value
    uint32_t inverted_pulse = htim2.Init.Period - pulse_value;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, inverted_pulse);

    // NO HAL_Delay() - function returns immediately!
}

void turnOffLED(void)
{
    // For active LOW LEDs, set PWM to full period (100% duty = OFF)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 999);   // LED1 - Red OFF
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 999);  // LED2 - Green OFF
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 999);   // LED3 - Blue OFF

    // Optionally stop PWM to save power
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
}

void chargeLED(int ms_delay)
{
    // Static variables to maintain pulse state
    static uint32_t pulse_value = 0;
    static int pulse_direction = 1; // 1 = increasing, 0 = decreasing
    static uint8_t pulse_step = 20;
    static uint32_t last_update = 0;  // Track last update time
    static uint32_t last_call = 0;    // Track last function call time

    uint32_t current_time = HAL_GetTick();

    // Reset if function hasn't been called for more than 500ms (configurable)
    #define RESET_TIMEOUT 500  // milliseconds
    if ((current_time - last_call) > RESET_TIMEOUT) {
        // Reset to starting state - LED off, ready to fade in
        pulse_value = 0;
        pulse_direction = 1;  // Start increasing (fade in)
        last_update = current_time;
    }

    // Update last call time
    last_call = current_time;

    // Check if enough time has passed since last update
    if ((current_time - last_update) < ms_delay) {
        return;  // Not enough time has passed, exit early
    }

    // Update the timestamp
    last_update = current_time;

    // Start PWM if not already running
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // Define pulse range
    uint32_t min_pulse = 0;
    uint32_t max_pulse = htim2.Init.Period - 100; // Leave some headroom

    // Update pulse value
    if (pulse_direction) {
        pulse_value += pulse_step;
        if (pulse_value >= max_pulse) {
            pulse_direction = 0; // Start decreasing
        }
    } else {
        pulse_value -= pulse_step;
        if (pulse_value <= min_pulse) {
            pulse_direction = 1; // Start increasing
        }
    }

    // For active low LED: invert the PWM value
    uint32_t inverted_pulse = htim2.Init.Period - pulse_value;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, inverted_pulse);

    // NO HAL_Delay() - function returns immediately!
}
