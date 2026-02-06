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

void LED_Rainbow(int ms_delay, uint8_t intensity)
{
    // Static variables to maintain rainbow state
    static uint8_t color_phase = 0;  // 0-5 for the 6 color transitions
    static uint8_t fade_step = 0;    // 0-255 within each phase
    static uint32_t last_update = 0;
    static uint32_t last_call = 0;

    // Clamp intensity to 0-255 range
    if (intensity > 255) intensity = 255;

    uint32_t current_time = HAL_GetTick();

    // Reset if function hasn't been called for more than 500ms
    #define RAINBOW_RESET_TIMEOUT 500
    if ((current_time - last_call) > RAINBOW_RESET_TIMEOUT) {
        color_phase = 0;
        fade_step = 0;
        last_update = current_time;
    }
    last_call = current_time;

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  // LED1 - Red
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); // LED2 - Green
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // LED3 - Blue

    // Check if enough time has passed since last update
    if ((current_time - last_update) < ms_delay) {
        return;
    }
    last_update = current_time;

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

    // Apply intensity scaling (0-255)
    // This caps the maximum brightness
    red = (red * intensity) / 255;
    green = (green * intensity) / 255;
    blue = (blue * intensity) / 255;

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

    // Increment fade step - use smaller increment for smoother transition
    if (fade_step < 250) {  // Prevent overflow
        fade_step += 5;
    } else {
        fade_step = 0;
        color_phase = (color_phase + 1) % 6;  // Move to next color phase
    }
}

void LED_Armed(int ms_delay, uint8_t intensity)
{
    // Static variables to maintain fade state for red LED
    static uint16_t pulse_value = 0;
    static uint8_t pulse_direction = 1;  // 1 = increasing (fade in), 0 = decreasing (fade out)
    static uint32_t last_update = 0;
    static uint32_t last_call = 0;
    static uint8_t initialized_armed = 0;

    // Clamp intensity to 0-255 range
    if (intensity > 255) intensity = 255;

    uint32_t current_time = HAL_GetTick();

    // Reset if function hasn't been called for more than 500ms
    #define ARMED_RESET_TIMEOUT 500
    if ((current_time - last_call) > ARMED_RESET_TIMEOUT) {
        pulse_value = 0;
        pulse_direction = 1;
        last_update = current_time;
        initialized_armed = 0;
    }

    last_call = current_time;

    // One-time initialization
    if (!initialized_armed) {
        // Start RED PWM only
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  // LED1 - Red

        // Turn off and stop green and blue LEDs
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 999); // Green OFF
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 999);  // Blue OFF
        HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1); // Stop green timer
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);  // Stop blue timer

        initialized_armed = 1;
    }

    // Check if enough time has passed since last update
    if ((current_time - last_update) < ms_delay) {
        return;
    }

    last_update = current_time;

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

    // Apply intensity scaling
    uint16_t scaled_brightness = (pulse_value * intensity) / 255;

    // Convert brightness (0-255) to PWM (0-999)
    // Active LOW: 0 brightness = 999 PWM (off), 255 brightness = 0 PWM (full on)
    uint32_t red_pwm = 999 - ((scaled_brightness * 999) / 255);

    // Set red LED PWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, red_pwm);
}

void LED_Off(void)
{
    // For active LOW LEDs, set PWM to full period (100% duty = OFF)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 999);   // LED1 - Red OFF
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 999);  // LED2 - Green OFF
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 999);   // LED3 - Blue OFF

    // Stop all PWM timers to save power
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
}


/**
 * @brief Flash LED with custom color (non-blocking)
 * @param flash_interval_ms Time between on and off in milliseconds
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 * @param intensity Overall brightness multiplier (0-255, where 255 is full)
 */
void LED_Alarm(int flash_interval_ms, uint8_t red, uint8_t green, uint8_t blue, uint8_t intensity)
{
    static uint8_t led_state = 0;  // 0 = OFF, 1 = ON
    static uint32_t last_toggle_time = 0;
    static uint32_t last_call = 0;
    static uint8_t initialized_alarm = 0;
    static uint8_t last_r = 0, last_g = 0, last_b = 0;  // Track which colors were active

    uint32_t current_time = HAL_GetTick();

    // Reset if function hasn't been called for more than 500ms
    #define ALARM_RESET_TIMEOUT 500
    if ((current_time - last_call) > ALARM_RESET_TIMEOUT) {
        led_state = 0;
        last_toggle_time = current_time;
        initialized_alarm = 0;
        last_r = last_g = last_b = 0;
    }

    last_call = current_time;

    // One-time initialization OR when colors change
    if (!initialized_alarm || last_r != red || last_g != green || last_b != blue) {
        // Start only the timers we need, stop the ones we don't
        if (red > 0) {
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);   // LED1 - Red
        } else {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 999);  // Set to OFF first
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);    // Then stop timer
        }

        if (green > 0) {
            HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);  // LED2 - Green
        } else {
            __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 999);
            HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
        }

        if (blue > 0) {
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);   // LED3 - Blue
        } else {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 999);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
        }

        initialized_alarm = 1;
        last_toggle_time = current_time;
        led_state = 1;  // Start with LED ON
        last_r = red;
        last_g = green;
        last_b = blue;
    }

    // Check if it's time to toggle
    if ((current_time - last_toggle_time) >= flash_interval_ms) {
        led_state = !led_state;  // Toggle state
        last_toggle_time = current_time;
    }

    if (led_state) {
        // LED ON - apply color and intensity

        // Clamp intensity to 0-255
        if (intensity > 255) intensity = 255;

        // Apply intensity scaling to each color component and update only active channels
        if (red > 0) {
            uint16_t scaled_red = (red * intensity) / 255;
            uint32_t red_pwm = 999 - ((scaled_red * 999) / 255);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, red_pwm);
        }

        if (green > 0) {
            uint16_t scaled_green = (green * intensity) / 255;
            uint32_t green_pwm = 999 - ((scaled_green * 999) / 255);
            __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, green_pwm);
        }

        if (blue > 0) {
            uint16_t scaled_blue = (blue * intensity) / 255;
            uint32_t blue_pwm = 999 - ((scaled_blue * 999) / 255);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, blue_pwm);
        }
    } else {
        // LED OFF - turn off only the active colors
        if (red > 0) {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 999);    // Red OFF
        }
        if (green > 0) {
            __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 999);   // Green OFF
        }
        if (blue > 0) {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 999);    // Blue OFF
        }
    }
}

void LED_Pulse(int duration, uint8_t r, uint8_t g, uint8_t b, uint8_t intensity)
{
    static uint16_t pulse_value = 0;     // 0–255
    static uint8_t direction = 1;        // 1 = up, 0 = down
    static uint32_t last_update = 0;
    static uint32_t last_call = 0;
    static uint8_t initialized_pulse = 0;
    static uint8_t last_r = 0, last_g = 0, last_b = 0;  // Track which colors were active

    uint32_t now = HAL_GetTick();

    // Clamp intensity
    if (intensity > 255) intensity = 255;
    if (duration < 20) duration = 20;   // prevent divide-by-zero / jitter

    // Reset if function hasn't been called recently
    #define PULSE_RESET_TIMEOUT 500
    if ((now - last_call) > PULSE_RESET_TIMEOUT) {
        pulse_value = 0;
        direction = 1;
        initialized_pulse = 0;
        last_update = now;
        last_r = last_g = last_b = 0;
    }
    last_call = now;

    // One-time init OR when colors change
    if (!initialized_pulse || last_r != r || last_g != g || last_b != b) {
        // Start only the timers we need, stop the ones we don't
        if (r > 0) {
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);   // Red needed
        } else {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 999);  // Set to OFF first
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);    // Then stop timer
        }

        if (g > 0) {
            HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);  // Green needed
        } else {
            __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 999);
            HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
        }

        if (b > 0) {
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);   // Blue needed
        } else {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 999);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
        }

        initialized_pulse = 1;
        last_r = r;
        last_g = g;
        last_b = b;
    }

    // Calculate step timing
    // 512 steps total (256 up + 256 down)
    uint16_t step_delay = duration / 512;
    if (step_delay < 1) step_delay = 1;

    if ((now - last_update) < step_delay) {
        return;
    }
    last_update = now;

    // Update pulse
    if (direction) {
        pulse_value++;
        if (pulse_value >= 255) {
            pulse_value = 255;
            direction = 0;
        }
    } else {
        if (pulse_value > 0) {
            pulse_value--;
        }
        if (pulse_value == 0) {
            direction = 1;
        }
    }

    // Apply pulse + intensity and update only active channels
    if (r > 0) {
        uint16_t pr = (r * pulse_value * intensity) / (255 * 255);
        uint32_t red_pwm = 999 - ((pr * 999) / 255);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, red_pwm);
    }

    if (g > 0) {
        uint16_t pg = (g * pulse_value * intensity) / (255 * 255);
        uint32_t green_pwm = 999 - ((pg * 999) / 255);
        __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, green_pwm);
    }

    if (b > 0) {
        uint16_t pb = (b * pulse_value * intensity) / (255 * 255);
        uint32_t blue_pwm = 999 - ((pb * 999) / 255);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, blue_pwm);
    }
}
