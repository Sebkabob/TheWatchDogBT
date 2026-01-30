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
extern uint8_t deviceState;
extern uint8_t deviceInfo;

void rainbow(int ms_delay)
{
    // Static variables to maintain rainbow state
    static uint8_t phase = 0;  // 0-5: R->Y->G->C->B->M->R
    static uint16_t fade_value = 0;
    static uint8_t fade_direction = 1; // 1 = increasing, 0 = decreasing
    static uint8_t fade_step = 5;
    static uint32_t last_update = 0;

    // Check if enough time has passed since last update
    uint32_t current_time = HAL_GetTick();
    if ((current_time - last_update) < ms_delay) {
        return;
    }

    last_update = current_time;

    // Define fade range
    #define FADE_MAX 255
    #define FADE_MIN 0

    // Update fade value
    if (fade_direction) {
        fade_value += fade_step;
        if (fade_value >= FADE_MAX) {
            fade_value = FADE_MAX;
            fade_direction = 0; // Start decreasing
        }
    } else {
        if (fade_value >= fade_step) {
            fade_value -= fade_step;
        } else {
            fade_value = FADE_MIN;
        }

        if (fade_value <= FADE_MIN) {
            fade_value = FADE_MIN;
            fade_direction = 1; // Start increasing
            phase = (phase + 1) % 6; // Move to next color phase
        }
    }

    // Software PWM simulation using brightness values
    uint8_t red = 0, green = 0, blue = 0;

    switch(phase) {
        case 0: // Red -> Yellow (add green)
            red = 255;
            green = fade_value;
            blue = 0;
            break;
        case 1: // Yellow -> Green (remove red)
            red = 255 - fade_value;
            green = 255;
            blue = 0;
            break;
        case 2: // Green -> Cyan (add blue)
            red = 0;
            green = 255;
            blue = fade_value;
            break;
        case 3: // Cyan -> Blue (remove green)
            red = 0;
            green = 255 - fade_value;
            blue = 255;
            break;
        case 4: // Blue -> Magenta (add red)
            red = fade_value;
            green = 0;
            blue = 255;
            break;
        case 5: // Magenta -> Red (remove blue)
            red = 255;
            green = 0;
            blue = 255 - fade_value;
            break;
    }

    // Apply software PWM by toggling based on brightness
    // Simple approach: use modulo counter for PWM simulation
    static uint8_t pwm_counter = 0;
    pwm_counter++;

    // LED1 (Red) - PA8
    HAL_GPIO_WritePin(GPIOA, LED1_Pin, (pwm_counter < red) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // LED2 (Green) - PB0
    HAL_GPIO_WritePin(GPIOB, LED2_Pin, (pwm_counter < green) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // LED3 (Blue) - PB3
    HAL_GPIO_WritePin(GPIOB, LED3_Pin, (pwm_counter < blue) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Armed(int ms_delay)
{
    // Static variables to maintain fade state for red LED
    static uint16_t pulse_value = 0;
    static uint8_t pulse_direction = 1; // 1 = increasing (fade in), 0 = decreasing (fade out)
    static uint8_t pulse_step = 3;
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

    // Define pulse range
    #define PULSE_MIN 0
    #define PULSE_MAX 255

    // Update pulse value
    if (pulse_direction) {
        pulse_value += pulse_step;
        if (pulse_value >= PULSE_MAX) {
            pulse_value = PULSE_MAX;
            pulse_direction = 0; // Start decreasing
        }
    } else {
        if (pulse_value >= pulse_step) {
            pulse_value -= pulse_step;
        } else {
            pulse_value = PULSE_MIN;
        }

        if (pulse_value <= PULSE_MIN) {
            pulse_value = PULSE_MIN;
            pulse_direction = 1; // Start increasing
        }
    }

    // Software PWM for LED1 (Red) - PA8
    static uint8_t pwm_counter = 0;
    pwm_counter++;

    HAL_GPIO_WritePin(GPIOA, LED1_Pin, (pwm_counter < pulse_value) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Turn off other LEDs
    HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_RESET);
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

    // Update the __timestamp__
    last_update = current_time;

    // Start PWM if not already running
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // Define pulse range
    uint32_t min_pulse = 0;
    uint32_t max_pulse = htim2.Init.Period - 100; // Leave some __headroom__

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
    // Stop PWM
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

    // Set GPIO HIGH to turn off LED (active low)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
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

    // Update the __timestamp__
    last_update = current_time;

    // Start PWM if not already running
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    // Define pulse range
    uint32_t min_pulse = 0;
    uint32_t max_pulse = htim2.Init.Period - 100; // Leave some __headroom__

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

void Lights(){

}
