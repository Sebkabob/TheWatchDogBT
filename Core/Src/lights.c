/*
 * lights.c
 *
 *  Created on: Oct 29, 2025
 *      Author: sebkabob
 */

#include "main.h"
#include "lights.h"
#include "sound.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim2;
extern uint8_t lockState;

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

void Lights(){
	if (HAL_GPIO_ReadPin(GPIOB, CHARGE_Pin) == 0){
		chargeLED(50);
	}
	else if (lockState == 1){
        testLED(10);
    }
    else if (lockState == 2) {
    	playTone(4186,400);
    	playTone(3520,400);
    	playTone(4186,400);
    	playTone(3520,400);
    	playTone(4186,400);
    	playTone(3520,400);
    	playTone(4186,400);
    	playTone(3520,400);
    	playTone(4186,400);
    	playTone(3520,400);
    	lockState = 0;
    }
    else {
        // Stop PWM and turn off LED
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

        // Set LED pin HI (since your LED is active low, this turns it off)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1);
    }
}
