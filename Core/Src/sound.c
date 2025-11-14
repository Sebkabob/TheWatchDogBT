/*
 * sound.c
 *
 *  Created on: Oct 29, 2025
 *      Author: sebkabob
 */

#include "main.h"
#include "sound.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

void playTone(uint32_t frequency_hz, uint32_t duration_ms) {
    if (frequency_hz == 0 || duration_ms == 0) return;

    // Calculate the period needed for this frequency
    // Timer clock = 64MHz / (__prescaler__ + 1) = 64MHz / 64 = 1MHz
    uint32_t timer_clock = 64000000 / 64;  // 1MHz
    uint32_t period = timer_clock / frequency_hz;

    // Limit period to reasonable bounds
    if (period > 65535) period = 65535;  // Max for 16-bit timer
    if (period < 2) period = 2;          // __Min__ for audible output

    // Stop the timer to reconfigure
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

    // Set new period for this frequency
    __HAL_TIM_SET_AUTORELOAD(&htim2, period - 1);

    // Set 50% duty cycle for square wave
    uint32_t pulse = period / 2;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);

    // Start PWM on both buzzer channels
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    // Play for specified duration
    HAL_Delay(duration_ms);

    // Stop the __buzzers__
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

    // Restore original period for LED (999)
    __HAL_TIM_SET_AUTORELOAD(&htim2, 999);
}

void firstBootTone(){
    playTone(260*2, 10);
    HAL_Delay(15);
    playTone(330*2, 20);
    HAL_Delay(15);
    playTone(392*2, 30);
}
