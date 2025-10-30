/*
 * sound.c
 *
 *  Created on: Oct 29, 2025
 *      Author: sebkabob
 */


/*
 * buzzer.c
 *
 *  Created on: Sep 1, 2025
 *      Author: sebkabob
 */
#include "main.h"
#include "sound.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


//void playTone(uint32_t frequency_hz, uint32_t duration_ms) {
//    if (frequency_hz == 0 || duration_ms == 0) return;
//
//    uint32_t system_clock_hz = 64000000 / 8;
//
//    uint32_t cycles_per_half_period = system_clock_hz / (2 * frequency_hz); // CPU cycles for half period
//    uint32_t start_time = HAL_GetTick();
//
//    // Play tone for the specified duration
//    while (HAL_GetTick() - start_time < duration_ms) {
//        for (volatile uint32_t i = 0; i < cycles_per_half_period; i++); // High phase
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, 1);
//
//        for (volatile uint32_t i = 0; i < cycles_per_half_period; i++); // Low phase
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, 0);
//    }
//}

void playTone(uint32_t frequency_hz, uint32_t duration_ms) {
    if (frequency_hz == 0 || duration_ms == 0) return;

    // Calculate the period needed for this frequency
    // Timer clock = 64MHz / (prescaler + 1) = 64MHz / 64 = 1MHz
    uint32_t timer_clock = 64000000 / 64;  // 1MHz
    uint32_t period = timer_clock / frequency_hz;

    // Limit period to reasonable bounds
    if (period > 65535) period = 65535;  // Max for 16-bit timer
    if (period < 2) period = 2;          // Min for audible output

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

    // Stop the buzzers
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

    // Restore original period for LED (999)
    __HAL_TIM_SET_AUTORELOAD(&htim2, 999);
}
