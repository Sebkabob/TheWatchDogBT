/*
 * buzzer.c
 *
 *  Created on: Sep 1, 2025
 *      Author: sebkabob
 */
#include "main.h"
#include "buzzer.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


void playTone(uint32_t frequency_hz, uint32_t duration_ms) {
    if (frequency_hz == 0 || duration_ms == 0) return;

    uint32_t system_clock_hz = 64000000 / 8;

    uint32_t cycles_per_half_period = system_clock_hz / (2 * frequency_hz); // CPU cycles for half period
    uint32_t start_time = HAL_GetTick();

    // Play tone for the specified duration
    while (HAL_GetTick() - start_time < duration_ms) {
        for (volatile uint32_t i = 0; i < cycles_per_half_period; i++); // High phase
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, 1);

        for (volatile uint32_t i = 0; i < cycles_per_half_period; i++); // Low phase
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, 0);
    }
}
