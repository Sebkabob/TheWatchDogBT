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
    if (duration_ms == 0) return;

    // DON'T change frequency - just pulse the buzzer briefly
    // Use current period, just set duty to 50% then back to 0

    uint32_t current_period = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t pulse = current_period / 2;

    // Quick pulse
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);

    HAL_Delay(duration_ms);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void firstBootTone(){
    playTone(260*2, 40);
    HAL_Delay(15);
    playTone(330*2, 50);
    HAL_Delay(15);
    playTone(392*2, 70);
}
