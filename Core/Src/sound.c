/***************************************************************************
 * sound.c
 * created by Sebastian Forenza 2026
 *
 * Functions in charge of interfacing with the
 * onboard magnetic transducers
 ***************************************************************************/

#include "main.h"
#include "sound.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

void BUZZER_Tone(uint32_t frequency_hz, uint32_t duration_ms) {
    if (frequency_hz == 0 || duration_ms == 0) return;

    // Calculate the period needed for this frequency
    // Timer clock = 64MHz / (prescaler + 1) = 64MHz / 64 = 1MHz
    uint32_t timer_clock = 64000000 / 64;  // 1MHz
    uint32_t period = timer_clock / frequency_hz / 2;

    // Limit period to reasonable bounds
    if (period > 65535) period = 65535;  // Max for 16-bit timer
    if (period < 2) period = 2;          // Min for audible output

    // Stop the timer to reconfigure
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    // Set new period for this frequency
    __HAL_TIM_SET_AUTORELOAD(&htim2, period - 1);

    // Set 50% duty cycle for square wave
    uint32_t pulse = period / 2;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);

    // Start PWM on buzzer channel
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // Play for specified duration
    HAL_Delay(duration_ms);

    // Stop the buzzer
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    // Restore original period for LED (999)
    __HAL_TIM_SET_AUTORELOAD(&htim2, 999);
}

void firstBootTone(){
	BUZZER_Tone(160, 10);
    HAL_Delay(15);
    BUZZER_Tone(230, 20);
    HAL_Delay(15);
    BUZZER_Tone(292, 30);
}

void SOUND_CalmAlarm(){
	BUZZER_Tone(415, 20);
    HAL_Delay(15);
    BUZZER_Tone(349, 20);
    HAL_Delay(15);
}

void SOUND_NormalAlarm(){
	BUZZER_Tone(1047, 300);
    HAL_Delay(15);
    BUZZER_Tone(880, 300);
    HAL_Delay(15);
    BUZZER_Tone(1047, 300);
    HAL_Delay(15);
    BUZZER_Tone(880, 300);
    HAL_Delay(15);
}

void SOUND_LoudAlarm(){
	BUZZER_Tone(2186, 300);
    HAL_Delay(15);
    BUZZER_Tone(3520, 300);
    HAL_Delay(15);
    BUZZER_Tone(2186, 300);
    HAL_Delay(15);
    BUZZER_Tone(3520, 300);
    HAL_Delay(15);
}

void SOUND_Disconnected(){
    BUZZER_Tone(380,10);
    HAL_Delay(10);
    BUZZER_Tone(280,12);
    HAL_Delay(10);
    BUZZER_Tone(100,15);
}
