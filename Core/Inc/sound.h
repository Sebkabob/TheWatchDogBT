/*
 * sound.h
 *
 *  Created on: Oct 29, 2025
 *      Author: sebkabob
 */

#ifndef INC_SOUND_H_
#define INC_SOUND_H_

#include <stdint.h>

extern TIM_HandleTypeDef htim2;

void playTone(uint32_t frequency_hz, uint32_t duration_ms);
void firstBootTone();
void SOUND_CalmAlarm();
void SOUND_NormalAlarm();
void SOUND_LoudAlarm();
//void playSong();


#endif /* INC_SOUND_H_ */
