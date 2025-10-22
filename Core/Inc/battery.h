/*
 * battery.h
 *
 *  Created on: Sep 7, 2025
 *      Author: sebkabob
 */

#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

#include "main.h"
#include "bq25186_reg.h"

extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;

void batteryInit(void);

uint8_t readBQ25186Register(uint8_t regAddr);

void ChargeLED(int ms_delay);

void BQ25186_SetChargeCurrent(uint16_t current_mA);

void BQ25186_SetBatteryVoltage(float voltage_V);

#endif /* INC_BATTERY_H_ */
