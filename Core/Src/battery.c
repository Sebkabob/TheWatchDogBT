/*
 * battery.c
 *
 *  Created on: Sep 7, 2025
 *      Author: sebkabob
 */

#include "bq25186_reg.h"
#include "main.h"
#include "battery.h"

static uint32_t pulse_value = 0;
static uint8_t pulse_direction = 1; // 1 = increasing, 0 = decreasing
static uint32_t pulse_step = 5; // Adjust for pulse speed (smaller = slower)

#define BQ25186_ADDR (0x6A << 1)

void batteryInit(){
	  BQ25186_DisablePrechargeLimit();
	  BQ25186_SetBatteryVoltage(4.2f);   // charge to 90% for safety
}

uint8_t readBQ25186Register(uint8_t regAddr) {
    uint8_t data = 0;

    // Step 1: Set the register pointer (write operation)
    if (HAL_I2C_Master_Transmit(&hi2c1, BQ25186_ADDR, &regAddr, 1, HAL_MAX_DELAY) != HAL_OK) {
        // Error handling
        return 0xFF; // Indicate error
    }

    // Step 2: Read the register data
    if (HAL_I2C_Master_Receive(&hi2c1, BQ25186_ADDR | 0x01, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
        // Error handling
        return 0xFF; // Indicate error
    }

    return data;
}

bool Battery_IsCharging(void)
{
	if (HAL_GPIO_ReadPin(GPIOB, CHARGE_Pin) == 0){
		return true;
	} else {
		return false;
	}
}

void ChargeLED(int ms_delay)
{
    // Read Power Good status (LOW = good power present)
    int power_good = (HAL_GPIO_ReadPin(GPIOB, CHARGE_Pin) == GPIO_PIN_RESET);

    // Read charging status from BQ25186
    uint8_t stat0 = readBQ25186Register(0x00);
    uint8_t charge_status = (stat0 >> 5) & 0x03;
    int is_charging = (charge_status == 0x01 || charge_status == 0x02); // CC or CV

    // Static variable to track previous charging state
    static int was_charging = 0;

    // Check if we just started charging (transition from not charging to charging)
    if (power_good && is_charging && !was_charging) {
        // Reset pulse variables for smooth start
        pulse_value = 10;  // Start at minimum (LED off for active low)
        pulse_direction = 1;  // Start increasing (getting brighter)
    }

    // Update the previous state
    was_charging = (power_good && is_charging);

    // Only pulse LED when power is good AND actively charging
    if (power_good && is_charging) {
        // Start PWM if not already running
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

        // Keep pulse range away from extremes to avoid hardware timing issues
        uint32_t min_pulse = 0;   // Don't go below 10
        uint32_t max_pulse = htim2.Init.Period - 400;  // Don't go above 989

        if (pulse_direction) {
            pulse_value += pulse_step;
            if (pulse_value >= max_pulse) {
                pulse_direction = 0;  // Start decreasing
            }
        } else {
            pulse_value -= pulse_step;
            if (pulse_value <= min_pulse) {
                pulse_direction = 1;  // Start increasing
            }
        }

        // For active low LED: invert the PWM value
        uint32_t inverted_pulse = htim2.Init.Period - pulse_value;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, inverted_pulse);
        HAL_Delay(ms_delay);
    } else {
        // Stop PWM completely to turn off LED and save power
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    }
}

void BQ25186_SetChargeCurrent(uint16_t current_mA) {
    uint8_t ichg_code;

    // Calculate ICHG code based on desired current
    if (current_mA <= 35) {
        ichg_code = current_mA - 5;  // For currents 5-35mA
    } else if (current_mA >= 40) {
        ichg_code = 31 + ((current_mA - 40) / 10);  // For currents 40mA+
    } else {
        ichg_code = 30;  // Default to 35mA for invalid range
    }

    // Ensure we don't exceed maximum
    if (ichg_code > 127) ichg_code = 127;  // Max code for 1000mA

    // Write to ICHG_CTRL register (0x4)
    // Bit 7 = 0 (charging enabled), Bits 6-0 = ichg_code
    uint8_t reg_value = ichg_code & 0x7F;  // Ensure bit 7 is 0

    // I2C write to register 0x4
    HAL_I2C_Mem_Write(&hi2c1, BQ25186_ADDR, 0x04,
                      I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
}

void BQ25186_DisablePrechargeLimit(void) {
    uint8_t reg_value;

    // 1. Set fast charge current to maximum safe value for 300mAh battery
    // For a 300mAh battery, 1C rate = 300mA
    // Let's use 300mA for fast charging (1C rate is safe)
    // ICHG = 300mA corresponds to code 67 (from register 0x04)
    // Formula: For ICHG > 35mA: ICHG = 40 + ((code-31)*10)
    // 300 = 40 + ((code-31)*10) → code = 57
    reg_value = 0x80 | 57;  // Enable current monitoring + 300mA code
    HAL_I2C_Mem_Write(&hi2c1, BQ25186_ADDR, 0x04,
                      I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);

    // 2. Configure precharge current (20% of fast charge = 60mA)
    // Set IPRECHG = 0 (precharge is 2x termination current)
    // Read register 0x05 first
    HAL_I2C_Mem_Read(&hi2c1, BQ25186_ADDR, 0x05,
                     I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);

    // Clear bit 6 (IPRECHG = 0, precharge is 2x term)
    // Set termination to 10% (bits 5:4 = 10b) for 30mA term, 60mA precharge
    reg_value &= ~(0x01 << 6);  // IPRECHG = 0 (2x term)
    reg_value &= ~(0x03 << 4);  // Clear ITERM bits
    reg_value |= (0x02 << 4);   // ITERM = 10% (30mA)

    HAL_I2C_Mem_Write(&hi2c1, BQ25186_ADDR, 0x05,
                      I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);

    // 3. Set battery regulation voltage to 4.2V (assuming Li-ion)
    // VBATREG = 3.5V + (code * 10mV)
    // 4.2V = 3.5V + (70 * 10mV)
    reg_value = 70;  // 0x46
    HAL_I2C_Mem_Write(&hi2c1, BQ25186_ADDR, 0x03,
                      I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);

    // 4. Set input current limit to support the charge current
    // Need at least 300mA + system load
    // Set ILIM to 500mA (register 0x08, bits 2:0 = 101b)
    HAL_I2C_Mem_Read(&hi2c1, BQ25186_ADDR, 0x08,
                     I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);

    reg_value &= ~(0x07);  // Clear ILIM bits
    reg_value |= 0x05;     // Set to 500mA (101b)

    HAL_I2C_Mem_Write(&hi2c1, BQ25186_ADDR, 0x08,
                      I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);

    // 5. Set safety timer to 3 hours (register 0x07, bits 3:2 = 00b)
    HAL_I2C_Mem_Read(&hi2c1, BQ25186_ADDR, 0x07,
                     I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);

    reg_value &= ~(0x03 << 2);  // Set safety timer to 3 hours

    HAL_I2C_Mem_Write(&hi2c1, BQ25186_ADDR, 0x07,
                      I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);

    // 6. Enable charging (register 0x04, bit 7 = 0)
    HAL_I2C_Mem_Read(&hi2c1, BQ25186_ADDR, 0x04,
                     I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);

    reg_value &= ~(0x01 << 7);  // CHG_DIS = 0 (enable charging)

    HAL_I2C_Mem_Write(&hi2c1, BQ25186_ADDR, 0x04,
                      I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
}

void BQ25186_SetBatteryVoltage(float voltage_V) {
    uint8_t vbat_code;

    // Clamp voltage to valid range
    if (voltage_V < 3.5f) voltage_V = 3.5f;
    if (voltage_V > 4.65f) voltage_V = 4.65f;

    // Calculate VBATREG code
    vbat_code = (uint8_t)((voltage_V - 3.5f) / 0.01f);  // 0.01V = 10mV steps

    // Ensure we don't exceed 7-bit range
    if (vbat_code > 127) vbat_code = 127;

    // Read current register to preserve PG_MODE bit
    uint8_t current_reg;
    HAL_I2C_Mem_Read(&hi2c1, BQ25186_ADDR, 0x03,
                     I2C_MEMADD_SIZE_8BIT, &current_reg, 1, HAL_MAX_DELAY);

    // Preserve bit 7 (PG_MODE), update bits 6-0
    uint8_t new_reg = (current_reg & 0x80) | (vbat_code & 0x7F);

    // Write to VBAT_CTRL register
    HAL_I2C_Mem_Write(&hi2c1, BQ25186_ADDR, 0x03,
                      I2C_MEMADD_SIZE_8BIT, &new_reg, 1, HAL_MAX_DELAY);
}
