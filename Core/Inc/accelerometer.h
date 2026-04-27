/*
 * accelerometer.h
 *
 *  Created on: Nov 22, 2025
 *      Author: sebkabob
 */

#ifndef INC_ACCELEROMETER_H_
#define INC_ACCELEROMETER_H_

#include "main.h"
#include "lis2dux12_reg.h"

/***************************************************************************
 * PUBLIC VARIABLES
 ***************************************************************************/
extern stmdev_ctx_t dev_ctx;
extern I2C_HandleTypeDef hi2c1;

/***************************************************************************
 * PUBLIC FUNCTION PROTOTYPES
 ***************************************************************************/

/**
 * @brief Initialize the LIS2DUX12 with MLC asset tracking configuration.
 * @details Loads the UCF config that programs the MLC decision tree and
 *          FSM programs. After init the sensor runs at 25 Hz, +/-16 g,
 *          low-power mode with on-chip motion classification.
 * @return 0 on success, negative on error
 */
int32_t LIS2DUX12_Init(void);

/**
 * @brief Non-blocking check for motion detection (interrupt flag).
 * @details Checks if the ACCEL_INT pin fired since the last call.
 *          Clears the flag after reading.
 * @return 1 if interrupt occurred, 0 otherwise
 */
uint8_t LIS2DUX12_IsMotionDetected(void);

/**
 * @brief Peek at motion status without clearing flag.
 * @return Current state of motion detection flag
 */
uint8_t LIS2DUX12_PeekMotionStatus(void);

/**
 * @brief Clear the motion detection flag without reading it.
 */
void LIS2DUX12_ClearMotion(void);
void LIS2DUX12_ClearMotionFlag(void);

/**
 * @brief Configure PB15 as a wake-up source from sleep mode.
 * @note Call before entering low-power sleep.
 */
void LIS2DUX12_ConfigureWakeup(void);

/**
 * @brief Power down accel completely (ODR=0, ~0.4 µA). No interrupts.
 * @return 0 on success, non-zero on I2C error
 */
int32_t LIS2DUX12_PowerDown(void);

/**
 * @brief Software-reset then power down (~0.4 µA). Clears MLC/FSM residual config.
 * @return 0 on success, non-zero on I2C error
 */
int32_t LIS2DUX12_ResetAndPowerDown(void);

/**
 * @brief Reconfigure accel into ultra-low-power wake-up-only mode (~1.5 µA).
 * @details Replaces MLC/FSM with a minimal 1.6 Hz wake-up-on-motion config.
 *          Call BEFORE gating I2C. Use LIS2DUX12_Init() to restore full config.
 * @return 0 on success, non-zero on I2C error
 */
int32_t LIS2DUX12_EnterUltraLowPowerWakeup(void);

/**
 * @brief Clear all accelerometer interrupt sources.
 */
void LIS2DUX12_ClearAllInterrupts(void);

/**
 * @brief Read raw acceleration data [X, Y, Z].
 */
void LIS2DUX12_ReadAcceleration(int16_t accel[3]);

/**
 * @brief Capture current gravity vector as the reference orientation.
 * @note  Call when device is armed/locked and stationary.
 */
void LIS2DUX12_CaptureReference(void);

/**
 * @brief Check if device is tilted > 15 deg from its reference orientation.
 * @note  Only meaningful when device is stationary (no dynamic accel).
 * @return 1 if tilted beyond threshold, 0 otherwise
 */
uint8_t LIS2DUX12_CheckTilt(void);

/**
 * @brief Scan I2C bus for devices (debug only).
 */
void LIS2DUX12_I2CScan(void);

#endif /* INC_ACCELEROMETER_H_ */
