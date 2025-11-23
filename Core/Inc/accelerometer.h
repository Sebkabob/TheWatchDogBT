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
 * PUBLIC DEFINES
 ***************************************************************************/
#define MOTION_THRESHOLD_LOW      2
#define MOTION_THRESHOLD_MEDIUM   8
#define MOTION_THRESHOLD_HIGH     20

/***************************************************************************
 * PUBLIC TYPES
 ***************************************************************************/
// Add any public types here if needed in the future

/***************************************************************************
 * PUBLIC VARIABLES
 ***************************************************************************/
extern stmdev_ctx_t dev_ctx;
extern I2C_HandleTypeDef hi2c1;  // Add this line!

/***************************************************************************
 * PUBLIC FUNCTION PROTOTYPES
 ***************************************************************************/

/**
 * @brief Initialize the LIS2DUX12 accelerometer
 * @details Configures the sensor for low-power motion detection with
 *          wake-up interrupt on INT1 pin. Sets 25Hz ODR, ±2g range.
 * @return 0 on success, -1 on error
 */
int32_t LIS2DUX12_Init(void);

/**
 * @brief Non-blocking check for motion detection
 * @details Checks if motion has been detected since the last call.
 *          This function clears the motion flag after reading.
 * @return 1 if motion was detected, 0 otherwise
 * @note This is the main function to use in your application loop
 */
uint8_t LIS2DUX12_IsMotionDetected(void);

/**
 * @brief Peek at motion status without clearing flag
 * @details Check the current state of the motion detection flag
 *          without clearing it. Useful for checking state without
 *          consuming the event.
 * @return Current state of motion detection flag
 */
uint8_t LIS2DUX12_PeekMotionStatus(void);

/**
 * @brief Manually clear the motion detection flag
 * @details Resets the motion detection flag without reading it.
 *          Useful when you want to discard pending motion events.
 */
void LIS2DUX12_ClearMotionFlag(void);

/**
 * @brief Set motion detection threshold
 * @details Adjust the sensitivity of motion detection at runtime.
 *          Lower values = more sensitive, higher values = less sensitive.
 * @param threshold Motion threshold value (recommended: 2-20)
 *                  - 2  = very sensitive (detects small vibrations)
 *                  - 8  = medium sensitivity (recommended for security)
 *                  - 20 = less sensitive (only larger movements)
 * @return 0 on success, -1 on error
 */
int32_t LIS2DUX12_SetMotionThreshold(uint8_t threshold);

/**
 * @brief Configure PB0 as a wake-up source from sleep mode
 * @details Sets up the accelerometer interrupt pin to wake the MCU
 *          from low-power sleep mode when motion is detected.
 * @note Call this function before entering sleep mode
 */
void LIS2DUX12_ConfigureWakeup(void);

/**
 * @brief Clear all accelerometer interrupt sources
 * @details Reads all interrupt source registers to clear any
 *          pending interrupts (wake-up, tap, 6D, etc.)
 */
void LIS2DUX12_ClearAllInterrupts(void);

/**
 * @brief Read raw acceleration data
 * @details Reads the current X, Y, Z acceleration values from the sensor
 * @param accel Array of 3 int16_t to store [X, Y, Z] acceleration values
 * @note Raw values are in LSB units. Convert using sensitivity factor:
 *       - ±2g: multiply by 0.061 to get mg
 */
void LIS2DUX12_ReadAcceleration(int16_t accel[3]);

/**
 * @brief Scan I2C bus for devices
 * @details Scans all I2C addresses (0-127) and checks for device presence.
 *          Useful for debugging I2C communication issues.
 * @note This is a debugging function and should not be used in production
 */
void LIS2DUX12_I2CScan(void);

#endif /* INC_ACCELEROMETER_H_ */
