/***************************************************************************
 * accelerometer.h
 * created by Sebastian Forenza 2026
 *
 * LIS2DUX12 accelerometer driver — public API for MLC asset tracking,
 * low-power wake-up modes, tilt detection, and motion-flag access.
 ***************************************************************************/

#ifndef INC_ACCELEROMETER_H_
#define INC_ACCELEROMETER_H_

#include "main.h"
#include "lis2dux12_reg.h"

extern stmdev_ctx_t dev_ctx;
extern I2C_HandleTypeDef hi2c1;

// Loads MLC asset-tracking UCF (25 Hz, +/-16 g, LP). Returns 0 on success.
int32_t LIS2DUX12_Init(void);

// Returns 1 and clears flag if INT1 fired since last call, 0 otherwise.
uint8_t LIS2DUX12_IsMotionDetected(void);

// Reads the motion flag without clearing it.
uint8_t LIS2DUX12_PeekMotionStatus(void);

void LIS2DUX12_ClearMotion(void);
void LIS2DUX12_ClearMotionFlag(void);

// Arms PB15 as a DEEPSTOP wakeup source (HIGH polarity).
void LIS2DUX12_ConfigureWakeup(void);

// Power-down (ODR=0, ~0.4 µA). Call before gating I2C.
int32_t LIS2DUX12_PowerDown(void);

// Software-reset then power-down — clears MLC/FSM residual current.
int32_t LIS2DUX12_ResetAndPowerDown(void);

// 1.6 Hz ULP wake-on-motion (~1.5 µA). Wipes MLC/FSM. LOW sensitivity.
int32_t LIS2DUX12_EnterUltraLowPowerWakeup(void);

// 3 Hz ULP wake-on-motion (~1.7 µA). Wipes MLC/FSM. MEDIUM sensitivity.
int32_t LIS2DUX12_EnterMediumLowPowerWakeup(void);

// Adds wake-up engine on top of the running UCF — MLC keeps classifying.
int32_t LIS2DUX12_ConfigArmedSleep(void);

void LIS2DUX12_ClearAllInterrupts(void);
void LIS2DUX12_ReadAcceleration(int16_t accel[3]);

// Snapshot current gravity vector as the tilt-detection reference.
void LIS2DUX12_CaptureReference(void);

// Returns 1 if tilted >15° from reference (10° hysteresis to clear).
uint8_t LIS2DUX12_CheckTilt(void);

void LIS2DUX12_I2CScan(void);

#endif /* INC_ACCELEROMETER_H_ */
