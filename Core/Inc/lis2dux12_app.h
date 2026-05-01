/***************************************************************************
 * lis2dux12_app.h
 *
 * MLC (Machine Learning Core) asset tracking module for LIS2DUX12.
 * Uses ST's pre-built UCF configuration to classify motion states
 * and detect impact/free-fall events on-chip.
 *
 * NOTE: Loading the UCF replaces ALL sensor configuration (ODR, FS,
 * interrupts). This is incompatible with the wake-up interrupt setup
 * in accelerometer.c. Use one or the other, not both.
 ***************************************************************************/

#ifndef INC_LIS2DUX12_APP_H_
#define INC_LIS2DUX12_APP_H_

#include "main.h"
#include "lis2dux12_reg.h"

/***************************************************************************
 * MLC OUTPUT VALUES
 * MLC1_SRC register decodes from the asset tracking decision tree
 ***************************************************************************/
#define MLC_STATE_STATIONARY_UPRIGHT     0x00
#define MLC_STATE_STATIONARY_NOT_UPRIGHT 0x04
#define MLC_STATE_IN_MOTION              0x08
#define MLC_STATE_SHAKEN                 0x0C

/***************************************************************************
 * FSM PROGRAM INDICES (asset tracking UCF)
 * FSM1 = Impact detection, FSM2 = Free-fall detection
 ***************************************************************************/
#define FSM_PROGRAM_IMPACT    0  /* FSM1 */
#define FSM_PROGRAM_FREEFALL  1  /* FSM2 */

/***************************************************************************
 * EXTERNAL VARIABLES (defined in main.c / accelerometer.c)
 ***************************************************************************/
extern stmdev_ctx_t dev_ctx;
extern I2C_HandleTypeDef hi2c1;

/***************************************************************************
 * PUBLIC FUNCTION PROTOTYPES
 ***************************************************************************/

/**
 * @brief  Initialize LIS2DUX12 with MLC asset tracking configuration.
 * @param  hi2c  Pointer to the HAL I2C handle (e.g. &hi2c1)
 * @return 0 on success, -1 on WHO_AM_I failure, -2 on reset timeout,
 *         -3 on UCF load error
 */
int lis2dux12_app_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read the MLC1 output register (current motion classification).
 * @param  mlc_out  Pointer to store the MLC1_SRC value
 * @return 0 on success, non-zero on I2C error
 */
int lis2dux12_app_get_mlc_output(uint8_t *mlc_out);

/**
 * @brief  Decode an MLC output value to a human-readable string.
 * @param  mlc_val  Value read from MLC1_SRC
 * @return Static string describing the motion state
 */
const char* lis2dux12_app_decode_mlc(uint8_t mlc_val);

/**
 * @brief  Check FSM status for impact and free-fall events.
 * @param  impact   Set to 1 if FSM1 (impact) triggered, 0 otherwise
 * @param  freefall Set to 1 if FSM2 (free-fall) triggered, 0 otherwise
 * @return 0 on success, non-zero on I2C error
 */
int lis2dux12_app_check_fsm_events(uint8_t *impact, uint8_t *freefall);

/**
 * @brief  Check if MLC status has changed (new classification available).
 * @return 1 if MLC1 output changed since last read, 0 otherwise
 */
int lis2dux12_app_mlc_status_changed(void);

/***************************************************************************
 * CACHED MLC STATE (avoids I2C reads in the BLE status path)
 *
 * Byte values sent over BLE (status update byte 6):
 *   0    = Stationary
 *   2    = In Motion   (MLC)
 *   3    = Shaken      (MLC)
 *   0xFE = Stabilizing (waiting for stillness before locking)
 *   0xFF = Unknown
 ***************************************************************************/
#define CACHED_STATE_STATIONARY     0
#define CACHED_STATE_IN_MOTION      2
#define CACHED_STATE_SHAKEN         3
#define CACHED_STATE_STABILIZING    0xFE

/**
 * @brief  Update the cached MLC state (call after reading MLC output).
 */
void lis2dux12_app_update_cached_state(uint8_t mlc_out);

/**
 * @brief  Get the cached state for BLE status byte.
 *         Returns CACHED_STATE_STABILIZING when the stabilizing
 *         override is set, otherwise the latest MLC classification.
 */
uint8_t lis2dux12_app_get_cached_mlc_state(void);

/**
 * @brief  Toggle the stabilizing override on byte 6 of the BLE status.
 *         Call with 1 when entering STATE_STABILIZING, 0 when leaving.
 */
void lis2dux12_app_set_stabilizing(uint8_t on);

/**
 * @brief  Read instantaneous X/Y/Z acceleration in milli-g.
 *         Sensor mode is read back from the device, so this works
 *         regardless of the FS/ODR baked into the loaded UCF.
 * @return 0 on success, non-zero on I2C error.
 */
int lis2dux12_app_read_accel_mg(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg);

#endif /* INC_LIS2DUX12_APP_H_ */
