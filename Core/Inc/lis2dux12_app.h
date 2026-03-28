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
 ***************************************************************************/

/**
 * @brief  Update the cached MLC state (call after reading MLC output).
 */
void lis2dux12_app_update_cached_state(uint8_t mlc_out);

/**
 * @brief  Get the cached MLC state mapped to a simple 0-3 value for BLE.
 *         0 = Stationary (original position), 1 = Tilted (>15 deg),
 *         2 = In Motion, 3 = Shaken, 0xFF = Unknown
 */
uint8_t lis2dux12_app_get_cached_mlc_state(void);

/**
 * @brief  Set/clear the tilt-detected flag.
 *         When set, stationary MLC states are overridden to "Tilted" (1)
 *         in the BLE status.
 */
void lis2dux12_app_set_tilt_detected(uint8_t tilted);

#endif /* INC_LIS2DUX12_APP_H_ */
