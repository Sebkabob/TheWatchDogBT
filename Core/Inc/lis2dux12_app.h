/***************************************************************************
 * lis2dux12_app.h
 * created by Sebastian Forenza 2026
 *
 * MLC asset-tracking module for the LIS2DUX12.
 *
 * NOTE: Loading the UCF replaces ALL sensor config (ODR, FS, INTs). It is
 * incompatible with the wake-up-only setup in accelerometer.c — use one
 * mode at a time.
 ***************************************************************************/

#ifndef INC_LIS2DUX12_APP_H_
#define INC_LIS2DUX12_APP_H_

#include "main.h"
#include "lis2dux12_reg.h"

// MLC1_SRC register values from the asset-tracking decision tree.
#define MLC_STATE_STATIONARY_UPRIGHT     0x00
#define MLC_STATE_STATIONARY_NOT_UPRIGHT 0x04
#define MLC_STATE_IN_MOTION              0x08
#define MLC_STATE_SHAKEN                 0x0C

// FSM program indices in the asset-tracking UCF.
#define FSM_PROGRAM_IMPACT    0  // FSM1
#define FSM_PROGRAM_FREEFALL  1  // FSM2

extern stmdev_ctx_t dev_ctx;
extern I2C_HandleTypeDef hi2c1;

// Init: WHO_AM_I, software reset, then replay the UCF stream.
// Returns 0 on success, -1 WHO_AM_I, -2 reset timeout, -3 UCF error.
int lis2dux12_app_init(I2C_HandleTypeDef *hi2c);

// Read the MLC1_SRC classification byte. 0 on success.
int lis2dux12_app_get_mlc_output(uint8_t *mlc_out);

// Returns a static string describing the MLC state.
const char* lis2dux12_app_decode_mlc(uint8_t mlc_val);

// Sets *impact / *freefall from the FSM status registers. 0 on success.
int lis2dux12_app_check_fsm_events(uint8_t *impact, uint8_t *freefall);

// Returns 1 if MLC1 output changed since the last read.
int lis2dux12_app_mlc_status_changed(void);

// Cached state byte sent over BLE (DEVICESTATUS byte 6):
//   0    Stationary
//   2    In Motion  (MLC)
//   3    Shaken     (MLC)
//   0xFE Stabilizing (waiting for stillness before locking)
//   0xFF Unknown
#define CACHED_STATE_STATIONARY     0
#define CACHED_STATE_IN_MOTION      2
#define CACHED_STATE_SHAKEN         3
#define CACHED_STATE_STABILIZING    0xFE

void lis2dux12_app_update_cached_state(uint8_t mlc_out);

// Returns CACHED_STATE_STABILIZING while the override is set, else the cached MLC.
uint8_t lis2dux12_app_get_cached_mlc_state(void);

// 1 = report STABILIZING in the BLE status; clear when leaving the state.
void lis2dux12_app_set_stabilizing(uint8_t on);

// Instantaneous X/Y/Z in milli-g — FS/ODR read back from the chip.
int lis2dux12_app_read_accel_mg(int16_t *x_mg, int16_t *y_mg, int16_t *z_mg);

#endif /* INC_LIS2DUX12_APP_H_ */
