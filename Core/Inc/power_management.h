/*
 * power_management.h
 *
 * Low-power peripheral gating for WatchDogBT
 *
 * Two low-power profiles:
 *   1. DISCONNECTED IDLE (not armed): Only BLE radio advertising.
 *      All timers, I2C, buzzer, LEDs, accelerometer interrupt OFF.
 *
 *   2. DISCONNECTED ARMED: BLE radio advertising + accelerometer
 *      interrupt active. Timers, I2C, buzzer, LEDs OFF.
 *
 * On BLE connect (or charging), call PowerMgmt_RestoreAll() to
 * bring everything back up.
 */

#ifndef INC_POWER_MANAGEMENT_H_
#define INC_POWER_MANAGEMENT_H_

#include <stdint.h>

/* ---- Main API ---------------------------------------------------------- */

/**
 * @brief  Shut down all non-essential peripherals for idle advertising.
 *         Accelerometer interrupt is DISABLED.
 *         Call when: disconnected + not armed + not charging.
 */
void PowerMgmt_EnterLowPower_Idle(void);

/**
 * @brief  Shut down non-essential peripherals but keep accelerometer
 *         interrupt active so motion can trigger an alarm.
 *         Call when: disconnected + armed + not charging.
 */
void PowerMgmt_EnterLowPower_Armed(void);

/**
 * @brief  Restore all peripherals to full-run state.
 *         Call when: BLE connection established, or charging detected.
 *         Safe to call multiple times (idempotent).
 */
void PowerMgmt_RestoreAll(void);

/**
 * @brief  Query whether peripherals are currently gated.
 * @return 1 if in a low-power gated state, 0 if fully running.
 */
uint8_t PowerMgmt_IsLowPower(void);

#endif /* INC_POWER_MANAGEMENT_H_ */
