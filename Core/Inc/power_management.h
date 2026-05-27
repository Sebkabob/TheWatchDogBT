/***************************************************************************
 * power_management.h
 * created by Sebastian Forenza 2026
 *
 * Public API for low-power peripheral gating + EEPROM power control.
 ***************************************************************************/

#ifndef INC_POWER_MANAGEMENT_H_
#define INC_POWER_MANAGEMENT_H_

#include <stdint.h>
#include "eeprom_map.h"

void PowerMgmt_EnterLowPower_Idle(void);   // disconnected idle, no motion wake
void PowerMgmt_EnterLowPower_Armed(void);  // armed/locked, accel wakes from DEEPSTOP
void PowerMgmt_RestoreAll(void);           // full wake (BLE connect, cable plug)
void PowerMgmt_RestoreForMotion(void);     // lean wake — skips LEDs / BATTERY_Init
uint8_t PowerMgmt_IsLowPower(void);

void PowerMgmt_EEPROM_PowerOn(void);
void PowerMgmt_EEPROM_PowerOff(void);

/* ---------------- Diagnostic accessors (boot/reset forensics) -------------
 *
 * PowerMgmt_CaptureResetCause must be called as the very first thing in
 * main() (before HAL_Init / SystemClock_Config) so the latched RCC->CSR
 * reset flags are read before anything has a chance to clear them. The
 * captured byte is then available for the lifetime of the boot via
 * PowerMgmt_GetResetCause.
 *
 * PowerMgmt_BootCount_Init runs after the EEPROM is up: it reads the
 * 4-byte boot counter from EEPROM offset 0x20, increments it, writes it
 * back, and caches the value for PowerMgmt_GetBootCount. If the EEPROM is
 * unreachable the cached value is 0 and the on-device counter is unchanged.
 *
 * EEPROM map (extends motion_logger.h's reserved 0x000..0x03F device-info
 * block — boot counter sits inside that reserved region):
 *   0x20..0x23   uint32_t LE boot_count
 */
void     PowerMgmt_CaptureResetCause(void);
uint8_t  PowerMgmt_GetResetCause(void);
void     PowerMgmt_BootCount_Init(void);
uint32_t PowerMgmt_GetBootCount(void);
uint32_t PowerMgmt_GetUptimeSeconds(void);

/* ---------------- Persisted BLE TX-power level ----------------------------
 *
 * Two-step radio output level applied via aci_hal_set_tx_power_level. The
 * NORMAL/HIGH enum maps to (En_High_Power, PA_Level) per the WB05 power
 * table (ble_api.h):
 *
 *   NORMAL = (0, 24)  →   0 dBm
 *   HIGH   = (1, 31)  →  +8 dBm   (legacy default — High SMPS rail needed)
 *
 * Stored at EEPROM offset 0x24 (2 bytes: magic + value). BleTxPower_Init
 * loads the cached value and pushes it to the radio — must run AFTER
 * MX_APPE_Init has stood up the BLE stack. BleTxPower_Set persists +
 * applies in one call. Out-of-range inputs clamp to HIGH so a corrupted
 * cell falls back to the legacy default instead of leaving the radio quiet.
 *
 * Note: aci_hal_set_tx_power_level only takes effect on new Link Layer
 * state machines, so a change applies on the next advertise/connect —
 * current connections keep the old level until they tear down.
 */
typedef enum {
    BLE_TX_POWER_NORMAL = 0,
    BLE_TX_POWER_HIGH   = 1,
} BleTxPower_t;

#define BLE_TX_POWER_DEFAULT       BLE_TX_POWER_HIGH
#define BLE_TX_POWER_COUNT         2u

/* EEPROM_BLE_TX_POWER_ADDR / _LEN / _MAGIC live in eeprom_map.h */

void         BleTxPower_Init(void);
BleTxPower_t BleTxPower_Get(void);
BleTxPower_t BleTxPower_Set(BleTxPower_t value);
void         BleTxPower_Apply(void);

#endif /* INC_POWER_MANAGEMENT_H_ */
