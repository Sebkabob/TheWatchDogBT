/***************************************************************************
 * power_management.h
 * created by Sebastian Forenza 2026
 *
 * Public API for low-power peripheral gating + EEPROM power control.
 ***************************************************************************/

#ifndef INC_POWER_MANAGEMENT_H_
#define INC_POWER_MANAGEMENT_H_

#include <stdint.h>

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

#endif /* INC_POWER_MANAGEMENT_H_ */
