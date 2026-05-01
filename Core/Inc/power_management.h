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

#endif /* INC_POWER_MANAGEMENT_H_ */
