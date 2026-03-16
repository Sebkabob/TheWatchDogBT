/*
 * power_management.h
 *
 * Low-power peripheral gating for WatchDogBT
 */

#ifndef INC_POWER_MANAGEMENT_H_
#define INC_POWER_MANAGEMENT_H_

#include <stdint.h>

/* ---- Main API ---------------------------------------------------------- */

void PowerMgmt_EnterLowPower_Idle(void);
void PowerMgmt_EnterLowPower_Armed(void);
void PowerMgmt_RestoreAll(void);
uint8_t PowerMgmt_IsLowPower(void);

/* ---- EEPROM power control (PB0) --------------------------------------- */

void PowerMgmt_EEPROM_PowerOn(void);
void PowerMgmt_EEPROM_PowerOff(void);

#endif /* INC_POWER_MANAGEMENT_H_ */
