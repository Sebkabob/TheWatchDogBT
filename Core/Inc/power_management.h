/*
 * power_management.h
 *
 * Ultra-low power management for WatchDogBT
 */

#ifndef INC_POWER_MANAGEMENT_H_
#define INC_POWER_MANAGEMENT_H_

#include <stdint.h>

/* Function prototypes */
void Configure_GPIO_For_LowPower(void);
void BQ25186_EnterLowPower(void);
void LIS2DUX12_EnterLowPower(void);
void Disable_Peripherals_For_Sleep(void);
void Configure_Wakeup_Optimized(void);
void Enter_DeepStop_Mode(void);
void Wakeup_System_Init(void);
uint8_t Should_Enter_Sleep(void);
uint32_t Get_Estimated_Sleep_Current_uA(void);

#endif /* INC_POWER_MANAGEMENT_H_ */
