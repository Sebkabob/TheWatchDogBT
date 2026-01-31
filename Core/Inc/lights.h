/***************************************************************************
 * lights.h
 * created by __Sebastian__ __Forenza__ 2026
 *
 * Header file for LED interface functions
 ***************************************************************************/

#ifndef __LIGHTS_H
#define __LIGHTS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Function prototypes */
void LED_Rainbow(int ms_delay);
void LED_Armed(int ms_delay);
void LED_Test(int ms_delay);
void LED_Off(void);
void LED_Charging(int ms_delay);

#ifdef __cplusplus
}
#endif

#endif /* __LIGHTS_H */
