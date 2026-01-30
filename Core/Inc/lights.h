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
void rainbow(int ms_delay);
void Armed(int ms_delay);
void testLED(int ms_delay);
void turnOffLED(void);
void chargeLED(int ms_delay);
void Lights(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIGHTS_H */
