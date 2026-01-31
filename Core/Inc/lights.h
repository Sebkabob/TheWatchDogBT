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

#include <stdint.h>

/* Function prototypes */

/**
 * @brief Display rainbow color cycle on RGB LED
 * @param ms_delay Update interval in milliseconds
 * @param intensity Overall brightness (0-255, where 255 is full brightness)
 */
void LED_Rainbow(int ms_delay, uint8_t intensity);

/**
 * @brief Display pulsing red LED for armed state
 * @param ms_delay Update interval in milliseconds
 * @param intensity Overall brightness (0-255, where 255 is full brightness)
 */
void LED_Armed(int ms_delay, uint8_t intensity);

/**
 * @brief Display test pattern on LED
 * @param ms_delay Update interval in milliseconds
 */
void LED_Test(int ms_delay);

/**
 * @brief Turn off all LEDs
 */
void LED_Off(void);

/**
 * @brief Display yellow (amber) pulsing LED for charging state
 * @param ms_delay Update interval in milliseconds
 * @param intensity Overall brightness (0-255, where 255 is full brightness)
 * @note Green LED is automatically set to 85% of red to prevent overpowering
 */
void LED_Charging(int ms_delay, uint8_t intensity);

/**
 * @brief Test each LED individually
 * @note This is a blocking function with delays
 */
void LED_Test_Individual(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIGHTS_H */
