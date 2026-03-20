/***************************************************************************
 * lights.h
 * created by Sebastian Forenza 2026
 *
 * Header file for LED interface functions
 *
 * LED1 (Red)  = PB3  -> TIM2_CH4 (hardware PWM)
 * LED2 (Green)= PB2  -> TIM2_CH3 (hardware PWM)
 * LED3 (Blue) = PB1  -> GPIO soft-PWM via SysTick
 ***************************************************************************/

#ifndef __LIGHTS_H
#define __LIGHTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---- Software PWM for LED3 (Blue, PB1) ---- */

/**
 * @brief  Must be called once before using any LED function.
 *         Sets up internal soft-PWM state for LED3.
 */
void LED_SoftPWM_Init(void);

/**
 * @brief  Call this from SysTick_Handler() every 1 ms.
 *         Drives the software PWM on PB1 (LED3 / blue).
 */
void LED_SoftPWM_Tick(void);

/**
 * @brief  Clamp the blue LED to a static ON or OFF before entering sleep.
 *
 *         Call right before entering any low-power mode that stops SysTick
 *         (STOP, DEEPSTOP).  This prevents the LED freezing at a random
 *         mid-PWM brightness.  No restore call is needed on wake — the
 *         SysTick-driven PWM resumes automatically.
 */
void LED_SoftPWM_ClampForSleep(void);

/* ---- Public LED API (unchanged interface) ---- */

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
 * @brief Turn off all LEDs
 */
void LED_Off(void);

/**
 * @brief Flash LED with custom color (non-blocking)
 * @param flash_interval_ms Time between on and off in milliseconds
 * @param red Red component (0-255)
 * @param green Green component (0-255)
 * @param blue Blue component (0-255)
 * @param intensity Overall brightness multiplier (0-255, where 255 is full)
 * @note This is non-blocking - call repeatedly in main loop
 */
void LED_Alarm(int flash_interval_ms, uint8_t red, uint8_t green, uint8_t blue, uint8_t intensity);

void LED_Pulse(int duration,
               uint8_t r,
               uint8_t g,
               uint8_t b,
               uint8_t intensity);

#ifdef __cplusplus
}
#endif

#endif /* __LIGHTS_H */
