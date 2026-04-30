/***************************************************************************
 * lights.h
 * created by Sebastian Forenza 2026
 *
 * Header file for LED interface functions
 *
 * LED1 (Red)  = PB3  -> TIM2_CH4 (hardware PWM)
 * LED2 (Green)= PB2  -> TIM2_CH3 (hardware PWM)
 * LED3 (Blue) = PB7  -> TIM2_CH2 (hardware PWM)
 ***************************************************************************/

#ifndef __LIGHTS_H
#define __LIGHTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ---- Public LED API ---- */

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
 * @brief Set a solid (non-animated) color — no internal state, safe to call rapidly
 */
void LED_Solid(uint8_t r, uint8_t g, uint8_t b, uint8_t intensity);

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

/**
 * @brief Force LED_Pulse to restart from dark on its next call.
 */
void LED_Pulse_Reset(void);

/**
 * @brief Begin the cable plug-in LED transition: capture the current LED
 *        color and fade it to black over 500 ms, then hold off for 250 ms.
 *        After completion, the next LED routine starts visually from dark.
 *        Call LED_PlugIn_Tick() repeatedly while LED_PlugIn_InProgress()
 *        returns true, instead of the normal LED routine.
 */
void LED_PlugIn_Start(void);

/**
 * @brief Drive the plug-in transition. Non-blocking; call from main loop.
 */
void LED_PlugIn_Tick(void);

/**
 * @brief True while the plug-in transition is still running.
 */
bool LED_PlugIn_InProgress(void);

/**
 * @brief Begin the cable unplug LED transition: capture the current LED
 *        color and fade it to black over 100 ms. After completion, control
 *        is handed back so the natural state-loop LED routine resumes.
 *        Call LED_PlugOut_Tick() repeatedly while LED_PlugOut_InProgress()
 *        returns true, instead of the normal LED routine.
 */
void LED_PlugOut_Start(void);

/**
 * @brief Drive the unplug transition. Non-blocking; call from main loop.
 */
void LED_PlugOut_Tick(void);

/**
 * @brief True while the unplug transition is still running.
 */
bool LED_PlugOut_InProgress(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIGHTS_H */
