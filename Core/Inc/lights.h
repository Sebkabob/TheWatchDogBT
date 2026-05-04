/***************************************************************************
 * lights.h
 * created by Sebastian Forenza 2026
 *
 * Public API for onboard RGB LED control. All channels are TIM2 hardware
 * PWM (active-low):
 *   LED1 (Red)   = PB3  → TIM2_CH4
 *   LED2 (Green) = PB2  → TIM2_CH3
 *   LED3 (Blue)  = PB7  → TIM2_CH2
 ***************************************************************************/

#ifndef __LIGHTS_H
#define __LIGHTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Animated colour cycle. ms_delay = step interval; intensity 0..255.
void LED_Rainbow(int ms_delay, uint8_t intensity);

// Slow red breathing pulse for armed/locked states.
void LED_Armed(int ms_delay, uint8_t intensity);

void LED_Off(void);

// Static colour (no animation state). Safe to call rapidly from main loop.
void LED_Solid(uint8_t r, uint8_t g, uint8_t b, uint8_t intensity);

// Non-blocking flash. Toggles every flash_interval_ms.
void LED_Alarm(int flash_interval_ms, uint8_t red, uint8_t green, uint8_t blue, uint8_t intensity);

// Soft breathing pulse over <duration> ms (full 0→255→0 sweep).
void LED_Pulse(int duration, uint8_t r, uint8_t g, uint8_t b, uint8_t intensity);

// Force the next LED_Pulse() to restart from dark.
void LED_Pulse_Reset(void);

// Plug-in LED transition: 500 ms fade-to-black, 250 ms hold off, then resume.
void LED_PlugIn_Start(void);
void LED_PlugIn_Tick(void);
bool LED_PlugIn_InProgress(void);

// Plug-out LED transition: 100 ms fade-to-black, then resume.
void LED_PlugOut_Start(void);
void LED_PlugOut_Tick(void);
bool LED_PlugOut_InProgress(void);

/***************************************************************************
 * Persisted LED brightness scalar (1..255, default 255). Applied as a
 * multiplier to every "status" LED call (armed pulse, stabilizing pulse,
 * alarm flash, find-my, connected-idle rainbow). Charging-status and the
 * drain-mode diagnostic bypass this and render at full brightness.
 *
 * EEPROM record at 0x1A: magic 0xC4 + value byte.
 ***************************************************************************/

#define LED_BRIGHTNESS_DEFAULT      255u
#define LED_BRIGHTNESS_MIN          1u
#define LED_BRIGHTNESS_MAX          255u
#define EEPROM_LED_BRIGHTNESS_ADDR  0x1A
#define EEPROM_LED_BRIGHTNESS_LEN   2
#define EEPROM_LED_BRIGHTNESS_MAGIC 0xC4

void    LedBrightness_Init(void);
uint8_t LedBrightness_Get(void);
// Clamps to [1, 255] (zero maps to 1) and persists. Returns the stored
// value. Skips the EEPROM write when unchanged.
uint8_t LedBrightness_Set(uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* __LIGHTS_H */
