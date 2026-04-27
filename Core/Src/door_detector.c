/***************************************************************************
 * door_detector.c
 *
 * MLC-gated door position detection.
 *
 * The MLC on the LIS2DUX12 already reliably detects motion vs stationary.
 * This module piggybacks on that: when the MLC confirms the device has
 * settled, we compare the current gravity vector to the reference
 * captured at arming time.
 *
 *   |cur − ref|²  >  ref_sq / mult   ⟹   door has moved
 *
 * Because we only read gravity when stationary, the measurement is
 * clean (no dynamic acceleration) and a single sample is sufficient.
 *
 * All math is integer-only (no FPU on Cortex-M0+).
 ***************************************************************************/

#include "door_detector.h"
#include "accelerometer.h"

/***************************************************************************
 * TUNING CONSTANTS
 *
 * Angle-significance multipliers indexed by sensitivity level.
 * The test is:  diff_sq * mult > ref_sq
 *   mult =  5  →  ~25 deg   (LOW)
 *   mult = 15  →  ~15 deg   (MEDIUM)
 *   mult = 52  →  ~8  deg   (HIGH)
 *
 * CLOSE_MULT is the hysteresis band: the door must return within ~5 deg
 * of the reference before we consider it "closed" again.
 ***************************************************************************/
static const int32_t angle_mult_table[3] = { 5, 15, 52 };

#define CLOSE_MULT  130   /* ~5 deg */

/***************************************************************************
 * INTERNAL STATE
 ***************************************************************************/
static struct {
    int16_t  ref[3];
    int32_t  ref_sq;
    uint8_t  ref_valid;
    uint8_t  is_open;       /* 1 = displaced from reference */
    uint8_t  sensitivity;   /* 0-2 */
} dd;

/***************************************************************************
 * PUBLIC API
 ***************************************************************************/

void DoorDetector_Init(void)
{
    dd.ref[0] = dd.ref[1] = dd.ref[2] = 0;
    dd.ref_sq    = 0;
    dd.ref_valid = 0;
    dd.is_open   = 0;
    dd.sensitivity = 1;  /* default MEDIUM */
}

void DoorDetector_CaptureReference(void)
{
    LIS2DUX12_ReadAcceleration(dd.ref);

    dd.ref_sq = (int32_t)dd.ref[0] * dd.ref[0]
              + (int32_t)dd.ref[1] * dd.ref[1]
              + (int32_t)dd.ref[2] * dd.ref[2];

    dd.ref_valid = (dd.ref_sq > 0);
    dd.is_open   = 0;
}

void DoorDetector_SetSensitivity(uint8_t level)
{
    if (level > 2) level = 2;
    dd.sensitivity = level;
}

uint8_t DoorDetector_Check(void)
{
    if (!dd.ref_valid) return DOOR_EVENT_NONE;

    int16_t cur[3];
    LIS2DUX12_ReadAcceleration(cur);

    int32_t dx = (int32_t)cur[0] - dd.ref[0];
    int32_t dy = (int32_t)cur[1] - dd.ref[1];
    int32_t dz = (int32_t)cur[2] - dd.ref[2];
    int32_t diff_sq = dx * dx + dy * dy + dz * dz;

    int32_t mult = angle_mult_table[dd.sensitivity];

    /* Door just opened? (was closed, now displaced beyond threshold) */
    if (!dd.is_open && diff_sq * mult > dd.ref_sq) {
        dd.is_open = 1;
        return DOOR_EVENT_OPENED;
    }

    /* Door just closed? (was open, now back within ~5 deg of reference) */
    if (dd.is_open && diff_sq * CLOSE_MULT <= dd.ref_sq) {
        dd.is_open = 0;
        return DOOR_EVENT_CLOSED;
    }

    return DOOR_EVENT_NONE;
}

uint8_t DoorDetector_IsOpen(void)
{
    return dd.is_open;
}
