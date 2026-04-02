/***************************************************************************
 * door_detector.h
 *
 * MLC-gated door position detection for LIS2DUX12.
 *
 * Instead of trying to detect rotation in real-time (noisy), this module
 * waits until the MLC confirms the device is stationary, then compares
 * the current gravity vector to the armed reference.  A significant
 * change means the door has moved to a new position.
 *
 * Usage:
 *   DoorDetector_CaptureReference()   — call when arming (door closed)
 *   DoorDetector_Check()              — call when MLC says stationary
 *   DoorDetector_IsOpen()             — query current door position
 ***************************************************************************/

#ifndef INC_DOOR_DETECTOR_H_
#define INC_DOOR_DETECTOR_H_

#include "main.h"

/* Return values from DoorDetector_Check() — match MotionType_t codes */
#define DOOR_EVENT_NONE    0
#define DOOR_EVENT_OPENED  6   /* MOTION_TYPE_DOOR_OPEN  */
#define DOOR_EVENT_CLOSED  7   /* MOTION_TYPE_DOOR_CLOSE */

void    DoorDetector_Init(void);
void    DoorDetector_CaptureReference(void);
void    DoorDetector_SetSensitivity(uint8_t level);

/**
 * @brief  Check door position against armed reference.
 *         Only call when the MLC confirms stationary (no dynamic accel).
 * @return DOOR_EVENT_OPENED  if the door just moved away from reference
 *         DOOR_EVENT_CLOSED  if the door just returned to reference
 *         DOOR_EVENT_NONE    if nothing changed
 */
uint8_t DoorDetector_Check(void);

/**
 * @brief  Returns 1 if the door is currently displaced from reference.
 */
uint8_t DoorDetector_IsOpen(void);

#endif /* INC_DOOR_DETECTOR_H_ */
