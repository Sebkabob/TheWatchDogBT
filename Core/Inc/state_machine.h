
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "main.h"

// State definitions
typedef enum {
    STATE_SLEEP,
    STATE_ALARM_ACTIVE,
	STATE_ARMED,
	STATE_DISCONNECTED_IDLE,
    STATE_CONNECTED_IDLE
} SystemState_t;

// Global state variables
extern volatile SystemState_t currentState;
extern volatile uint8_t deviceState;

// Function prototypes
void StateMachine_Init(void);
void StateMachine_Run(void);
void StateMachine_ChangeState(SystemState_t newState);

#endif
