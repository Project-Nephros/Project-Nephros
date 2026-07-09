// Define to prevent recursive inclusion
#ifndef __STATEMACHINE_H__
#define __STATEMACHINE_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "main.h"
#include <stdint.h>

//Variables
/*Motor state machine.
 *
 * Original code declared Start / Maintenance / End states,
 * but the state variable was not actually used.
 *
 * This version uses the state machine to run:
 * START_RAMP -> PID_TEST -> END_RAMP -> STOPPED
 */
typedef enum
{
    STATE_START_MOTOR_RAMP = 0,
    STATE_MOTOR_PID_TEST,
    STATE_MOTOR_END_RAMP,
    STATE_MOTOR_STOPPED,
    STATE_EMERGENCY_STOP
} State;

//Functions
void RunStateMachine(uint32_t currentTick);
void SetState(State state);
State GetState(void);


#ifdef __cplusplus
}
#endif

#endif /* __STATEMACHINE_H__ */