// Define to prevent recursive inclusion
#ifndef __PUMPMOTOR_H__
#define __PUMPMOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "main.h"
#include <stdint.h>

// Existing motor PWM functions
void StartMotorPWM(void);
void UpdateARR(uint32_t ARR);
uint32_t GetARR(void);
void MotorValuesInit(void);

// ADDED: Stop PWM output after ramp-down is completed.
// Original code could slow the motor down, but did not fully stop PWM.
void StopMotorPWM(void);

// MODIFIED: StartPump and EndPump now return uint8_t.
// Return 0 = still ramping
// Return 1 = ramp finished
// This allows main.c to use a proper state machine.
uint8_t StartPump(void);
uint8_t EndPump(void);

// ADDED: ResetPID clears PID internal memory before entering PID control.
// This was not in the original code.
void ResetPID(void);

// MODIFIED: UpdatePID now uses float error instead of int32_t.
// This supports proportional, integral, and derivative calculation.
// At this stage, placeholder flow error values can be used.
void UpdatePID(float error);

#ifdef __cplusplus
}
#endif

#endif /* __PUMPMOTOR_H__ */
















////Define to prevent recursive inclusion
//#ifndef __PUMPMOTOR_H__
//#define __PUMPMOTOR_H__
//
//#ifdef __cplusplus
//extern "C" {
//#endif
//
//    //INCLUDES
//    #include "main.h"
//
//    //Defines
//    void StartMotorPWM(void);
//    void UpdateARR(uint32_t ARR);
//    uint32_t GetARR(void);
//    void MotorValuesInit(void);
//    void StartPump(void);
//    void EndPump(uint32_t *ARR);
//    void UpdatePID(int32_t error); //need to update error type
//
//#ifdef __cplusplus
//}
//#endif
//
//#endif /*__ PUMPMOTOR_H__ */
