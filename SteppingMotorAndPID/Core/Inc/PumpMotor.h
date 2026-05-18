//Define to prevent recursive inclusion 
#ifndef __PUMPMOTOR_H__
#define __PUMPMOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

    //INCLUDES
    #include "main.h"

    //Defines
    void StartMotorPWM(void);
    void UpdateARR(uint32_t ARR);
    uint32_t GetARR(void);
    void MotorValuesInit(void);
    void StartPump(void);
    void EndPump(uint32_t *ARR);
    void UpdatePID(int32_t error); //need to update error type

#ifdef __cplusplus
}
#endif

#endif /*__ PUMPMOTOR_H__ */
