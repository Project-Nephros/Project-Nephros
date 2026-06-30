// Define to prevent recursive inclusion
#ifndef __ALARMSYSTEM_H__
#define __ALARMSYSTEM_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "main.h"
#include <stdint.h>

void Temp(float temp);
void Pressure(float pressure, int pressureTimer);
void Air(int air);
void Normal(float temp, int pressureTimer, int air);

#ifdef __cplusplus
}
#endif

#endif /* __PUMPMOTOR_H__ */