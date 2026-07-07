// Define to prevent recursive inclusion
#ifndef __ALARMSYSTEM_H__
#define __ALARMSYSTEM_H__

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "main.h"
#include <stdint.h>

int Temp(float temp);
int Pressure(float pressure, int pressureTimer);
int Air(int air);
void Normal(float temp, int pressureTimer, int air);

#ifdef __cplusplus
}
#endif

#endif /* __ALARMSYSTEM_H__ */
