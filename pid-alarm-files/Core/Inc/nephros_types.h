/*
 * nephros_types.h
 *
 *  Created on: Jul 9, 2026
 *      Author: chany
 */

#ifndef INC_NEPHROS_TYPES_H_
#define INC_NEPHROS_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float temperature_c;
    uint16_t pressure;
    bool air_detected;
} NephrosSensorData;

typedef struct
{
    uint32_t volume_ml;
    uint32_t duration_min;
    float required_flow_ml_min;
} NephrosSetup;

typedef enum
{
    NEPHROS_RUNNING = 0,
    NEPHROS_HALTED
} NephrosSystemState;

typedef enum
{
    NEPHROS_FAULT_NONE = 0,
    NEPHROS_FAULT_AIR,
    NEPHROS_FAULT_TEMP_LOW,
    NEPHROS_FAULT_TEMP_HIGH,
    NEPHROS_FAULT_PRESSURE
} NephrosFaultCode;

#endif /* INC_NEPHROS_TYPES_H_ */
