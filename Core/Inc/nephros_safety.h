/*
 * nephros_safety.h
 *
 *  Created on: Jul 9, 2026
 *      Author: chany
 */

#ifndef INC_NEPHROS_SAFETY_H_
#define INC_NEPHROS_SAFETY_H_

#include "main.h"
#include "nephros_types.h"
#include <stdbool.h>
#include <stdint.h>

/* ---------------- Safety thresholds ---------------- */

/* Normal dialysate temperature range */
#define TEMP_NORMAL_LOW_C       35.0f
#define TEMP_NORMAL_HIGH_C      38.0f

/*
 * Prototype severe-temperature limits.
 * Team should confirm final values.
 */
#define TEMP_HALT_LOW_C         33.0f
#define TEMP_HALT_HIGH_C        40.0f

/*
 * Demo pressure window.
 * Replace with real calibrated arterial/venous windows later.
 */
#define PRESSURE_LOW_LIMIT      100U
#define PRESSURE_HIGH_LIMIT     200U
#define PRESSURE_CONFIRM_MS     3000U

#define SENSOR_LOG_PERIOD_MS    1000U

/* Fan hysteresis */
#define FAN_ON_TEMP_C           38.0f
#define FAN_OFF_TEMP_C          37.5f

/* ---------------- Output pins ---------------- */
// MODIFIED: NOW FOUND IN MAIN.H - meant to allow easier change through .ioc file.

typedef void (*NephrosLogWriter)(const char *text);

typedef struct
{
    NephrosSystemState state;
    NephrosFaultCode fault;

    bool warning_active;
    bool halted;
    bool resume_requested;

    bool lcd_message_valid;
    const char *lcd_line1;
    const char *lcd_line2;
    uint32_t message_hold_ms;

    bool can_cycle_lcd_view;
    bool force_normal_lcd_redraw;
} NephrosSafetyOutput;

void NephrosSafety_Init(NephrosLogWriter writer);

NephrosSafetyOutput NephrosSafety_Update(
    const NephrosSensorData *sensor,
    bool recheck_button_pressed,
    uint32_t now_ms
);

bool NephrosSafety_TemperatureWarning(float temperature_c);
NephrosFaultCode NephrosSafety_CheckTemperature(float temperature_c);
bool NephrosSafety_PressureInstantOutOfRange(uint16_t pressure);

const char *NephrosSafety_FaultName(NephrosFaultCode fault);

#endif /* INC_NEPHROS_SAFETY_H_ */
