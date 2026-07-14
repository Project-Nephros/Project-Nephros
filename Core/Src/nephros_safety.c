/*
 * nephros_safety.c
 *
 *  Created on: Jul 9, 2026
 *      Author: chany
 */

#include "nephros_safety.h"

#include <stdio.h>

/*
 * Internal safety state.
 * These are kept inside this file so main.c does not need to manage fault logic.
 */
static NephrosSystemState system_state = NEPHROS_RUNNING;
static NephrosFaultCode latched_fault = NEPHROS_FAULT_NONE;

/* Pressure must remain out of range for 3 seconds before halt */
static bool pressure_timer_active = false;
static uint32_t pressure_out_start_ms = 0;

/* Sensor log timing */
static uint32_t last_sensor_log_ms = 0;

/* Warning and fan state */
static bool temp_warning_was_active = false;
static bool fan_on = false;

/*
 * Function pointer used to print logs.
 * main.c gives us NephrosUI_Write during NephrosSafety_Init().
 */
static NephrosLogWriter log_write = NULL;


/* Private helper function prototypes */
static void log_event(
    uint32_t now_ms,
    const char *event_name,
    const char *detail
);

static void log_sensor_reading(
    const NephrosSensorData *sensor,
    uint32_t now_ms
);

static bool pressure_out_of_range_long_enough(
    uint16_t pressure,
    uint32_t now_ms
);

static NephrosFaultCode find_critical_fault(
    const NephrosSensorData *sensor,
    uint32_t now_ms
);

static void set_outputs(
    bool warning_led_on,
    bool buzzer_on,
    bool pump_kill_on
);

static void update_fan(float temperature_c);

static NephrosSafetyOutput make_output(void);


/*
 * Initialises the safety system.
 *
 * This clears fault state, turns off alarm outputs,
 * stores the UART logging function, and turns off the fan.
 */
void NephrosSafety_Init(NephrosLogWriter writer)
{
    log_write = writer;

    system_state = NEPHROS_RUNNING;
    latched_fault = NEPHROS_FAULT_NONE;

    pressure_timer_active = false;
    pressure_out_start_ms = 0;

    last_sensor_log_ms = 0;
    temp_warning_was_active = false;
    fan_on = false;

    set_outputs(false, false, false);

    HAL_GPIO_WritePin(
        FAN_GPIO_Port,
        FAN_Pin,
        GPIO_PIN_RESET
    );
}


/*
 * Main safety update function.
 *
 * main.c should call this every loop.
 * It checks temperature, pressure, air, fan, alarm LED, buzzer, pump kill,
 * and returns LCD instructions back to main.c.
 */
NephrosSafetyOutput NephrosSafety_Update(
    const NephrosSensorData *sensor,
    bool recheck_button_pressed,
    uint32_t now_ms
)
{
    NephrosSafetyOutput output;
    NephrosFaultCode current_fault;
    bool temp_warning_now;

    output = make_output();

    if (sensor == NULL)
    {
        return output;
    }

    /*
     * Fan and logging run every loop, even during halted state.
     */
    update_fan(sensor->temperature_c);
    log_sensor_reading(sensor, now_ms);

    /*
     * Check for current critical faults:
     * air, severe temperature, or pressure out of range for 3 seconds.
     */
    current_fault = find_critical_fault(sensor, now_ms);

    /*
     * HALTED MODE:
     * - warning LED on
     * - buzzer on
     * - pump kill on
     * - button means "re-check"
     */
    if (system_state == NEPHROS_HALTED)
    {
        set_outputs(true, true, true);

        output.state = system_state;
        output.fault = latched_fault;
        output.halted = true;
        output.can_cycle_lcd_view = false;

        if (recheck_button_pressed)
        {
            log_event(now_ms, "RECHECK_REQUESTED", "");

            current_fault = find_critical_fault(sensor, now_ms);

            if (current_fault == NEPHROS_FAULT_NONE)
            {
                system_state = NEPHROS_RUNNING;
                latched_fault = NEPHROS_FAULT_NONE;

                set_outputs(false, false, false);

                log_event(
                    now_ms,
                    "RESUMED",
                    "MANUAL_RECHECK_OK"
                );

                output.state = system_state;
                output.fault = NEPHROS_FAULT_NONE;
                output.halted = false;

                output.resume_requested = true;

                output.lcd_message_valid = true;
                output.lcd_line1 = "SAFETY CHECK";
                output.lcd_line2 = "RESUMED";
                output.message_hold_ms = 750U;

                output.force_normal_lcd_redraw = true;
                output.can_cycle_lcd_view = true;
            }
            else
            {
                latched_fault = current_fault;

                log_event(
                    now_ms,
                    "RECHECK_FAILED",
                    NephrosSafety_FaultName(current_fault)
                );

                output.fault = current_fault;
                output.lcd_message_valid = true;
                output.lcd_line1 = "FAULT STILL ON";
                output.lcd_line2 = NephrosSafety_FaultName(current_fault);
                output.can_cycle_lcd_view = false;
            }
        }

        return output;
    }

    /*
     * RUNNING MODE:
     * Critical fault has highest priority.
     * If there is a critical fault, latch halt immediately.
     */
    if (current_fault != NEPHROS_FAULT_NONE)
    {
        system_state = NEPHROS_HALTED;
        latched_fault = current_fault;

        /*
         * Halt state:
         * LED on + buzzer on + pump kill on.
         */
        set_outputs(true, true, true);

        log_event(
            now_ms,
            "HALT",
            NephrosSafety_FaultName(current_fault)
        );

        output.state = system_state;
        output.fault = current_fault;
        output.halted = true;

        output.lcd_message_valid = true;
        output.lcd_line1 = "SYSTEM HALTED";
        output.lcd_line2 = NephrosSafety_FaultName(current_fault);

        output.can_cycle_lcd_view = false;
        return output;
    }

    /*
     * WARNING-ONLY MODE:
     * Temperature is outside 35-38 C but not severe enough to halt.
     *
     * Warning-only rule:
     * - LED on
     * - buzzer off
     * - pump kill off
     */
    temp_warning_now =
        NephrosSafety_TemperatureWarning(sensor->temperature_c);

    if (temp_warning_now)
    {
        set_outputs(true, false, false);

        output.warning_active = true;
        output.can_cycle_lcd_view = false;

        if (!temp_warning_was_active)
        {
            log_event(
                now_ms,
                "TEMP_WARNING_ON",
                "OUTSIDE_35_TO_38C"
            );

            output.lcd_message_valid = true;
            output.lcd_line1 = "TEMP ALERT";
            output.lcd_line2 = "OUTSIDE 35-38C";
        }

        temp_warning_was_active = true;
        return output;
    }

    /*
     * NORMAL MODE:
     * No warning and no halt.
     *
     * Outputs:
     * - LED off
     * - buzzer off
     * - pump kill off
     */
    set_outputs(false, false, false);

    if (temp_warning_was_active)
    {
        log_event(
            now_ms,
            "TEMP_WARNING_CLEARED",
            ""
        );

        output.force_normal_lcd_redraw = true;
    }

    temp_warning_was_active = false;

    output.state = NEPHROS_RUNNING;
    output.fault = NEPHROS_FAULT_NONE;
    output.warning_active = false;
    output.halted = false;
    output.can_cycle_lcd_view = true;

    return output;
}


/*
 * Returns true when temperature is outside the normal 35-38 C range.
 *
 * This is warning-level only.
 * Severe temperature is checked separately.
 */
bool NephrosSafety_TemperatureWarning(float temperature_c)
{
    return (temperature_c < TEMP_NORMAL_LOW_C ||
            temperature_c > TEMP_NORMAL_HIGH_C);
}


/*
 * Checks whether temperature is severely out of range.
 *
 * Returns:
 * - NEPHROS_FAULT_TEMP_LOW
 * - NEPHROS_FAULT_TEMP_HIGH
 * - NEPHROS_FAULT_NONE
 */
NephrosFaultCode NephrosSafety_CheckTemperature(float temperature_c)
{
    if (temperature_c <= TEMP_HALT_LOW_C)
    {
        return NEPHROS_FAULT_TEMP_LOW;
    }

    if (temperature_c >= TEMP_HALT_HIGH_C)
    {
        return NEPHROS_FAULT_TEMP_HIGH;
    }

    return NEPHROS_FAULT_NONE;
}


/*
 * Checks whether pressure is instantly outside the pressure window.
 *
 * This does not halt by itself.
 * The pressure must stay out of range for 3 seconds before a halt.
 */
bool NephrosSafety_PressureInstantOutOfRange(uint16_t pressure)
{
    return (pressure < PRESSURE_LOW_LIMIT ||
            pressure > PRESSURE_HIGH_LIMIT);
}


/*
 * Converts a fault enum into a readable string for LCD and logs.
 */
const char *NephrosSafety_FaultName(NephrosFaultCode fault)
{
    switch (fault)
    {
        case NEPHROS_FAULT_AIR:
            return "AIR DETECTED";

        case NEPHROS_FAULT_TEMP_LOW:
            return "TEMP TOO LOW";

        case NEPHROS_FAULT_TEMP_HIGH:
            return "TEMP TOO HIGH";

        case NEPHROS_FAULT_PRESSURE:
            return "PRESSURE FAULT";

        default:
            return "NO FAULT";
    }
}


/*
 * Creates a blank/default safety output.
 *
 * This prevents uninitialised return values.
 */
static NephrosSafetyOutput make_output(void)
{
    NephrosSafetyOutput output;

    output.state = system_state;
    output.fault = latched_fault;

    output.warning_active = false;
    output.halted = (system_state == NEPHROS_HALTED);
    output.resume_requested = false;

    output.lcd_message_valid = false;
    output.lcd_line1 = "";
    output.lcd_line2 = "";
    output.message_hold_ms = 0U;

    output.can_cycle_lcd_view = false;
    output.force_normal_lcd_redraw = false;

    return output;
}


/*
 * Finds the current critical fault.
 *
 * Priority:
 * 1. Air detected: immediate halt
 * 2. Severe temperature: immediate halt
 * 3. Pressure out of range for 3 seconds: halt
 */
static NephrosFaultCode find_critical_fault(
    const NephrosSensorData *sensor,
    uint32_t now_ms
)
{
    NephrosFaultCode temp_fault;

    if (sensor->air_detected)
    {
        return NEPHROS_FAULT_AIR;
    }

    temp_fault =
        NephrosSafety_CheckTemperature(sensor->temperature_c);

    if (temp_fault != NEPHROS_FAULT_NONE)
    {
        return temp_fault;
    }

    if (pressure_out_of_range_long_enough(
            sensor->pressure,
            now_ms
        ))
    {
        return NEPHROS_FAULT_PRESSURE;
    }

    return NEPHROS_FAULT_NONE;
}


/*
 * Implements the 3-second pressure confirmation timer.
 *
 * If pressure goes out of range, the timer starts.
 * If pressure returns to normal before 3 seconds, the timer resets.
 * If pressure stays out of range for 3 seconds, it returns true.
 */
static bool pressure_out_of_range_long_enough(
    uint16_t pressure,
    uint32_t now_ms
)
{
    if (!NephrosSafety_PressureInstantOutOfRange(pressure))
    {
        if (pressure_timer_active)
        {
            log_event(
                now_ms,
                "PRESSURE_BACK_IN_RANGE",
                ""
            );
        }

        pressure_timer_active = false;
        return false;
    }

    if (!pressure_timer_active)
    {
        pressure_timer_active = true;
        pressure_out_start_ms = now_ms;

        log_event(
            now_ms,
            "PRESSURE_OUT_OF_RANGE_START",
            ""
        );
    }

    return ((uint32_t)(now_ms - pressure_out_start_ms)
            >= PRESSURE_CONFIRM_MS);
}


/*
 * Controls the warning LED, buzzer, and pump kill signal.
 *
 * Warning only:
 *   LED on, buzzer off, pump kill off
 *
 * Halt:
 *   LED on, buzzer on, pump kill on
 */
static void set_outputs(
    bool warning_led_on,
    bool buzzer_on,
    bool pump_kill_on
)
{
    HAL_GPIO_WritePin(
        ALARM_LED_GPIO_Port,
        ALARM_LED_Pin,
        warning_led_on ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        BUZZER_GPIO_Port,
        BUZZER_Pin,
        buzzer_on ? GPIO_PIN_SET : GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        PUMP_KILL_GPIO_Port,
        PUMP_KILL_Pin,
        pump_kill_on ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}


/*
 * Controls the 12 V fan through a MOSFET.
 *
 * Fan turns on above FAN_ON_TEMP_C.
 * Fan turns off only after cooling below FAN_OFF_TEMP_C.
 *
 * This hysteresis prevents rapid ON/OFF switching around the limit.
 */
static void update_fan(float temperature_c)
{
    if (!fan_on && temperature_c > FAN_ON_TEMP_C)
    {
        fan_on = true;
    }
    else if (fan_on && temperature_c <= FAN_OFF_TEMP_C)
    {
        fan_on = false;
    }

    HAL_GPIO_WritePin(
        FAN_GPIO_Port,
        FAN_Pin,
        fan_on ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}


/*
 * Logs one event with a timestamp.
 *
 * Example:
 * EVENT,5230,HALT,TEMP TOO HIGH
 */
static void log_event(
    uint32_t now_ms,
    const char *event_name,
    const char *detail
)
{
    char line[128];

    if (log_write == NULL)
    {
        return;
    }

    snprintf(
        line,
        sizeof(line),
        "EVENT,%lu,%s,%s\r\n",
        (unsigned long)now_ms,
        event_name,
        detail
    );

    log_write(line);
}


/*
 * Logs sensor readings periodically.
 *
 * Example:
 * DATA,1000,36.5,150,0,RUNNING
 *
 * Meaning:
 * timestamp, temperature, pressure, air_detected, system_state
 */
static void log_sensor_reading(
    const NephrosSensorData *sensor,
    uint32_t now_ms
)
{
    char line[128];

    if (log_write == NULL || sensor == NULL)
    {
        return;
    }

    if ((uint32_t)(now_ms - last_sensor_log_ms)
        < SENSOR_LOG_PERIOD_MS)
    {
        return;
    }

    last_sensor_log_ms = now_ms;

    snprintf(
        line,
        sizeof(line),
        "DATA,%lu,%.1f,%u,%u,%s\r\n",
        (unsigned long)now_ms,
        sensor->temperature_c,
        (unsigned int)sensor->pressure,
        sensor->air_detected ? 1U : 0U,
        (system_state == NEPHROS_HALTED) ? "HALTED" : "RUNNING"
    );

    log_write(line);
}
