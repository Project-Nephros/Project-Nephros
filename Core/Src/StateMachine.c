// Includes
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "StateMachine.h"
#include "PumpMotor.h"

// Private Includes
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Placeholder control settings for this week's deliverable.
 * These values are used because the design team has not started moving fluid yet. */
#define TARGET_FLOW_ML_MIN          250
#define PID_TEST_DURATION_MS        35000U   // ADDED: extended from 30000U so we can see PID response after the 25s setpoint restore
#define CONTROL_UPDATE_INTERVAL_MS  100U
#define UART_PRINT_INTERVAL_MS      500U

/* Test scenario constants for dummy-value PID testing.
 *
 *   t = 15s : setpoint changes from 250 to 180 mL/min
 *   t = 20s : positive disturbance of +50 mL/min starts
 *   t = 25s : positive disturbance ends AND setpoint restored to 250 mL/min
 *   t = 35s : test ends
 */
#define TEST_ALT_TARGET_ML_MIN      180
#define TEST_SETPOINT_CHANGE_MS     15000U
#define TEST_SETPOINT_RESTORE_MS    25000U
#define TEST_POS_DISTURB_START_MS   20000U
#define TEST_POS_DISTURB_END_MS     25000U
#define TEST_POS_DISTURB_VALUE      50

/*These values are only used for placeholder flow simulation.
 * They should match the slow and fast ARR values in PumpMotor.c.
 */
#define PLACEHOLDER_SLOW_ARR        999U
#define PLACEHOLDER_FAST_ARR        51U
#define PLACEHOLDER_MAX_FLOW        300

//PID Variables
static State currentState = STATE_START_MOTOR_RAMP;

static uint32_t lastPIDSystemCheck = 0;

static uint32_t pidStartTick = 0U;
static uint32_t lastControlUpdateTick = 0U;
static uint32_t lastUartPrintTick = 0U;

static int32_t targetFlow = TARGET_FLOW_ML_MIN;
static int32_t measuredFlow = 0;
static int32_t flowError = 0;

static uint8_t stoppedMessagePrinted = 0U;
//static uint8_t emergencyMessagePrinted = 0U;

//Prototype Functions
static int32_t GetPlaceholderMeasuredFlow(uint32_t arr, uint32_t elapsedMs); //will be removed later
static const char* GetStateName(State state);
static void PrintMotorStatus(void);

//helper Functions
static int32_t GetPlaceholderMeasuredFlow(uint32_t arr, uint32_t elapsedMs)
{
    /*
     * ADDED:
     * Placeholder flow model.
     *
     * This function simulates a measured flow value using the current ARR.
     * It is only for PID testing before the real flow sensor is available.
     *
     * Smaller ARR means higher speed, so simulated flow increases.
     *
     * Later, this whole function should be replaced by real flow sensor reading.
     */

    int32_t simulatedFlow = 0;

    if (arr >= PLACEHOLDER_SLOW_ARR)
    {
        simulatedFlow = 0;
    }
    else if (arr <= PLACEHOLDER_FAST_ARR)
    {
        simulatedFlow = PLACEHOLDER_MAX_FLOW;
    }
    else
    {
        uint32_t arrRange = PLACEHOLDER_SLOW_ARR - PLACEHOLDER_FAST_ARR;
        uint32_t speedPosition = PLACEHOLDER_SLOW_ARR - arr;

        simulatedFlow = (int32_t)((speedPosition * PLACEHOLDER_MAX_FLOW) / arrRange);
    }

    /*
     * ADDED:
     * Artificial disturbance for PID testing.
     *
     * Between 10s and 18s, we reduce the simulated measured flow.
     * This allows us to check whether PID reacts by increasing motor speed.
     */
    if ((elapsedMs > 10000U) && (elapsedMs < 18000U))
    {
        simulatedFlow -= 30;
    }

    /*
     * ADDED:
     * Positive disturbance for the test scenario.
     * Between 20s and 25s, add a positive offset to simulated flow.
     * This tests whether the PID slows the motor down (raises ARR) to reject overshoot.
     */
    if ((elapsedMs >= TEST_POS_DISTURB_START_MS) && (elapsedMs < TEST_POS_DISTURB_END_MS))
    {
    	simulatedFlow += TEST_POS_DISTURB_VALUE;
    }


    if (simulatedFlow < 0)
    {
        simulatedFlow = 0;
    }

    return simulatedFlow;
}

static const char* GetStateName(State state)
{
    /*Convert state enum to readable text for UART debug messages.*/
    switch (state)
    {
        case STATE_START_MOTOR_RAMP:
            return "START_RAMP";

        case STATE_MOTOR_PID_TEST:
            return "PID_TEST";

        case STATE_MOTOR_END_RAMP:
            return "END_RAMP";

        case STATE_MOTOR_STOPPED:
            return "STOPPED";
        
        case STATE_EMERGENCY_STOP:
            return "EMERGENCY_STOP";

        default:
            return "UNKNOWN";
    }
}

static void PrintMotorStatus(void)
{
    /*UART debug print.
     *
     * This prints the current state, target flow, placeholder measured flow,
     * flow error, and ARR value.
     *
     * This is useful for proving that the motor speed is being controlled
     * based on the PID error.
     */

    char buffer[150];

    int len = snprintf(
        buffer,
        sizeof(buffer),
        "State=%s, Target=%ld mL/min, Measured=%ld mL/min, Error=%ld, ARR=%lu\r\n",
        GetStateName(currentState),
        (long)targetFlow,
        (long)measuredFlow,
        (long)flowError,
        (unsigned long)GetARR()
    );

    if (len > 0)
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, (uint16_t)strlen(buffer), 100);
    }
}

void SetState(State state){
    currentState = state;
}

State GetState(void){
    return currentState;
}

//Functions
void RunStateMachine(uint32_t currentTick){

    if (currentTick - lastPIDSystemCheck < 1){
        return;
    }

    lastPIDSystemCheck = currentTick;

    switch (currentState)
      {
        case STATE_START_MOTOR_RAMP:
        {
          /*Periodic UART print during ramp-up so we can see ARR decreasing. */
          if ((currentTick - lastUartPrintTick) >= UART_PRINT_INTERVAL_MS)
          {
            PrintMotorStatus(); //check we can do this
            lastUartPrintTick = currentTick;
          }

          if (StartPump())
          {
            ResetPID();
            pidStartTick = currentTick;
            lastControlUpdateTick = currentTick;
            lastUartPrintTick = currentTick;
            currentState = STATE_MOTOR_PID_TEST;
            PrintMotorStatus();
          }
          break;
        }
        
        case STATE_MOTOR_PID_TEST:
        {
            /*PID testing stage. measuredFlow comes fromGetPlaceholderMeasuredFlow().
            * Later, replace this placeholder value with real flow sensor data.
            */
            uint32_t elapsedMs = currentTick - pidStartTick;\

            /*Scheduled setpoint changes for the test scenario.
            * 0–15s:  target = 250
            * 15–25s: target = 180
            * 25s+ :  target = 250
            */

            if (elapsedMs >= TEST_SETPOINT_RESTORE_MS)
            {
              targetFlow = TARGET_FLOW_ML_MIN;
            }
            else if (elapsedMs >= TEST_SETPOINT_CHANGE_MS)
            {
              targetFlow = TEST_ALT_TARGET_ML_MIN;
            }
            else
            {
              targetFlow = TARGET_FLOW_ML_MIN;
            }

            if ((currentTick - lastControlUpdateTick) >= CONTROL_UPDATE_INTERVAL_MS)
            {
                measuredFlow = GetPlaceholderMeasuredFlow(GetARR(), elapsedMs);
                flowError = targetFlow - measuredFlow;

                /*Motor speed is controlled based on PID error.*/
                UpdatePID((float)flowError);

                lastControlUpdateTick = currentTick;
            }

            if ((currentTick - lastUartPrintTick) >= UART_PRINT_INTERVAL_MS)
            {
                PrintMotorStatus();
                lastUartPrintTick = currentTick;
            }

            /*Automatically finish the placeholder PID test after 30 seconds.*/
            if (elapsedMs >= PID_TEST_DURATION_MS)
            {
                currentState = STATE_MOTOR_END_RAMP;
                PrintMotorStatus();
            }

            break;
        }

        case STATE_MOTOR_END_RAMP:
        {
            /* ADDED: Periodic UART print during ramp-down so we can see ARR increasing. */
            if ((currentTick - lastUartPrintTick) >= UART_PRINT_INTERVAL_MS)
            {
                PrintMotorStatus();
                lastUartPrintTick = currentTick;
            }

            if (EndPump())
            {
                currentState = STATE_MOTOR_STOPPED;
                PrintMotorStatus();
            }
            break;
        }

        case STATE_MOTOR_STOPPED:
        {
            /*
            * ADDED:
            * Final stopped state.
            * Print the stopped message only once.
            */
            if (!stoppedMessagePrinted)
            {
                char stopMessage[] = "Pump motor PID test completed. Motor stopped.\r\n";

                HAL_UART_Transmit(
                    &huart2,
                    (uint8_t*)stopMessage,
                    strlen(stopMessage),
                    100
                );

                stoppedMessagePrinted = 1U;
            }

            break;
        }

        case STATE_EMERGENCY_STOP:
        {
          StopMotorPWM();
          HAL_GPIO_WritePin(GPIOB, Alarm_LED_Pin | Buzzer_Pin, GPIO_PIN_SET);
          PrintMotorStatus();
          // if (a reset button pressed or whatever) 
          // {
          //   MotorValuesInit();
          //   StartMotorPWM();
          //   motorstate = MOTOR_STATE_START_RAMP;
          // }

          // if (decide not to reset)
          // {
          //   end session or whatever. motorstate = MOTOR_STATE_STOP_RAMP; -> but might want printed message to be ended session w emergency stop
          // }
          break;
        }

        default:
        {
            /*Safety fallback.
            */
            StopMotorPWM();
            currentState = STATE_MOTOR_STOPPED;
            break;
        }
      }
}



