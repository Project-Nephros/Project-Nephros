// Includes
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "PumpMotor.h"

// Private Includes
#include <string.h>
#include <stdio.h>
#include <math.h>

/*
 * PumpMotor.c
 *
 * Purpose:
 * This file controls the stepper motor speed by changing the PWM frequency.
 * The PWM frequency is controlled by changing TIM2 ARR.
 *
 * Important relationship:
 * Smaller ARR  -> higher PWM frequency -> faster motor
 * Larger ARR   -> lower PWM frequency  -> slower motor
 *
 * This version is built on top of the original teammate code.
 */

/* -------------------------------------------------------------------------- */
/* Motor ramp variables                                                       */
/* -------------------------------------------------------------------------- */

/*
 * Existing idea from original code:
 * kTimeUpdateInterval controls how often the ramp updates.
 *
 * ADDED/MODIFIED:
 * Values are now marked as const because they are fixed tuning values.
 */
static const uint32_t kTimeUpdateIntervalMs = 20U;

/*
 * Existing idea from original code:
 * This is the final ARR after start ramping.
 *
 * NOTE:
 * If TIM2 ticks every 1 us:
 * ARR = 52 - 1 gives around 19.23 kHz.
 * With 1/32 microstepping and 6400 steps/rev, this is around 180 rpm.
 *
 * This is still a placeholder and should be tuned later.
 */
static const uint32_t kFinalStartARR = 52U - 1U;

/*
 * Existing idea from original code:
 * Larger ramp factor means slower and smoother ramp.
 */
static const uint32_t kRampFactor = 50U;

/*
 * Existing idea from original code:
 * This is the slow ARR used at start/end.
 *
 * MODIFIED:
 * Original code used 1000 directly.
 * Here we use 1000 - 1 to follow normal timer ARR convention.
 */
static const uint32_t kStartAndEndARR = 1000U - 1U;

/*
 * ADDED:
 * Safety bounds for ARR.
 * This prevents PID or other functions from setting an unsafe ARR value.
 */
static const uint32_t kMinARR = 52U - 1U;
static const uint32_t kMaxARR = 1000U - 1U;

/* -------------------------------------------------------------------------- */
/* PID variables                                                              */
/* -------------------------------------------------------------------------- */

/*
 * MODIFIED:
 * Original code used:
 * static int32_t kP = -0.1;
 *
 * That does not work correctly because int32_t cannot store -0.1.
 * It would become 0, so PID would not change the motor speed.
 *
 * Here float is used instead.
 *
 * Sign explanation:
 * error = targetFlow - measuredFlow
 *
 * If measuredFlow is too low, error is positive.
 * We need the motor to go faster.
 * Faster motor means smaller ARR.
 * Therefore kP is negative.
 */
static const float kP = -0.10f;

/*
 * ADDED:
 * Integral and derivative terms are included for a complete PID structure.
 * For early testing, Ki and Kd are set to 0.
 * This means the controller currently behaves mostly like P control,
 * but the structure is ready for PID tuning later.
 */
static const float kI = 0.00f;
static const float kD = 0.00f;

/*
 * MODIFIED:
 * Original code had kMaxPIDARRChange = 1, but it incorrectly limited ARR itself.
 *
 * Here this value limits how much ARR can change in one PID update.
 * This prevents sudden speed jumps.
 */
static const float kMaxPIDARRChange = 5.0f;

/*
 * ADDED:
 * Anti-windup limit for the integral term.
 */
static const float kIntegralLimit = 500.0f;

/*
 * ADDED:
 * Internal PID memory.
 */
static float gIntegralError = 0.0f;
static float gPreviousError = 0.0f;
static uint32_t gLastPIDTick = 0U;

/*
 * ADDED:
 * Track whether PWM is currently running.
 */
static uint8_t gMotorPwmRunning = 0U;

/* -------------------------------------------------------------------------- */
/* Private helper functions                                                   */
/* -------------------------------------------------------------------------- */

static float ClampFloat(float value, float minValue, float maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }

    if (value > maxValue)
    {
        return maxValue;
    }

    return value;
}

static int32_t ClampInt32(int32_t value, int32_t minValue, int32_t maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }

    if (value > maxValue)
    {
        return maxValue;
    }

    return value;
}

/* -------------------------------------------------------------------------- */
/* Public motor functions                                                     */
/* -------------------------------------------------------------------------- */

void StartMotorPWM(void)
{
    /*
     * Existing function from original code.
     *
     * ADDED:
     * gMotorPwmRunning prevents repeatedly starting PWM unnecessarily.
     */
    if (!gMotorPwmRunning)
    {
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
        gMotorPwmRunning = 1U;
    }
}

void StopMotorPWM(void)
{
    /*
     * ADDED:
     * This function fully stops PWM output.
     * The original EndPump only ramped the ARR up but did not stop PWM.
     */
    if (gMotorPwmRunning)
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        gMotorPwmRunning = 0U;
    }
}

void UpdateARR(uint32_t ARR)
{
    /*
     * Existing function from original code.
     *
     * ADDED:
     * ARR is clamped to prevent unsafe frequency values.
     */
    if (ARR < kMinARR)
    {
        ARR = kMinARR;
    }

    if (ARR > kMaxARR)
    {
        ARR = kMaxARR;
    }

    /*
     * Keep PWM duty cycle around 50%.
     * This gives a clean STEP pulse for the stepper driver.
     */
    uint32_t CCRValue = (ARR + 1U) / 2U;

    __HAL_TIM_SET_AUTORELOAD(&htim2, ARR);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, CCRValue);

    /*
     * Force timer to immediately apply new ARR/CCR values.
     */
    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
}

uint32_t GetARR(void)
{
    /*
     * Existing function from original code.
     * This returns the current timer ARR value.
     */
    return __HAL_TIM_GET_AUTORELOAD(&htim2);
}

void MotorValuesInit(void)
{
    /*
     * MODIFIED:
     * Original code used UpdateARR(52 - 1), which starts at high speed.
     *
     * For smoother and safer startup, initialise the motor at the slow ARR.
     * StartPump() will then ramp from slow speed to target speed.
     */
    UpdateARR(kStartAndEndARR);

    /*
     * ADDED:
     * Reset PID memory during motor initialisation.
     */
    ResetPID();
}

uint8_t StartPump(void)
{
    /*
     * MODIFIED:
     * Original StartPump did not return whether ramping was complete.
     * This version returns:
     * 0 = still ramping
     * 1 = finished ramping
     */

    static uint32_t lastUpdate = 0U;

    /*
     * ADDED:
     * Make sure PWM is running before ramping.
     */
    StartMotorPWM();

    uint32_t currentARR = GetARR();

    /*
     * If already at or faster than target ARR, start ramp is complete.
     */
    if (currentARR <= kFinalStartARR)
    {
        UpdateARR(kFinalStartARR);
        return 1U;
    }

    uint32_t currentTime = HAL_GetTick();

    if ((currentTime - lastUpdate) >= kTimeUpdateIntervalMs)
    {
        uint32_t errorBetweenCurrentAndFinal = currentARR - kFinalStartARR;
        uint32_t decrease = errorBetweenCurrentAndFinal / kRampFactor;

        if (decrease < 1U)
        {
            decrease = 1U;
        }

        if (currentARR > decrease)
        {
            currentARR -= decrease;
        }
        else
        {
            currentARR = kFinalStartARR;
        }

        if (currentARR < kFinalStartARR)
        {
            currentARR = kFinalStartARR;
        }

        UpdateARR(currentARR);
        lastUpdate = currentTime;
    }

    return 0U;
}

uint8_t EndPump(void)
{
    /*
     * MODIFIED:
     * Original EndPump required a uint32_t pointer.
     * That made main.c harder to use and caused type mismatch issues.
     *
     * This version reads the current ARR internally using GetARR().
     *
     * Return:
     * 0 = still ramping down
     * 1 = ramp finished and PWM stopped
     */

    static uint32_t lastUpdate = 0U;

    uint32_t currentARR = GetARR();

    /*
     * If ARR has already reached the slow end value,
     * stop PWM and finish the ramp-down.
     */
    if (currentARR >= kStartAndEndARR)
    {
        UpdateARR(kStartAndEndARR);
        StopMotorPWM();
        return 1U;
    }

    uint32_t currentTime = HAL_GetTick();

    if ((currentTime - lastUpdate) >= kTimeUpdateIntervalMs)
    {
        uint32_t progressToEnd = kStartAndEndARR - currentARR;
        uint32_t increase = progressToEnd / kRampFactor;

        if (increase < 1U)
        {
            increase = 1U;
        }

        currentARR += increase;

        if (currentARR > kStartAndEndARR)
        {
            currentARR = kStartAndEndARR;
        }

        UpdateARR(currentARR);
        lastUpdate = currentTime;
    }

    return 0U;
}

void ResetPID(void)
{
    /*
     * ADDED:
     * Clear PID memory before a new PID test starts.
     */
    gIntegralError = 0.0f;
    gPreviousError = 0.0f;
    gLastPIDTick = 0U;
}

void UpdatePID(float error)
{
    /*
     * MODIFIED:
     * Original function only attempted simple P control.
     * This version has a complete PID structure:
     * P = current error
     * I = accumulated error
     * D = rate of error change
     *
     * For now, kI and kD are set to 0, so this behaves like P control.
     * This is acceptable for placeholder PID testing.
     */

    uint32_t currentTime = HAL_GetTick();

    float dt = 0.1f;

    if (gLastPIDTick != 0U)
    {
        dt = (float)(currentTime - gLastPIDTick) / 1000.0f;

        if (dt <= 0.0f)
        {
            dt = 0.1f;
        }
    }

    gLastPIDTick = currentTime;

    /*
     * Integral term with anti-windup.
     */
    gIntegralError += error * dt;
    gIntegralError = ClampFloat(gIntegralError, -kIntegralLimit, kIntegralLimit);

    /*
     * Derivative term.
     */
    float derivativeError = (error - gPreviousError) / dt;
    gPreviousError = error;

    /*
     * PID output is the ARR change.
     *
     * Because ARR has an inverse relationship with motor speed:
     * positive error should normally reduce ARR.
     */
    float arrChangeFloat =
        (kP * error) +
        (kI * gIntegralError) +
        (kD * derivativeError);

    /*
     * Limit the ARR change per PID update.
     * This avoids sudden motor speed jumps.
     */
    arrChangeFloat = ClampFloat(
        arrChangeFloat,
        -kMaxPIDARRChange,
        kMaxPIDARRChange
    );

    int32_t arrChange = (int32_t)roundf(arrChangeFloat);

    int32_t currentARR = (int32_t)GetARR();
    int32_t newARR = currentARR + arrChange;

    /*
     * Keep ARR within safe operating range.
     */
    newARR = ClampInt32(newARR, (int32_t)kMinARR, (int32_t)kMaxARR);

    UpdateARR((uint32_t)newARR);
}

////Includes
//#include "main.h"
//#include "tim.h"
//#include "usart.h"
//#include "PumpMotor.h"
//
////Private Includes
//#include <string.h>
//#include <stdio.h>
//#include <math.h>
//
////Variables - TODO UPDATE VARIABLES TO REALISTIC VALUES
// //START/END VARIABLES
// static uint32_t kTimeUpdateInterval = 20;
// static uint32_t kFinalStartARR = 52 -1; //we'll just say 60rpm for now
// static uint32_t kRampFactor = 50; //TO BE TESTED AND CHANGED
// #define kStartAndEndARR 1000 //static uint32_t kStartAndEndARR = 0; //Find Start Frequency by the lowest frequency we get no movement
//
// //PID VARIABLES
// static int32_t kP = -0.1;
// static uint32_t kMaxPIDARRChange = 1;
//
//void StartMotorPWM(void){
//    //Start PWM which starts stepping
//    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
//}
//
//
//void UpdateARR(uint32_t ARR){
//	uint32_t CCRValue = (ARR + 1) /2;
//	__HAL_TIM_SET_AUTORELOAD(&htim2, ARR);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, CCRValue);
//	HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
//}
//
//uint32_t GetARR(void){
//    return __HAL_TIM_GET_AUTORELOAD(&htim2);
//}
//
//void MotorValuesInit(void){
//    UpdateARR(52 -1);
//}
//
////could potentially have startpump and endpump functions return a string
////so when it finishes that string can be printed to console
//
////i think the finalfrequency this function reaches should be quite beneath
////the expected frequency for flow rate otherwise it may be accidentally activated
//void StartPump(void) {
//    //tim2 is currently set to tick every microsecond
//    //we want to ramp up frequency a certain amount per a certain amount of time
//    //until it reaches a certain frequency and then we end function
//    static uint32_t ARR = kStartAndEndARR;
//    static uint32_t lastUpdate = 0;
//
//    if (ARR > kFinalStartARR) {
//
//        uint32_t currentTime = HAL_GetTick();
//
//        if ((currentTime - lastUpdate) >= kTimeUpdateInterval) {
//
//            uint32_t errorBetweenCurrentAndFinal = ARR - kFinalStartARR;
//            uint32_t decrease = errorBetweenCurrentAndFinal / kRampFactor;
//
//            if (decrease < 1)
//            {
//                decrease = 1;
//            }
//
//                ARR -= decrease;
//
//            if (ARR < kFinalStartARR)
//            {
//                ARR = kFinalStartARR;
//            }
//
//            UpdateARR(ARR);
//
//            lastUpdate = currentTime;
//        }
//    }
//}
//
////this function may be a bit weird and needs some more thinking
////we will only want to call it say when a button is pressed
////at that time we will hand it the frequency and then this should work
////but i need to double check this
//void EndPump(uint32_t *ARR) {
//
//    static uint32_t lastUpdate = 0;
//    static uint32_t startARR = 0;
//    static uint8_t rampStarted = 0;
//
//    if (!rampStarted) {
//        startARR = *ARR;
//        rampStarted = 1;
//    }
//
//    if (*ARR < kStartAndEndARR) {
//
//        uint32_t currentTime = HAL_GetTick();
//
//        if ((currentTime - lastUpdate) >= kTimeUpdateInterval) {
//
//            uint32_t progressFromStartARR = *ARR - startARR;
//            uint32_t increase = progressFromStartARR / kRampFactor;
//
//            if (increase < 1)
//            {
//                increase = 1;
//            }
//
//            *ARR += increase;
//
//            if (*ARR > kStartAndEndARR)
//            {
//                *ARR = kStartAndEndARR;
//            }
//
//
//            UpdateARR(*ARR);
//            lastUpdate = currentTime;
//        }
//    }
//    else {
//        rampStarted = 0;
//    }
//}
//
//
//
//void UpdatePID(int32_t error) {
//
//    //typically if error increase, we want to subtract it.
//    //make sure the kp value doesnt cause overcompensation (if too high)
//
//    int32_t errorChange = (error * kP);
//
//    int32_t ARR = (int32_t)GetARR();
//    ARR += errorChange;
//
//    if (ARR < 0) { ARR = 0;};
//    if (ARR > kMaxPIDARRChange) { ARR = kMaxPIDARRChange;};
//
//    UpdateARR((uint32_t)ARR);
//
//}
