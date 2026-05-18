
//Includes
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "PumpMotor.h"

//Private Includes
#include <string.h>
#include <stdio.h>
#include <math.h>

//Variables - TODO UPDATE VARIABLES TO REALISTIC VALUES 
 //START/END VARIABLES
 static uint32_t kTimeUpdateInterval = 20;
 static uint32_t kFinalStartARR = 52 -1; //we'll just say 60rpm for now
 static uint32_t kRampFactor = 50; //TO BE TESTED AND CHANGED
 #define kStartAndEndARR 1000 //static uint32_t kStartAndEndARR = 0; //Find Start Frequency by the lowest frequency we get no movement

 //PID VARIABLES
 static int32_t kP = -0.1;
 static uint32_t kMaxPIDARRChange = 1;

void StartMotorPWM(void){
    //Start PWM which starts stepping
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}


void UpdateARR(uint32_t ARR){
	uint32_t CCRValue = (ARR + 1) /2;
	__HAL_TIM_SET_AUTORELOAD(&htim2, ARR);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, CCRValue);
	HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
}

uint32_t GetARR(void){
    return __HAL_TIM_GET_AUTORELOAD(&htim2);
}

void MotorValuesInit(void){
    UpdateARR(52 -1);
}

//could potentially have startpump and endpump functions return a string 
//so when it finishes that string can be printed to console

//i think the finalfrequency this function reaches should be quite beneath
//the expected frequency for flow rate otherwise it may be accidentally activated
void StartPump(void) {
    //tim2 is currently set to tick every microsecond
    //we want to ramp up frequency a certain amount per a certain amount of time
    //until it reaches a certain frequency and then we end function
    static uint32_t ARR = kStartAndEndARR;
    static uint32_t lastUpdate = 0;

    if (ARR > kFinalStartARR) {
        
        uint32_t currentTime = HAL_GetTick();

        if ((currentTime - lastUpdate) >= kTimeUpdateInterval) {
            
            uint32_t errorBetweenCurrentAndFinal = ARR - kFinalStartARR;
            uint32_t decrease = errorBetweenCurrentAndFinal / kRampFactor;

            if (decrease < 1)
            {
                decrease = 1;
            }

                ARR -= decrease;

            if (ARR < kFinalStartARR)
            {
                ARR = kFinalStartARR;
            }

            UpdateARR(ARR);

            lastUpdate = currentTime;
        }
    }
}

//this function may be a bit weird and needs some more thinking
//we will only want to call it say when a button is pressed
//at that time we will hand it the frequency and then this should work
//but i need to double check this
void EndPump(uint32_t *ARR) {

    static uint32_t lastUpdate = 0;
    static uint32_t startARR = 0;
    static uint8_t rampStarted = 0;

    if (!rampStarted) {
        startARR = *ARR;
        rampStarted = 1;
    }

    if (*ARR < kStartAndEndARR) {
        
        uint32_t currentTime = HAL_GetTick();

        if ((currentTime - lastUpdate) >= kTimeUpdateInterval) {

            uint32_t progressFromStartARR = *ARR - startARR;
            uint32_t increase = progressFromStartARR / kRampFactor;

            if (increase < 1)
            {
                increase = 1;
            }

            *ARR += increase;

            if (*ARR > kStartAndEndARR)
            {
                *ARR = kStartAndEndARR;
            }

            
            UpdateARR(*ARR);
            lastUpdate = currentTime;
        }
    }
    else {
        rampStarted = 0;
    }
}



void UpdatePID(int32_t error) {

    //typically if error increase, we want to subtract it.
    //make sure the kp value doesnt cause overcompensation (if too high)
    
    int32_t errorChange = (error * kP);

    int32_t ARR = (int32_t)GetARR();
    ARR += errorChange;

    if (ARR < 0) { ARR = 0;};
    if (ARR > kMaxPIDARRChange) { ARR = kMaxPIDARRChange;};

    UpdateARR((uint32_t)ARR);

}
