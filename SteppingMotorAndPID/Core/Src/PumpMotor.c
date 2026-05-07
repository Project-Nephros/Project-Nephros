
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
 static uint32_t kTimeUpdateInterval = 0;
 static uint32_t kUpdateChange = 1;
 static uint32_t kFinalStartFrequency = 0;
 static uint32_t kEndFrequency = 0;
 
 static uint32_t kP = 0.1;
 static int32_t kMaxPIDFrequencyChange = 1; 

 static uint32_t kStartFrequency = 0;

void StartMotorPWM(void){
    //Start PWM which starts stepping
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}


void UpdateFrequency(uint32_t frequency){
	uint32_t CCRValue = (frequency + 1) /2;
	__HAL_TIM_SET_AUTORELOAD(&htim2, frequency);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, CCRValue);
	HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);
}

uint32_t GetFrequency(void){
    return __HAL_TIM_GET_AUTORELOAD(&htim2);
}

void MotorValuesInit(void){
    UpdateFrequency(kStartFrequency);
}

//could potentially have startpump and endpump functions return a string 
//so when it finishes that string can be printed to console

//i think the finalfrequency this function reaches should be quite beneath
//the expected frequency for flow rate otherwise it may be accidentally activated
void StartPump(void) {
    //tim2 is currently set to tick every microsecond
    //we want to ramp up frequency a certain amount per a certain amount of time
    //until it reaches a certain frequency and then we end function
    static uint32_t frequency = 0;
    static uint32_t lastUpdate = 0;

    if (frequency < kFinalStartFrequency) {
        
        uint32_t currentTime = HAL_GetTick();

        if ((currentTime - lastUpdate) >= kTimeUpdateInterval) {
            frequency += kUpdateIncrease;
            UpdateFrequency(frequency);
        }
    }
}

//this function may be a bit weird and needs some more thinking
//we will only want to call it say when a button is pressed
//at that time we will hand it the frequency and then this should work
//but i need to double check this
void EndPump(uint32_t *frequency) {

    static uint32_t lastUpdate = 0;

    if (*frequency > kEndFrequency) {
        
        uint32_t currentTime = HAL_GetTick();

        if ((currentTime - lastUpdate) >= kTimeUpdateInterval) {
            *frequency -= kUpdateIncrease;
            UpdateFrequency(*frequency);
        }
    }
}



void UpdatePID(int32_t error) {

    //typically if error increase, we want to subtract it.
    //make sure the kp value doesnt cause overcompensation (if too high)
    
    int32_t errorChange = (error * kP);

    int32_t frequency = (int32_t)GetFrequency();
    frequency += errorChange;

    if (frequency < 0) { frequency = 0};
    if (frequency > kMaxPIDFrequencyChange) { frequency = kMaxPIDFrequencyChange};

    UpdateFrequency((uint32_t)frequency);

}