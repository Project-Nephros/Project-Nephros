// Includes
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "AlarmSystem.h"

// Private Includes
#include <string.h>
#include <stdio.h>
#include <math.h>

char msg[100];

void Temp(float temp){
    if (temp < 35.0 || temp > 38.0) {
        sprintf(msg, "WARNING: Temp out of range (%.1f C)\r\n", temp);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

        if (temp < 32.0 || temp > 40.0) {
            // SEVERE: Stop pumps and alarm
            HAL_GPIO_WritePin(GPIOB, RED_LED_Pin | BUZZER_Pin, GPIO_PIN_SET);
            HAL_UART_Transmit(&huart2, (uint8_t*)"CRITICAL: SEVERE TEMP - HALT PUMPS!\r\n", 37, 100);
        }
    }
}
int Pressure(float pressure, int pressureTimer){
    if (pressure < 20.0 || pressure > 80.0) {
        pressureTimer++;
        if (pressureTimer >= 3) {
            HAL_GPIO_WritePin(GPIOB, RED_LED_Pin | BUZZER_Pin, GPIO_PIN_SET);
            HAL_UART_Transmit(&huart2, (uint8_t*)"ALARM: PRESSURE ERROR > 3 SECONDS!\r\n", 36, 100);
        }
    } else {
        pressureTimer = 0;
    }

    return pressureTimer;
}

void Air(int air){
    if (air == 1) {
        HAL_GPIO_WritePin(GPIOB, RED_LED_Pin | BUZZER_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit(&huart2, (uint8_t*)"ALARM: AIR DETECTED! SYSTEM LOCKED.\r\n", 37, 100);
    }
}

void Normal(float temp, int pressureTimer, int air){
    if (air == 0 && pressureTimer < 3 && (temp >= 32.0 && temp <= 40.0)) {
    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
        if (temp >= 35.0 && temp <= 38.0) {
        HAL_GPIO_WritePin(GPIOB, RED_LED_Pin | BUZZER_Pin, GPIO_PIN_RESET);
        }
    } else {
        HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
    }

}