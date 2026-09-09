/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PumpMotor.h"
#include "StateMachine.h"
#include "nephros_ui.h"
#include "nephros_safety.h"
#include "liquidcrystal_i2c.h"


#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include <errno.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static NephrosSensorData sensor_data =
{
    .temperature_c = 36.5f,
    .pressure = 150U,
    .air_detected = false
};

static NephrosSetup setup_data;

//TIMER VARIABLES
uint32_t lastAlarmLCDCheck = 0;
static uint32_t end_message_hold_ms = 0U;
static bool lcd_message_active = false;
static bool pending_button_press = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void read_sensors(NephrosSensorData *sensor);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /*THIS NEEDS TO BE FIXED
   * MODIFIED:
   * Original manual ARR test variables were removed from main.c.
   * Motor speed settings are now handled inside PumpMotor.c.
   *
   * ADDED:
   * main.c now focuses on state-machine control:
   * start ramp -> PID test -> end ramp -> stopped.
   */

  char startMessage[] =
      "Pump motor PID test started. Using placeholder flow values.\r\n";

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */

  NephrosUI_Init();

  NephrosUI_ShowBoot();

  NephrosUI_ShowConsolePrompt();

  NephrosUI_RunStartupMenu(&setup_data);

  NephrosUI_ShowSetupComplete(&setup_data);

  NephrosSafety_Init(NephrosUI_Write);
  
    /*
   * Existing motor initialisation.
   *
   * MODIFIED:
   * MotorValuesInit() now starts from slow ARR instead of high speed.
   */
  MotorValuesInit();

  /*
   * Start PWM output.
   * The motor begins at slow ARR, then StartPump() ramps it up.
   */
  StartMotorPWM();

  /*
   * UART message to confirm the program has started.
   */
  HAL_UART_Transmit(
      &huart2,
      (uint8_t*)startMessage,
      strlen(startMessage),
      100
  );

  //PrintMotorStatus(); //FIXME Need to figure out what we'll do in the actual code or whatever. as this wont work 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    	uint32_t currentTick = HAL_GetTick();

    	    /*
    	     * Always check the button quickly.
    	     * If a press happens, remember it until the 20 ms control block uses it.
    	     */
    	    if (NephrosUI_ButtonPressed(currentTick))
    	    {
    	        pending_button_press = true;
    	    }

    	    /*
    	     * Run safety/UI logic every 20 ms.
    	     */
    	    if (currentTick - lastAlarmLCDCheck >= 20U)
    	    {
    	        bool button_pressed;
    	        NephrosSafetyOutput safety_output;

    	        lastAlarmLCDCheck = currentTick;

    	        /*
    	         * Use the stored button press once.
    	         */
    	        button_pressed = pending_button_press;
    	        pending_button_press = false;

    	        read_sensors(&sensor_data);

    	        safety_output = NephrosSafety_Update(
    	            &sensor_data,
    	            button_pressed,
    	            currentTick
    	        );

    	        /*
    	         * Emergency stop handling.
    	         */
    	        if (safety_output.halted)
    	        {
    	            SetState(STATE_EMERGENCY_STOP);
    	        }

    	        /*
    	         * Resume handling.
    	         */
    	        if (safety_output.resume_requested)
    	        {
    	            MotorValuesInit();
    	            StartMotorPWM();
    	            SetState(STATE_START_MOTOR_RAMP);
    	        }

    	        /*
    	         * Run motor state machine after safety decision.
    	         */
    	        RunStateMachine(currentTick);

    	        /*
    	         * LCD message handling.
    	         */
    	        if (safety_output.lcd_message_valid)
    	        {
    	            NephrosUI_ShowMessage(
    	                safety_output.lcd_line1,
    	                safety_output.lcd_line2
    	            );

    	            lcd_message_active = true;
    	            end_message_hold_ms =
    	                currentTick + safety_output.message_hold_ms;
    	        }
    	        else if (lcd_message_active &&
    	                 currentTick >= end_message_hold_ms)
    	        {
    	            lcd_message_active = false;
    	            NephrosUI_ForceRedraw();
    	        }
    	        else if (!lcd_message_active)
    	        {
    	            if (safety_output.force_normal_lcd_redraw)
    	            {
    	                NephrosUI_ForceRedraw();
    	            }

    	            if (safety_output.can_cycle_lcd_view)
    	            {
    	                if (button_pressed)
    	                {
    	                    NephrosUI_NextView();
    	                }

    	                NephrosUI_ShowNormalIfChanged(&sensor_data);

            }
        }

      }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
static void read_sensors(NephrosSensorData *sensor)
{
    /*
     * For now, this function does nothing because we are using
     * the debugger to edit sensor_data manually.
     *
     * Later, Group A sensor code should update:
     * sensor->temperature_c
     * sensor->pressure
     * sensor->air_detected
     */

    (void)sensor;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
