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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "PumpMotor.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * ADDED:
 * Motor state machine.
 *
 * Original code declared Start / Maintenance / End states,
 * but the state variable was not actually used.
 *
 * This version uses the state machine to run:
 * START_RAMP -> PID_TEST -> END_RAMP -> STOPPED
 */
typedef enum
{
    MOTOR_STATE_START_RAMP = 0,
    MOTOR_STATE_PID_TEST,
    MOTOR_STATE_END_RAMP,
    MOTOR_STATE_STOPPED
} MotorState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * ADDED:
 * Placeholder control settings for this week's deliverable.
 * These values are used because the design team has not started moving fluid yet.
 */
#define TARGET_FLOW_ML_MIN          250
#define PID_TEST_DURATION_MS        30000U
#define CONTROL_UPDATE_INTERVAL_MS  100U
#define UART_PRINT_INTERVAL_MS      500U

/*
 * ADDED:
 * These values are only used for placeholder flow simulation.
 * They should match the slow and fast ARR values in PumpMotor.c.
 */
#define PLACEHOLDER_SLOW_ARR        999U
#define PLACEHOLDER_FAST_ARR        51U
#define PLACEHOLDER_MAX_FLOW        300

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
 * ADDED:
 * Variables used by the state machine and PID test.
 */
static MotorState motorState = MOTOR_STATE_START_RAMP;

static uint32_t pidStartTick = 0U;
static uint32_t lastControlUpdateTick = 0U;
static uint32_t lastUartPrintTick = 0U;

static int32_t targetFlow = TARGET_FLOW_ML_MIN;
static int32_t measuredFlow = 0;
static int32_t flowError = 0;

static uint8_t stoppedMessagePrinted = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/*
 * ADDED:
 * Helper functions for placeholder PID testing and UART output.
 */
static int32_t GetPlaceholderMeasuredFlow(uint32_t arr, uint32_t elapsedMs);
static const char* GetMotorStateName(MotorState state);
static void PrintMotorStatus(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

    if (simulatedFlow < 0)
    {
        simulatedFlow = 0;
    }

    return simulatedFlow;
}

static const char* GetMotorStateName(MotorState state)
{
    /*
     * ADDED:
     * Convert state enum to readable text for UART debug messages.
     */
    switch (state)
    {
        case MOTOR_STATE_START_RAMP:
            return "START_RAMP";

        case MOTOR_STATE_PID_TEST:
            return "PID_TEST";

        case MOTOR_STATE_END_RAMP:
            return "END_RAMP";

        case MOTOR_STATE_STOPPED:
            return "STOPPED";

        default:
            return "UNKNOWN";
    }
}

static void PrintMotorStatus(void)
{
    /*
     * ADDED:
     * UART debug print.
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
        GetMotorStateName(motorState),
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /*
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

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

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

  PrintMotorStatus();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint32_t currentTick = HAL_GetTick();

    switch (motorState)
    {
        case MOTOR_STATE_START_RAMP:
        {
            /*
             * ADDED:
             * Start ramp stage.
             *
             * StartPump() returns 1 when ramp-up is complete.
             */
            if (StartPump())
            {
                ResetPID();

                pidStartTick = currentTick;
                lastControlUpdateTick = currentTick;
                lastUartPrintTick = currentTick;

                motorState = MOTOR_STATE_PID_TEST;

                PrintMotorStatus();
            }

            break;
        }

        case MOTOR_STATE_PID_TEST:
        {
            /*
             * ADDED:
             * PID testing stage.
             *
             * Since there is no real flow sensor yet, measuredFlow comes from
             * GetPlaceholderMeasuredFlow().
             *
             * Later, replace this placeholder value with real flow sensor data.
             */
            uint32_t elapsedMs = currentTick - pidStartTick;

            if ((currentTick - lastControlUpdateTick) >= CONTROL_UPDATE_INTERVAL_MS)
            {
                measuredFlow = GetPlaceholderMeasuredFlow(GetARR(), elapsedMs);
                flowError = targetFlow - measuredFlow;

                /*
                 * ADDED:
                 * Motor speed is now controlled based on PID error.
                 */
                UpdatePID((float)flowError);

                lastControlUpdateTick = currentTick;
            }

            if ((currentTick - lastUartPrintTick) >= UART_PRINT_INTERVAL_MS)
            {
                PrintMotorStatus();
                lastUartPrintTick = currentTick;
            }

            /*
             * ADDED:
             * Automatically finish the placeholder PID test after 30 seconds.
             */
            if (elapsedMs >= PID_TEST_DURATION_MS)
            {
                motorState = MOTOR_STATE_END_RAMP;
                PrintMotorStatus();
            }

            break;
        }

        case MOTOR_STATE_END_RAMP:
        {
            /*
             * ADDED:
             * End ramp stage.
             *
             * EndPump() returns 1 when ramp-down is complete and PWM is stopped.
             */
            if (EndPump())
            {
                motorState = MOTOR_STATE_STOPPED;
                PrintMotorStatus();
            }

            break;
        }

        case MOTOR_STATE_STOPPED:
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

        default:
        {
            /*
             * ADDED:
             * Safety fallback.
             */
            StopMotorPWM();
            motorState = MOTOR_STATE_STOPPED;
            break;
        }
    }

    HAL_Delay(1);

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

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  __disable_irq();

  while (1)
  {
  }

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

  /*
   * User can add printf debugging here if needed.
   */

  /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */

///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2026 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "tim.h"
//#include "usart.h"
//#include "gpio.h"
//#include "PumpMotor.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include <string.h>
//#include <stdio.h>
//#include <math.h>
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//
///* USER CODE BEGIN PV */
//
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
///* USER CODE BEGIN PFP */
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//
//  /* USER CODE BEGIN 1 */
//
//
//  /*frequency = (timclocksource / PSC) / ARR (period)
//	smaller ARR value = higher the frequency
//
//	ARR = frequency
//	PSC = speed		(currently 80-1, matching 80mhz timclocksource (edit in clock config))
//	CCR = length of pulse (on) ((ARR+1)/2 for 50% Duty Cycle)
//
//	At 1/32 steps, we have 6400 steps per revolution. Which is 6.4khz per revolution.
//	*/
//
//  //ARR value required for different frequencies
//  //Slower Values to the top
//  uint32_t hz = 1000000-1; //current ARR set at 1000-1 = 1khz.
//  uint32_t khz = 1000-1; //6.4 rotations per seond.
//  uint32_t a = 500-1;
//  uint32_t sixtyrpm = 156-1;
//  uint32_t oneeightyrpm = 52 -1;
//  uint32_t tenkhz = 100-1;
//  //Faster Values to the bottom
//
//  //State Machine
//  enum MotorState {
//    Start,
//    Maintenance,
//    End
//  };
//
//  enum MotorState motorState = Start;
//
//  //For Messages
//  char buffer[50];
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_USART2_UART_Init();
//  MX_TIM1_Init();
//  MX_TIM2_Init();
//  /* USER CODE BEGIN 2 */
//  MotorValuesInit();
//  StartMotorPWM();
//
//
//  //Example console message.
//  sprintf(buffer, "StartwithPWM\r\n");
//  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
//
//
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//
//	//Frequency Change
//	//StartPump();
//	EndPump(GetARR());
//	HAL_Delay(10);
//	//For changing direction of motor
//	//HAL_GPIO_WritePin(GPIOA, Direction_Pin, GPIO_PIN_SET); //test which direction is clockwise vs anti
//
//
//  //Change ARR (choose a method)
//  //TIM1->ARR = slow;
//  //__HAL_TIM_SET_AUTORELOAD(&htim1, slow)
//
//
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure LSE Drive Capability
//  */
//  HAL_PWR_EnableBkUpAccess();
//  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
//  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//  RCC_OscInitStruct.MSICalibrationValue = 0;
//  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
//  RCC_OscInitStruct.PLL.PLLM = 1;
//  RCC_OscInitStruct.PLL.PLLN = 40;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
//  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
//  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Enable MSI Auto calibration
//  */
//  HAL_RCCEx_EnableMSIPLLMode();
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//#ifdef USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
