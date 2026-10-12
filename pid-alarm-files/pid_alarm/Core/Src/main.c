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
#include "dma.h"
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
#define RX_BUFFER_SIZE      512U    /* DMA buffer for one UART1 line (msg is ~90 chars) */
#define RX_TIMEOUT_MS       3000U   /* warn if no UART1 data for this long */
#define DEBUG_RX            1       /* 1 = print every received line + parse result */
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
    .air_detected = false,
    .flow_rate = 0.0f
};

static NephrosSetup setup_data;

/* TIMER VARIABLES */
uint32_t lastAlarmLCDCheck = 0;
static uint32_t end_message_hold_ms = 0U;
static bool lcd_message_active = false;
static bool pending_button_press = false;

/* RECEIVE UART1 VARIABLES */
extern DMA_HandleTypeDef hdma_usart1_rx;           /* defined in usart.c */
static uint8_t  rx_Buffer[RX_BUFFER_SIZE];         /* DMA writes here */
static uint8_t  main_Buffer[RX_BUFFER_SIZE + 1];   /* ISR copies finished line here */
static volatile uint16_t rx_len = 0;               /* length of line in main_Buffer */
static volatile uint8_t  data_ready_flag = 0;      /* set by ISR, cleared by main */
static uint32_t last_rx_ms = 0;                    /* time of last good line */
static uint32_t last_rx_warn_ms = 0;               /* rate-limit "no data" warning */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void start_uart1_rx(void);
static void read_sensors(NephrosSensorData *sensor, uint32_t now_ms);
static bool find_value(const char *text, const char *key, float *out);
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
  char startMessage[] =
      "Pump motor started. Listening for sensor data on UART1.\r\n";
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* LCD + UART2 startup menu (password, volume, duration) */
  NephrosUI_Init();
  NephrosUI_ShowBoot();
  NephrosUI_ShowConsolePrompt();
  NephrosUI_RunStartupMenu(&setup_data);
  NephrosUI_ShowSetupComplete(&setup_data);

  /* Safety monitor logs through UART2 */
  NephrosSafety_Init(NephrosUI_Write);

  /* Motor starts at slow ARR, StartPump() ramps it up */
  MotorValuesInit();
  StartMotorPWM();

  HAL_UART_Transmit(&huart2, (uint8_t*)startMessage, strlen(startMessage), 100);

  /*
   * Start UART1 reception AFTER the menu.
   * start_uart1_rx() clears any overrun built up while we sat in the menu,
   * otherwise the first DMA start would abort immediately.
   */
  last_rx_ms = HAL_GetTick();
  start_uart1_rx();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t currentTick = HAL_GetTick();

      /* Check button every loop; used once in the 20 ms block */
      if (NephrosUI_ButtonPressed(currentTick))
      {
          pending_button_press = true;
      }

      /* Run safety/UI/motor logic every 20 ms */
      if (currentTick - lastAlarmLCDCheck >= 20U)
      {
          bool button_pressed;
          NephrosSafetyOutput safety_output;

          lastAlarmLCDCheck = currentTick;

          button_pressed = pending_button_press;
          pending_button_press = false;

          /* Update sensor_data from the latest UART1 line (if one arrived) */
          read_sensors(&sensor_data, currentTick);

          safety_output = NephrosSafety_Update(&sensor_data, button_pressed, currentTick);

          /* Emergency stop */
          if (safety_output.halted)
          {
              SetState(STATE_EMERGENCY_STOP);
          }

          /* Resume after a successful re-check */
          if (safety_output.resume_requested)
          {
              MotorValuesInit();
              StartMotorPWM();
              SetState(STATE_START_MOTOR_RAMP);
          }

          /* Motor state machine runs after the safety decision */
          RunStateMachine(currentTick);

          /* LCD message handling */
          if (safety_output.lcd_message_valid)
          {
              NephrosUI_ShowMessage(safety_output.lcd_line1, safety_output.lcd_line2);
              lcd_message_active = true;
              end_message_hold_ms = currentTick + safety_output.message_hold_ms;
          }
          else if (lcd_message_active && currentTick >= end_message_hold_ms)
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
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

/*
 * Start (or restart) UART1 DMA reception cleanly.
 * Clears error flags + flushes the receive register first, so a stale
 * overrun from earlier can't instantly abort the new reception.
 */
static void start_uart1_rx(void)
{
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF |
                                   UART_CLEAR_NEF  | UART_CLEAR_PEF |
                                   UART_CLEAR_IDLEF);
    __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_Buffer, RX_BUFFER_SIZE);

    /* Only want "line finished" events, not half-buffer events */
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

/*
 * Find "key" in text and read the number after it.
 * e.g. key "T1=" in "...T1=36.75, ..." -> 36.75
 * Returns false if the key is missing or no number follows it.
 */
static bool find_value(const char *text, const char *key, float *out)
{
    const char *p = strstr(text, key);
    char *end;
    float v;

    if (p == NULL)
    {
        return false;
    }

    p += strlen(key);
    v = strtof(p, &end);

    if (end == p || !isfinite(v))
    {
        return false;
    }

    *out = v;
    return true;
}

/*
 * Copy the latest UART1 line out of the ISR buffer and parse it.
 * Lenient: each field is searched for separately, so missing/extra
 * fields or odd spacing don't break anything. Only fields that are
 * found get updated; the rest keep their previous value.
 */
static void read_sensors(NephrosSensorData *sensor, uint32_t now_ms)
{
    char line[RX_BUFFER_SIZE + 1];
    const char *msg;
    uint16_t len;
    float v;
    int fields = 0;

    /* No new line: warn (rate limited) if data has gone quiet */
    if (!data_ready_flag)
    {
        if ((now_ms - last_rx_ms) >= RX_TIMEOUT_MS &&
            (now_ms - last_rx_warn_ms) >= RX_TIMEOUT_MS)
        {
            last_rx_warn_ms = now_ms;
            NephrosUI_Write("WARN: no UART1 data\r\n");
        }
        return;
    }

    /* Copy safely: stop the ISR overwriting main_Buffer mid-copy */
    __disable_irq();
    len = rx_len;
    if (len > RX_BUFFER_SIZE) len = RX_BUFFER_SIZE;
    memcpy(line, main_Buffer, len);
    data_ready_flag = 0;
    __enable_irq();
    line[len] = '\0';

    /* If several lines got merged, use the newest one (last '[') */
    msg = strrchr(line, '[');
    if (msg == NULL)
    {
        msg = line;
    }

    if (find_value(msg, "T1=", &v))
    {
        sensor->temperature_c = v;
        fields++;
    }

    if (find_value(msg, "Bubble=", &v))
    {
        sensor->air_detected = (v != 0.0f);
        fields++;
    }

    if (find_value(msg, "P1=", &v))
    {
        if (v < 0.0f)     v = 0.0f;
        if (v > 65535.0f) v = 65535.0f;
        sensor->pressure = (uint16_t)v;
        fields++;
    }

    if (find_value(msg, "F1=", &v))
    {
        sensor->flow_rate = v;
        fields++;
    }

    if (fields > 0)
    {
        last_rx_ms = now_ms;
    }

#if DEBUG_RX
    {
        char dbg[48];
        NephrosUI_Write("RX: ");
        NephrosUI_Write(msg);
        if (len == 0 || line[len - 1] != '\n')
        {
            NephrosUI_Write("\r\n");
        }
        snprintf(dbg, sizeof(dbg), "  -> %d/4 fields updated\r\n", fields);
        NephrosUI_Write(dbg);
    }
#endif
}

/* UART1 line finished (line went idle): copy it out, flag main, restart DMA */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        if (Size > RX_BUFFER_SIZE) Size = RX_BUFFER_SIZE;
        memcpy(main_Buffer, rx_Buffer, Size);
        main_Buffer[Size] = '\0';
        rx_len = Size;
        data_ready_flag = 1;

        start_uart1_rx();
    }
}

/* UART1 error (overrun/noise/framing): abort and restart so RX never dies */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UART_AbortReceive(huart);
        start_uart1_rx();
    }
}

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
