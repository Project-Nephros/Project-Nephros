/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Project Nephros — Flow + Pressure Monitor (merged)
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
  *
  * Architecture:
  *  - Flow sensor on PA3 (TIM2_CH4 input capture, interrupt-driven pulse count)
  *    Pulses are integrated over 1-second windows. Result feeds a 10-sample MA.
  *  - Pressure sensors on PA0 (ADC1_IN5) and PA1 (ADC1_IN6), interrupt-driven
  *    sequential conversions, sampled every 500 ms. Each sensor has its own
  *    10-sample MA (covering 5 s of history at 500 ms cadence).
  *  - Combined line printed every 500 ms over USART2.
  *  - Non-blocking main loop using HAL_GetTick() — no HAL_Delay anywhere.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Pressure sensor scaling */
#define ADC_MAX_COUNT        4095.0f
#define VREF_VOLTS           3.3f
#define DIVIDER_RATIO        0.5f
#define SENSOR_V_AT_ZERO     0.5f
#define SENSOR_VOLTAGE_SPAN  4.0f
#define SENSOR_PSI_MAX       30.0f

/* Timing */
#define FLOW_PERIOD_MS       500U   /* pulse integration window */
#define PRESSURE_PERIOD_MS   500U    /* ADC sample + print cadence */
#define UART_TIMEOUT_MS      100U

/* Flow scaling: pulses/sec to L/min uses sensor K-factor (5880 pulses per L) */
#define FLOW_PULSES_PER_LITRE  5880.0

/* Moving average buffers */
#define MA_DEPTH             10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* --- Flow (written by ISR, read by main) --- */
volatile uint32_t pulse_count_1 = 0;   /* TIM2_CH1 / PA5 */
volatile uint32_t pulse_count_4 = 0;   /* TIM2_CH4 / PA3 */

/* Flow MA */
typedef struct {
    double  buffer[MA_DEPTH];
    uint8_t index;
    uint8_t filled;
    double  latest_mLmin;
    double  avg_mLmin;
} flow_ma_t;
static flow_ma_t flow1 = {0};   /* PA5 / CH1 */
static flow_ma_t flow2 = {0};   /* PA3 / CH4 */

/* --- Pressure (ISR writes raws, main consumes) --- */
static volatile uint8_t  current_channel = 0;
static volatile uint32_t raw_sensor1 = 0;
static volatile uint32_t raw_sensor2 = 0;
static volatile uint8_t  data_ready = 0;

/* Pressure MAs */
static float   p1_buffer[MA_DEPTH] = {0};
static float   p2_buffer[MA_DEPTH] = {0};
static uint8_t p_index = 0;
static uint8_t p_filled = 0;

/* --- Timing --- */
static uint32_t last_flow_tick = 0;
static uint32_t last_pressure_tick = 0;

/* Startup message latch */
static uint8_t startup_msg_sent = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void flow_update(flow_ma_t *f, uint32_t window_pulses);
static void  pressure_start_sequence(void);
static float adc_counts_to_volts(uint32_t raw_count);
static float sensor_volts_to_psi(float sensor_volts);
static void  uart_print(const char *message);
static void  print_combined_line(void);
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* ADC self-calibration before first conversion */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
      Error_Handler();
  }

  /* Start flow capture */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

  uart_print("\r\n===================================\r\n");
  uart_print("  Project Nephros — Flow + Pressure\r\n");
  uart_print("  Flow: PA3 / PA5 (TIM2_CH4 / TIM2_CH1)\r\n");
  uart_print("  Pressure: PA0 / PA1 (ADC1)\r\n");
  uart_print("  Print cadence: 500 ms\r\n");
  uart_print("===================================\r\n\r\n");

  last_flow_tick     = HAL_GetTick();
  last_pressure_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t now = HAL_GetTick();

      /* --- Flow window: integrate pulses over 1 second --- */
      if (now - last_flow_tick >= FLOW_PERIOD_MS)
      {
          last_flow_tick += FLOW_PERIOD_MS;

          __disable_irq();
          uint32_t p1 = pulse_count_1; pulse_count_1 = 0;
          uint32_t p4 = pulse_count_4; pulse_count_4 = 0;
          __enable_irq();

          flow_update(&flow1, p1);
          flow_update(&flow2, p4);
      }



      /* --- Pressure sample + combined print every 500 ms --- */
      if (now - last_pressure_tick >= PRESSURE_PERIOD_MS)
      {
          last_pressure_tick += PRESSURE_PERIOD_MS;

          /* Kick off interrupt-driven dual-channel ADC */
          pressure_start_sequence();

          /* Wait for both channels (typically <1 ms total at 247.5 cycles
             sampling + 12.5 cycles conv * 2 channels). Bounded by a tick
             to avoid lockup if ADC misbehaves. */
          uint32_t wait_start = HAL_GetTick();
          while (!data_ready)
          {
              if (HAL_GetTick() - wait_start > 10) break;  /* safety bail */
          }

          if (data_ready)
          {
              data_ready = 0;
              uint32_t s1 = raw_sensor1;
              uint32_t s2 = raw_sensor2;

              float psi1 = sensor_volts_to_psi(adc_counts_to_volts(s1) / DIVIDER_RATIO);
              float psi2 = sensor_volts_to_psi(adc_counts_to_volts(s2) / DIVIDER_RATIO);

              /* Push into MA buffers */
              p1_buffer[p_index] = psi1;
              p2_buffer[p_index] = psi2;
              p_index = (p_index + 1) % MA_DEPTH;
              if (p_filled < MA_DEPTH) p_filled++;
          }

          print_combined_line();
      }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ---------------- Flow ---------------- */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        pulse_count_1++;
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
        pulse_count_4++;
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);  /* keep heartbeat on one channel */
    }
}

static void flow_update(flow_ma_t *f, uint32_t window_pulses)
{
    double flow_Lmin  = (double)window_pulses * 60.0 / FLOW_PULSES_PER_LITRE;
    double flow_mLmin = flow_Lmin * 1000.0;

    f->latest_mLmin = flow_mLmin;
    f->buffer[f->index] = flow_mLmin;
    f->index = (f->index + 1) % MA_DEPTH;
    if (f->filled < MA_DEPTH) f->filled++;

    if (f->filled >= MA_DEPTH)
    {
        double sum = 0;
        for (int i = 0; i < MA_DEPTH; i++) sum += f->buffer[i];
        f->avg_mLmin = sum / (double)MA_DEPTH;
    }
}

/* ---------------- Pressure ---------------- */

static void pressure_start_sequence(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    current_channel = 1;
    sConfig.Channel      = ADC_CHANNEL_5;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
    if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) Error_Handler();
}

static float adc_counts_to_volts(uint32_t raw_count)
{
    return ((float)raw_count / ADC_MAX_COUNT) * VREF_VOLTS;
}

static float sensor_volts_to_psi(float sensor_volts)
{
    float psi = (sensor_volts - SENSOR_V_AT_ZERO) /
                (SENSOR_VOLTAGE_SPAN / SENSOR_PSI_MAX);
    if (psi < 0.0f)           psi = 0.0f;
    if (psi > SENSOR_PSI_MAX) psi = SENSOR_PSI_MAX;
    return psi;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    ADC_ChannelConfTypeDef sConfig = {0};

    if (current_channel == 1)
    {
        raw_sensor1 = HAL_ADC_GetValue(&hadc1);
        current_channel = 2;
        sConfig.Channel      = ADC_CHANNEL_6;
        sConfig.Rank         = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
        sConfig.SingleDiff   = ADC_SINGLE_ENDED;
        sConfig.OffsetNumber = ADC_OFFSET_NONE;
        sConfig.Offset       = 0;
        HAL_ADC_ConfigChannel(&hadc1, &sConfig);
        HAL_ADC_Start_IT(&hadc1);
    }
    else if (current_channel == 2)
    {
        raw_sensor2 = HAL_ADC_GetValue(&hadc1);
        data_ready = 1;
    }
}

/* ---------------- UART ---------------- */

static void uart_print(const char *message)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)message,
                      (uint16_t)strlen(message), UART_TIMEOUT_MS);
}

static void print_combined_line(void)
{
    char buffer[128];

    /* Wait for both MAs to fill before showing averages */
    if (flow1.filled < MA_DEPTH || flow2.filled < MA_DEPTH || p_filled < MA_DEPTH)
    {
        if (!startup_msg_sent)
        {
            uart_print("Please allow up to 10 seconds for the moving means to fill...\r\n");
            startup_msg_sent = 1;
        }
        return;
    }


    /* Pressure MA */
    float p1_sum = 0, p2_sum = 0;
    for (int i = 0; i < MA_DEPTH; i++) { p1_sum += p1_buffer[i]; p2_sum += p2_buffer[i];}
    float p1_avg = p1_sum / (float)MA_DEPTH;
    float p2_avg = p2_sum / (float)MA_DEPTH;

    snprintf(buffer, sizeof(buffer),
             "F1: %.2f mL/min | F2: %.2f mL/min | S1: %.2f PSI | S2: %.2f PSI\r\n",
             flow1.avg_mLmin, flow2.avg_mLmin, p1_avg, p2_avg);

    uart_print(buffer);
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
