/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Project Nephros — Fluid Safety Monitor
  *                   STM32L432KC (Nucleo-32)
  ******************************************************************************
  * NOTE: If your CubeMX-regenerated main.c has different values inside
  *       SystemClock_Config / MX_*_Init, TRUST CubeMX's version over this
  *       file. Only the USER CODE BEGIN/END blocks are authoritative here.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "liquidcrystal_i2c.h"
#include <stdio.h>
#include <string.h>
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
#define LED_TEMP_PORT   GPIOA
#define LED_TEMP_PIN    GPIO_PIN_11
#define LED_PRES_PORT   GPIOB
#define LED_PRES_PIN    GPIO_PIN_5
#define LED_KILL_PORT   GPIOB
#define LED_KILL_PIN    GPIO_PIN_4
#define BTN_PORT        GPIOA
#define BTN_PIN         GPIO_PIN_12

#define TEMP_NORMAL_LOW_C       35.0f
#define TEMP_NORMAL_HIGH_C      38.0f

/*
 * Prototype severe-temperature limits.
 * Confirm the final limits with your team.
 */
#define TEMP_HALT_LOW_C         33.0f
#define TEMP_HALT_HIGH_C        40.0f
#define PRESSURE_HIGH_LIMIT     200U
/* Pressure must remain invalid for this long before halting */
#define PRESSURE_CONFIRM_MS     3000U
/* Write one sensor log line every second */
#define SENSOR_LOG_PERIOD_MS    1000U
#define DEBOUNCE_MS             50U

/*MENU LOGIC*/
#define CONSOLE_PASSWORD     "1234"
#define MAX_VOLUME_ML        5000U
#define MAX_DURATION_MIN     1440U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef enum
{
    VIEW_TEMP = 0,
    VIEW_PRES,
    VIEW_AIR,
    VIEW_COUNT
} LcdView;

typedef enum
{
    SYSTEM_RUNNING = 0,
    SYSTEM_HALTED
} SystemState;

typedef enum
{
    FAULT_NONE = 0,
    FAULT_AIR,
    FAULT_TEMP_LOW,
    FAULT_TEMP_HIGH,
    FAULT_PRESSURE
} FaultCode;

/* LCD / button state */
static LcdView current_view = VIEW_AIR;
static LcdView last_drawn_view = VIEW_COUNT;

static GPIO_PinState last_btn_state = GPIO_PIN_SET;
static uint32_t last_debounce_ms = 0;

/* Demo sensor values — replace later with live sensor data */
static float temperature_c = 36.5f;
static uint16_t pressure = 150;
static bool air_detected = false;

/* Startup-menu values */
static uint32_t setup_volume_ml = 0;
static uint32_t setup_duration_min = 0;
static float required_flow_ml_min = 0.0f;

/* Safety state */
static SystemState system_state = SYSTEM_RUNNING;
static FaultCode latched_fault = FAULT_NONE;

/* Minor temperature warning state */
static bool temperature_warning_active = false;

/* Pressure persistence timer */
static bool pressure_timer_active = false;
static uint32_t pressure_out_start_ms = 0;

/* Logging timer */
static uint32_t last_sensor_log_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void read_sensors(void);

static bool handle_button(void);

static void draw_normal_view(LcdView view);
static void draw_temperature_warning(void);
static void show_halt_screen(const char *line1, const char *line2);

static void uart_write(const char *text);
static void uart_read_line(char *buffer, size_t buffer_len, bool mask_input);
static bool parse_positive_u32(const char *text, uint32_t *value_out);
static uint32_t ask_for_positive_number(const char *prompt, uint32_t max_value);
static void run_console_setup(void);

static bool temperature_is_alert(void);
static bool temperature_is_severe(FaultCode *fault_out);

static bool pressure_is_out_of_range(void);
static bool pressure_out_of_range_long_enough(uint32_t now);

static FaultCode find_critical_fault(uint32_t now);
static const char *fault_name(FaultCode fault);

static void log_event(uint32_t now, const char *event_name, const char *detail);
static void log_sensor_reading(uint32_t now);

static void latch_system_halt(FaultCode fault, uint32_t now);
static void resume_system(uint32_t now);
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
  /*dbg_system_clock = SystemCoreClock;
  dbg_systick_ctrl = SysTick->CTRL;
  dbg_systick_load = SysTick->LOAD;
  dbg_systick_val  = SysTick->VAL;
  dbg_primask      = __get_PRIMASK();
  dbg_tick_before_lcd = HAL_GetTick();*/
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
    /* USER CODE BEGIN 2 */
  HD44780_Init(2);

  HD44780_Clear();
  HD44780_SetCursor(0, 0);
  HD44780_PrintStr("Nephros booting");
  HAL_Delay(800);

  HD44780_Clear();
  HD44780_SetCursor(0, 0);
  HD44780_PrintStr("UART setup");
  HD44780_SetCursor(0, 1);
  HD44780_PrintStr("Open console");


  /* Wait here until password and setup values are entered */
  run_console_setup();

  /* Brief LCD confirmation of the calculated flow requirement */
  char flow_screen[17];

  HD44780_Clear();
  HD44780_SetCursor(0, 0);
  HD44780_PrintStr("Setup complete");

  snprintf(
      flow_screen,
      sizeof(flow_screen),
      "Flow %.1f mL/m",
      required_flow_ml_min
  );

  HD44780_SetCursor(0, 1);
  HD44780_PrintStr(flow_screen);

  HAL_Delay(2000);

  HD44780_Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    	uint32_t now = HAL_GetTick();
    	    FaultCode current_fault;
    	    bool button_pressed;
    	    bool temp_warning;

    	    /* Read latest sensor values */
    	    read_sensors();

    	    /* Write timestamped sensor data periodically */
    	    log_sensor_reading(now);

    	    /* Detect one debounced button press */
    	    button_pressed = handle_button();

    	    /*
    	     * Critical-fault check:
    	     * - air: immediate
    	     * - severe temperature: immediate
    	     * - pressure: only after 3 continuous seconds invalid
    	     */
    	    current_fault = find_critical_fault(now);

    	    /* =========================================================
    	     * HALTED STATE
    	     * ========================================================= */
    	    if (system_state == SYSTEM_HALTED)
    	    {
    	        /* Keep the pump-stop / kill output active */
    	        HAL_GPIO_WritePin(
    	            LED_KILL_PORT,
    	            LED_KILL_PIN,
    	            GPIO_PIN_SET
    	        );

    	        HAL_GPIO_WritePin(
    	            LED_TEMP_PORT,
    	            LED_TEMP_PIN,
    	            GPIO_PIN_RESET
    	        );

    	        HAL_GPIO_WritePin(
    	            LED_PRES_PORT,
    	            LED_PRES_PIN,
    	            GPIO_PIN_RESET
    	        );

    	        /*
    	         * In halted mode, the button is a manual re-check request.
    	         * It does NOT cycle LCD views.
    	         */
    	        if (button_pressed)
    	        {
    	            log_event(now, "RECHECK_REQUESTED", "");

    	            /* Read fresh values before deciding whether to resume */
    	            read_sensors();

    	            now = HAL_GetTick();
    	            current_fault = find_critical_fault(now);

    	            if (current_fault == FAULT_NONE)
    	            {
    	                resume_system(now);

    	                /* Force the normal LCD view to redraw */
    	                last_drawn_view = VIEW_COUNT;
    	                temperature_warning_active = false;

    	                /*
    	                 * Demo-only confirmation delay.
    	                 * Replace later with non-blocking timing when PID is active.
    	                 */
    	                HAL_Delay(750);
    	            }
    	            else
    	            {
    	                latched_fault = current_fault;

    	                log_event(
    	                    now,
    	                    "RECHECK_FAILED",
    	                    fault_name(current_fault)
    	                );

    	                show_halt_screen(
    	                    "FAULT STILL ON",
    	                    fault_name(current_fault)
    	                );
    	            }
    	        }

    	        HAL_Delay(20);
    	        continue;
    	    }

    	    /* =========================================================
    	     * RUNNING STATE — critical faults override everything
    	     * ========================================================= */
    	    if (current_fault != FAULT_NONE)
    	    {
    	        latch_system_halt(current_fault, now);

    	        HAL_Delay(20);
    	        continue;
    	    }

    	    /*
    	     * System is running normally at this point.
    	     * Release the pump-stop / kill output.
    	     */
    	    HAL_GPIO_WritePin(
    	        LED_KILL_PORT,
    	        LED_KILL_PIN,
    	        GPIO_PIN_RESET
    	    );

    	    /*
    	     * Pressure is not an immediate warning anymore.
    	     * It becomes a stop alarm only after 3 continuous seconds.
    	     */
    	    HAL_GPIO_WritePin(
    	        LED_PRES_PORT,
    	        LED_PRES_PIN,
    	        GPIO_PIN_RESET
    	    );

    	    /* Normal-mode button behaviour: switch LCD screen */
    	    if (button_pressed)
    	    {
    	        current_view = (current_view + 1) % VIEW_COUNT;
    	        last_drawn_view = VIEW_COUNT;
    	    }

    	    /* =========================================================
    	     * MINOR TEMPERATURE WARNING
    	     * ========================================================= */
    	    temp_warning = temperature_is_alert();

    	    if (temp_warning)
    	    {
    	        HAL_GPIO_WritePin(
    	            LED_TEMP_PORT,
    	            LED_TEMP_PIN,
    	            GPIO_PIN_SET
    	        );

    	        if (!temperature_warning_active)
    	        {
    	            temperature_warning_active = true;

    	            log_event(
    	                now,
    	                "TEMP_WARNING_ON",
    	                "OUTSIDE_35_TO_38C"
    	            );

    	            draw_temperature_warning();
    	        }
    	    }
    	    else
    	    {
    	        HAL_GPIO_WritePin(
    	            LED_TEMP_PORT,
    	            LED_TEMP_PIN,
    	            GPIO_PIN_RESET
    	        );

    	        if (temperature_warning_active)
    	        {
    	            temperature_warning_active = false;

    	            log_event(
    	                now,
    	                "TEMP_WARNING_CLEARED",
    	                ""
    	            );

    	            /* Force normal screen redraw */
    	            last_drawn_view = VIEW_COUNT;
    	        }

    	        if (current_view != last_drawn_view)
    	        {
    	            draw_normal_view(current_view);
    	            last_drawn_view = current_view;
    	        }
    	    }

    	    HAL_Delay(20);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin PB4 PB5 */
  GPIO_InitStruct.Pin = LD3_Pin|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void uart_write(const char *text)
{
    if (text == NULL) {
        return;
    }

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)text,
        (uint16_t)strlen(text),
        HAL_MAX_DELAY
    );
}

/*
 * Reads a line from the UART console.
 *
 * mask_input = true:
 *   used for password entry, prints *
 *
 * mask_input = false:
 *   used for numbers, prints the typed characters normally
 */
static void uart_read_line(char *buffer, size_t buffer_len, bool mask_input)
{
    uint8_t received_char;
    size_t index = 0;

    if (buffer == NULL || buffer_len == 0) {
        return;
    }

    while (1) {
        if (HAL_UART_Receive(&huart2, &received_char, 1, HAL_MAX_DELAY) != HAL_OK) {
            continue;
        }

        /*
         * Some terminals send CR + LF when Enter is pressed.
         * Ignore a leftover LF at the beginning of the next input.
         */
        if (received_char == '\n' && index == 0) {
            continue;
        }

        /* Enter pressed */
        if (received_char == '\r' || received_char == '\n') {
            uart_write("\r\n");
            break;
        }

        /* Backspace pressed */
        if (received_char == '\b' || received_char == 0x7F) {
            if (index > 0) {
                index--;
                uart_write("\b \b");
            }
            continue;
        }

        /* Accept normal printable characters only */
        if (received_char >= 32 && received_char <= 126) {
            if (index < buffer_len - 1) {
                buffer[index++] = (char)received_char;

                if (mask_input) {
                    uart_write("*");
                } else {
                    HAL_UART_Transmit(&huart2, &received_char, 1, HAL_MAX_DELAY);
                }
            }
        }
    }

    buffer[index] = '\0';
}

static bool parse_positive_u32(const char *text, uint32_t *value_out)
{
    const char *start;
    char *end;
    unsigned long parsed_value;

    if (text == NULL || value_out == NULL) {
        return false;
    }

    start = text;

    while (isspace((unsigned char)*start)) {
        start++;
    }

    if (*start == '\0' || *start == '-') {
        return false;
    }

    errno = 0;
    parsed_value = strtoul(start, &end, 10);

    while (isspace((unsigned char)*end)) {
        end++;
    }

    if (start == end ||
        *end != '\0' ||
        errno == ERANGE ||
        parsed_value == 0UL ||
        parsed_value > UINT32_MAX) {
        return false;
    }

    *value_out = (uint32_t)parsed_value;
    return true;
}

static uint32_t ask_for_positive_number(const char *prompt, uint32_t max_value)
{
    char input[24];
    uint32_t value;

    while (1) {
        uart_write(prompt);
        uart_read_line(input, sizeof(input), false);

        if (parse_positive_u32(input, &value) && value <= max_value) {
            return value;
        }

        uart_write("Invalid input. Enter a positive whole number in range.\r\n");
    }
}

static void run_console_setup(void)
{
    char password[24];
    char summary[140];

    uart_write("\r\n");
    uart_write("====================================\r\n");
    uart_write("     NEPHROS STARTUP SETUP MENU\r\n");
    uart_write("====================================\r\n");

    /* Password stage */
    while (1) {
        uart_write("Password: ");
        uart_read_line(password, sizeof(password), true);

        if (strcmp(password, CONSOLE_PASSWORD) == 0) {
            break;
        }

        uart_write("Incorrect password. Please try again.\r\n");
    }

    uart_write("Access granted.\r\n\r\n");

    /* Volume stage */
    setup_volume_ml = ask_for_positive_number(
        "Enter volume in mL (1-5000): ",
        MAX_VOLUME_ML
    );

    /* Duration stage */
    setup_duration_min = ask_for_positive_number(
        "Enter duration in minutes (1-1440): ",
        MAX_DURATION_MIN
    );

    /* Required flow rate in mL per minute */
    required_flow_ml_min =
        (float)setup_volume_ml / (float)setup_duration_min;

    snprintf(
        summary,
        sizeof(summary),
        "\r\nSetup saved:\r\n"
        "Volume = %lu mL\r\n"
        "Duration = %lu min\r\n"
        "Required flow = %.1f mL/min\r\n",
        (unsigned long)setup_volume_ml,
        (unsigned long)setup_duration_min,
        required_flow_ml_min
    );

    uart_write(summary);
    uart_write("Safety monitor is starting...\r\n\r\n");
}

/* Stub — replace body when real sensors are wired in */
static void read_sensors(void)
{
}

static bool handle_button(void)
{
    GPIO_PinState now = HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN);
    uint32_t t = HAL_GetTick();
    bool pressed = false;

    /* Falling edge: idle-high -> pressed-low */
    if (now == GPIO_PIN_RESET &&
        last_btn_state == GPIO_PIN_SET &&
        (t - last_debounce_ms) > DEBOUNCE_MS) {

        pressed = true;
        last_debounce_ms = t;
    }

    last_btn_state = now;
    return pressed;
}

static void draw_normal_view(LcdView v)
{
    char line1[17] = {0};
    char line2[17] = {0};

    switch (v) {
    case VIEW_TEMP:
        snprintf(line1, sizeof line1, "Temperature");
        snprintf(line2, sizeof line2, "%.1f C", temperature_c);
        break;
    case VIEW_PRES:
        snprintf(line1, sizeof line1, "Pressure");
        snprintf(line2, sizeof line2, "%u", pressure);
        break;
    case VIEW_AIR:
        snprintf(line1, sizeof line1, "Air Status");
        snprintf(line2, sizeof line2, "%s", air_detected ? "DETECTED" : "Clear");
        break;
    default:
        return;
    }

    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(line1);
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(line2);
}

static void draw_temperature_warning(void)
{
    HD44780_Clear();

    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("TEMP ALERT");

    HD44780_SetCursor(0, 1);
    HD44780_PrintStr("OUTSIDE 35-38C");

    /* Normal LCD view must redraw once warning clears */
    last_drawn_view = VIEW_COUNT;
}
static void show_halt_screen(const char *line1, const char *line2)
{
    HD44780_Clear();

    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(line1);

    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(line2);
}

static bool temperature_is_alert(void)
{
    return (temperature_c < TEMP_NORMAL_LOW_C ||
            temperature_c > TEMP_NORMAL_HIGH_C);
}

static bool temperature_is_severe(FaultCode *fault_out)
{
    if (temperature_c <= TEMP_HALT_LOW_C)
    {
        if (fault_out != NULL)
        {
            *fault_out = FAULT_TEMP_LOW;
        }

        return true;
    }

    if (temperature_c >= TEMP_HALT_HIGH_C)
    {
        if (fault_out != NULL)
        {
            *fault_out = FAULT_TEMP_HIGH;
        }

        return true;
    }

    return false;
}

static bool pressure_is_out_of_range(void)
{
    /*
     * Temporary one-sided threshold.
     * Replace with arterial/venous operating windows later.
     */
    return (pressure > PRESSURE_HIGH_LIMIT);
}

static bool pressure_out_of_range_long_enough(uint32_t now)
{
    if (!pressure_is_out_of_range())
    {
        if (pressure_timer_active)
        {
            log_event(
                now,
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
        pressure_out_start_ms = now;

        log_event(
            now,
            "PRESSURE_OUT_OF_RANGE_START",
            ""
        );
    }

    return ((uint32_t)(now - pressure_out_start_ms)
            >= PRESSURE_CONFIRM_MS);
}

static FaultCode find_critical_fault(uint32_t now)
{
    FaultCode temp_fault = FAULT_NONE;

    /* Highest-priority immediate stop */
    if (air_detected)
    {
        return FAULT_AIR;
    }

    /* Severe temperature stop only */
    if (temperature_is_severe(&temp_fault))
    {
        return temp_fault;
    }

    /* Pressure requires 3 seconds continuously out of range */
    if (pressure_out_of_range_long_enough(now))
    {
        return FAULT_PRESSURE;
    }

    return FAULT_NONE;
}

static const char *fault_name(FaultCode fault)
{
    switch (fault)
    {
        case FAULT_AIR:
            return "AIR DETECTED";

        case FAULT_TEMP_LOW:
            return "TEMP TOO LOW";

        case FAULT_TEMP_HIGH:
            return "TEMP TOO HIGH";

        case FAULT_PRESSURE:
            return "PRESSURE FAULT";

        default:
            return "UNKNOWN FAULT";
    }
}

static void log_event(
    uint32_t now,
    const char *event_name,
    const char *detail
)
{
    char line[128];

    snprintf(
        line,
        sizeof(line),
        "EVENT,%lu,%s,%s\r\n",
        (unsigned long)now,
        event_name,
        detail
    );

    uart_write(line);
}

static void log_sensor_reading(uint32_t now)
{
    char line[128];

    if ((uint32_t)(now - last_sensor_log_ms)
        < SENSOR_LOG_PERIOD_MS)
    {
        return;
    }

    last_sensor_log_ms = now;

    snprintf(
        line,
        sizeof(line),
        "DATA,%lu,%.1f,%u,%u,%s\r\n",
        (unsigned long)now,
        temperature_c,
        (unsigned int)pressure,
        air_detected ? 1U : 0U,
        (system_state == SYSTEM_HALTED)
            ? "HALTED"
            : "RUNNING"
    );

    uart_write(line);
}

static void latch_system_halt(
    FaultCode fault,
    uint32_t now
)
{
    if (system_state != SYSTEM_HALTED ||
        latched_fault != fault)
    {
        log_event(
            now,
            "HALT",
            fault_name(fault)
        );
    }

    system_state = SYSTEM_HALTED;
    latched_fault = fault;

    /* Pump-stop / kill output */
    HAL_GPIO_WritePin(
        LED_KILL_PORT,
        LED_KILL_PIN,
        GPIO_PIN_SET
    );

    HAL_GPIO_WritePin(
        LED_TEMP_PORT,
        LED_TEMP_PIN,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        LED_PRES_PORT,
        LED_PRES_PIN,
        GPIO_PIN_RESET
    );

    show_halt_screen(
        "SYSTEM HALTED",
        fault_name(fault)
    );
}

static void resume_system(uint32_t now)
{
    system_state = SYSTEM_RUNNING;
    latched_fault = FAULT_NONE;

    HAL_GPIO_WritePin(
        LED_KILL_PORT,
        LED_KILL_PIN,
        GPIO_PIN_RESET
    );

    log_event(
        now,
        "RESUMED",
        "MANUAL_RECHECK_OK"
    );

    show_halt_screen(
        "SAFETY CHECK",
        "RESUMED"
    );
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
    while (1) {
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
