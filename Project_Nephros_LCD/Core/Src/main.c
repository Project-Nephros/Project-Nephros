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

#define TEMP_WARN_C     25.0f
#define TEMP_KILL_C     35.0f
#define PRES_WARN       200
#define DEBOUNCE_MS     50
#define ALERT_ROTATE_MS 2000   /* show each active alert for 2 s before flipping */

/* Alert IDs — indices into the warn arrays */
#define ALERT_TEMP      0
#define ALERT_PRES      1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef enum {
    VIEW_TEMP = 0,    /* Temperature */
    VIEW_PRES,        /* Pressure    */
    VIEW_AIR,         /* Air Status  */
    VIEW_COUNT
} LcdView;

/* >>> CHANGE THIS to switch the LCD view <<<
 *   VIEW_TEMP (0) — Temperature
 *   VIEW_PRES (1) — Pressure
 *   VIEW_AIR  (2) — Air Status
 */
static LcdView       current_view     = 2;

static GPIO_PinState last_btn_state   = GPIO_PIN_SET;
static uint32_t      last_debounce_ms = 0;
static LcdView       last_drawn_view  = VIEW_COUNT;
static bool          last_was_alert   = false;

/* Multi-alert rotation state */
static int           alert_show_idx       = 0;
static uint32_t      last_alert_change_ms = 0;

/* Dummy sensor values — edit live via the debugger to exercise states */
static float    temperature_c = 25.0f;
static uint16_t pressure      = 150;
static bool     air_detected  = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void read_sensors(void);
static void handle_button(void);
static void draw_normal_view(LcdView v);
static void draw_alert(const char *line1, const char *line2);
static void enter_kill_state(const char *reason);
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
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */
    HD44780_Init(2);
    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("Nephros booting");
    HAL_Delay(800);
    HD44780_Clear();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        read_sensors();

        /* 1. KILL STATE — checked first, never returns */
        if (temperature_c > TEMP_KILL_C) {
            enter_kill_state("OVERTEMP");
        }
        if (air_detected) {
            enter_kill_state("AIR IN LINE");
        }

        /* 2. WARNING STATE — evaluate every threshold every loop */
        bool temp_warn = (temperature_c > TEMP_WARN_C);
        bool pres_warn = (pressure      > PRES_WARN);

        /* WARNING LEDs — one per condition, lit continuously while active.
         * These do NOT follow the LCD rotation; each LED tracks its own threshold. */
        HAL_GPIO_WritePin(LED_TEMP_PORT, LED_TEMP_PIN,
                          temp_warn ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_PRES_PORT, LED_PRES_PIN,
                          pres_warn ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_KILL_PORT, LED_KILL_PIN, GPIO_PIN_RESET);

        /* Collect every active alert for the LCD rotation */
        int active_alerts[2];
        int n_active = 0;
        if (temp_warn) active_alerts[n_active++] = ALERT_TEMP;
        if (pres_warn) active_alerts[n_active++] = ALERT_PRES;

        /* 3. INPUT — button (if wired) advances current_view by 1 on press */
        handle_button();

        /* 4. LCD */
        if (n_active > 0) {
            uint32_t t = HAL_GetTick();
            bool first_entry    = !last_was_alert;
            bool time_to_rotate = (t - last_alert_change_ms) >= ALERT_ROTATE_MS;

            if (first_entry) {
                alert_show_idx       = 0;
                last_alert_change_ms = t;
            } else if (time_to_rotate) {
                alert_show_idx       = (alert_show_idx + 1) % n_active;
                last_alert_change_ms = t;
            }

            /* Clamp in case an alert just cleared and shrank the list */
            if (alert_show_idx >= n_active) {
                alert_show_idx = 0;
            }

            int showing = active_alerts[alert_show_idx];

            /* Only repaint on entry or on rotation — not every loop */
            if (first_entry || time_to_rotate) {
                char buf[17];
                if (showing == ALERT_TEMP) {
                    snprintf(buf, sizeof buf, "T=%.1fC", temperature_c);
                    draw_alert("TEMP ALERT", buf);
                } else {
                    snprintf(buf, sizeof buf, "P=%u", pressure);
                    draw_alert("PRES ALERT", buf);
                }
            }
            last_was_alert = true;
        } else {
            /* No active warnings — show normal view */
            if (last_was_alert || current_view != last_drawn_view) {
                draw_normal_view(current_view);
                last_drawn_view = current_view;
                last_was_alert  = false;
            }
        }

        HAL_Delay(50);
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

    /** Configure the main internal regulator output voltage */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;     /* 4 MHz */
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;            /* → SYSCLK = 80 MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x10909CEC;            /* 100 kHz @ 80 MHz PCLK1 */
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
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
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Reset output pins low */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

    /* PA11 — Temp LED (output) */
    GPIO_InitStruct.Pin   = GPIO_PIN_11;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB4, PB5 — Air/Kill LED, Pres LED (outputs) */
    GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PA12 — Button (input, internal pull-up) */
    GPIO_InitStruct.Pin   = GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* Stub — replace body when real sensors are wired in */
static void read_sensors(void)
{
}

static void handle_button(void)
{
    GPIO_PinState now = HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN);
    uint32_t t = HAL_GetTick();

    /* Falling edge (idle-high → pressed) with debounce */
    if (now == GPIO_PIN_RESET && last_btn_state == GPIO_PIN_SET
        && (t - last_debounce_ms) > DEBOUNCE_MS) {
        current_view = (current_view + 1) % VIEW_COUNT;
        last_debounce_ms = t;
    }
    last_btn_state = now;
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

static void draw_alert(const char *line1, const char *line2)
{
    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr((char *)line1);
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr((char *)line2);
    last_drawn_view = VIEW_COUNT;   /* invalidate so normal view redraws on exit */
}

/* Critical failure — drive only the red LED, lock the LCD,
 * and spin forever. Physical reset required to exit. */
static void enter_kill_state(const char *reason)
{
    HAL_GPIO_WritePin(LED_TEMP_PORT, LED_TEMP_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PRES_PORT, LED_PRES_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_KILL_PORT, LED_KILL_PIN, GPIO_PIN_SET);

    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("SYSTEM HALTED");
    HD44780_SetCursor(0, 1);
    HD44780_PrintStr((char *)reason);

    while (1) {
        /* lock — physical reset required */
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
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
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
