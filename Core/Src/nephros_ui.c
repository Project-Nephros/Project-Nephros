/*
 * nephros_ui.c
 *
 *  Created on: Jul 9, 2026
 *      Author: chany
 */

#include "nephros_ui.h"
#include "liquidcrystal_i2c.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>

/*
 * USART2 is created in main.c by CubeMX.
 * This file uses it for the PC/Python UART menu.
 */
extern UART_HandleTypeDef huart2;

/* Startup menu settings */
#define CONSOLE_PASSWORD     "1234"
#define MAX_VOLUME_ML        5000U
#define MAX_DURATION_MIN     1440U

/*
 * LCD page options during normal operation.
 */
typedef enum
{
    VIEW_TEMP = 0,
    VIEW_PRES,
    VIEW_AIR,
    VIEW_COUNT
} LcdView;

/* Current LCD page state */
static LcdView current_view = VIEW_AIR;
static LcdView last_drawn_view = VIEW_COUNT;

/* Button debounce state */
static GPIO_PinState last_btn_state = GPIO_PIN_SET;
static uint32_t last_debounce_ms = 0;

/* Private helper function prototypes */
static void uart_read_line(char *buffer, size_t buffer_len, bool mask_input);
static bool parse_positive_u32(const char *text, uint32_t *value_out);
static uint32_t ask_for_positive_number(const char *prompt, uint32_t max_value);


/*
 * Initialises the LCD module.
 * Call once during startup after I2C has already been initialised.
 */
void NephrosUI_Init(void)
{
    HD44780_Init(2);
    HD44780_Clear();
}


/*
 * Sends text to the PC/Python UART console.
 * This is also used by the safety module for logging.
 */
void NephrosUI_Write(const char *text)
{
    if (text == NULL)
    {
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
 * Shows the initial boot message on the LCD.
 */
void NephrosUI_ShowBoot(void)
{
    HD44780_Clear();
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("Nephros booting");

    HAL_Delay(800);
}


/*
 * Shows the UART setup instruction on the LCD.
 * The system waits at this stage while the user enters setup values.
 */
void NephrosUI_ShowConsolePrompt(void)
{
    HD44780_Clear();

    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("UART setup");

    HD44780_SetCursor(0, 1);
    HD44780_PrintStr("Open console");
}


/*
 * Runs the full UART startup menu:
 * password -> volume -> duration -> required flow calculation.
 */
void NephrosUI_RunStartupMenu(NephrosSetup *setup)
{
    char password[24];
    char summary[160];

    if (setup == NULL)
    {
        return;
    }

    NephrosUI_Write("\r\n");
    NephrosUI_Write("====================================\r\n");
    NephrosUI_Write("     NEPHROS STARTUP SETUP MENU\r\n");
    NephrosUI_Write("====================================\r\n");

    while (1)
    {
        NephrosUI_Write("Password: ");
        uart_read_line(password, sizeof(password), true);

        if (strcmp(password, CONSOLE_PASSWORD) == 0)
        {
            break;
        }

        NephrosUI_Write("Incorrect password. Please try again.\r\n");
    }

    NephrosUI_Write("Access granted.\r\n\r\n");

    setup->volume_ml = ask_for_positive_number(
        "Enter volume in mL (1-5000): ",
        MAX_VOLUME_ML
    );

    setup->duration_min = ask_for_positive_number(
        "Enter duration in minutes (1-1440): ",
        MAX_DURATION_MIN
    );

    setup->required_flow_ml_min =
        (float)setup->volume_ml / (float)setup->duration_min;

    snprintf(
        summary,
        sizeof(summary),
        "\r\nSetup saved:\r\n"
        "Volume = %lu mL\r\n"
        "Duration = %lu min\r\n"
        "Required flow = %.1f mL/min\r\n",
        (unsigned long)setup->volume_ml,
        (unsigned long)setup->duration_min,
        setup->required_flow_ml_min
    );

    NephrosUI_Write(summary);
    NephrosUI_Write("Safety monitor is starting...\r\n\r\n");
}


/*
 * Shows setup confirmation on the LCD after volume/duration entry.
 */
void NephrosUI_ShowSetupComplete(const NephrosSetup *setup)
{
    char line2[17];

    if (setup == NULL)
    {
        return;
    }

    HD44780_Clear();

    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("Setup complete");

    snprintf(
        line2,
        sizeof(line2),
        "Flow %.1f mL/m",
        setup->required_flow_ml_min
    );

    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(line2);

    HAL_Delay(2000);

    HD44780_Clear();
    NephrosUI_ForceRedraw();
}


/*
 * Reads the physical push button with debounce.
 *
 * Returns true once per real button press.
 * In normal mode, main.c uses this to change LCD pages.
 * In halted mode, main.c passes this to the safety system as a re-check request.
 */
bool NephrosUI_ButtonPressed(uint32_t now_ms)
{
    GPIO_PinState now_state;
    bool pressed = false;

    now_state = HAL_GPIO_ReadPin(
        NEPHROS_BTN_GPIO_Port,
        NEPHROS_BTN_Pin
    );

    if (now_state == GPIO_PIN_RESET &&
        last_btn_state == GPIO_PIN_SET &&
        (uint32_t)(now_ms - last_debounce_ms) > NEPHROS_DEBOUNCE_MS)
    {
        pressed = true;
        last_debounce_ms = now_ms;
    }

    last_btn_state = now_state;

    return pressed;
}


/*
 * Moves to the next normal LCD view:
 * Temperature -> Pressure -> Air Status -> Temperature.
 */
void NephrosUI_NextView(void)
{
    current_view = (LcdView)((current_view + 1) % VIEW_COUNT);
    NephrosUI_ForceRedraw();
}


/*
 * Forces the next normal LCD draw to repaint,
 * even if the selected view has not changed.
 */
void NephrosUI_ForceRedraw(void)
{
    last_drawn_view = VIEW_COUNT;
}


/*
 * Shows the current normal LCD page if it needs redrawing.
 *
 * This is used only when there is no active warning/halt message.
 */
void NephrosUI_ShowNormalIfChanged(const NephrosSensorData *sensor)
{
    char line1[17] = {0};
    char line2[17] = {0};

    if (sensor == NULL)
    {
        return;
    }

    if (current_view == last_drawn_view)
    {
        return;
    }

    switch (current_view)
    {
        case VIEW_TEMP:
            snprintf(line1, sizeof(line1), "Temperature");
            snprintf(line2, sizeof(line2), "%.1f C", sensor->temperature_c);
            break;

        case VIEW_PRES:
            snprintf(line1, sizeof(line1), "Pressure");
            snprintf(line2, sizeof(line2), "%u", sensor->pressure);
            break;

        case VIEW_AIR:
            snprintf(line1, sizeof(line1), "Air Status");
            snprintf(
                line2,
                sizeof(line2),
                "%s",
                sensor->air_detected ? "DETECTED" : "Clear"
            );
            break;

        default:
            return;
    }

    HD44780_Clear();

    HD44780_SetCursor(0, 0);
    HD44780_PrintStr(line1);

    HD44780_SetCursor(0, 1);
    HD44780_PrintStr(line2);

    last_drawn_view = current_view;
}


/*
 * Shows a two-line LCD message.
 * Used for warnings, halt messages, and resume messages.
 */
void NephrosUI_ShowMessage(const char *line1, const char *line2)
{
    HD44780_Clear();

    HD44780_SetCursor(0, 0);
    HD44780_PrintStr((char *)line1);

    HD44780_SetCursor(0, 1);
    HD44780_PrintStr((char *)line2);

    NephrosUI_ForceRedraw();
}


/*
 * Reads one full line from the UART console.
 *
 * mask_input = true:
 *   password mode, prints '*'
 *
 * mask_input = false:
 *   normal mode, echoes typed characters
 */
static void uart_read_line(char *buffer, size_t buffer_len, bool mask_input)
{
    uint8_t received_char;
    size_t index = 0;

    if (buffer == NULL || buffer_len == 0)
    {
        return;
    }

    while (1)
    {
        if (HAL_UART_Receive(
                &huart2,
                &received_char,
                1,
                HAL_MAX_DELAY
            ) != HAL_OK)
        {
            continue;
        }

        /*
         * Some terminals send CR + LF.
         * If a leftover LF appears at the start of the next input, ignore it.
         */
        if (received_char == '\n' && index == 0)
        {
            continue;
        }

        /* Enter key */
        if (received_char == '\r' || received_char == '\n')
        {
            NephrosUI_Write("\r\n");
            break;
        }

        /* Backspace key */
        if (received_char == '\b' || received_char == 0x7F)
        {
            if (index > 0)
            {
                index--;
                NephrosUI_Write("\b \b");
            }

            continue;
        }

        /* Normal printable character */
        if (received_char >= 32 && received_char <= 126)
        {
            if (index < buffer_len - 1)
            {
                buffer[index++] = (char)received_char;

                if (mask_input)
                {
                    NephrosUI_Write("*");
                }
                else
                {
                    HAL_UART_Transmit(
                        &huart2,
                        &received_char,
                        1,
                        HAL_MAX_DELAY
                    );
                }
            }
        }
    }

    buffer[index] = '\0';
}


/*
 * Converts a text input into a positive uint32_t value.
 *
 * Returns true if the input is valid.
 * Returns false for empty text, negative numbers, decimals, letters, or overflow.
 */
static bool parse_positive_u32(const char *text, uint32_t *value_out)
{
    const char *start;
    char *end;
    unsigned long parsed_value;

    if (text == NULL || value_out == NULL)
    {
        return false;
    }

    start = text;

    while (isspace((unsigned char)*start))
    {
        start++;
    }

    if (*start == '\0' || *start == '-')
    {
        return false;
    }

    errno = 0;
    parsed_value = strtoul(start, &end, 10);

    while (isspace((unsigned char)*end))
    {
        end++;
    }

    if (start == end ||
        *end != '\0' ||
        errno == ERANGE ||
        parsed_value == 0UL ||
        parsed_value > UINT32_MAX)
    {
        return false;
    }

    *value_out = (uint32_t)parsed_value;
    return true;
}


/*
 * Repeatedly asks the user for a positive whole number until valid.
 */
static uint32_t ask_for_positive_number(const char *prompt, uint32_t max_value)
{
    char input[24];
    uint32_t value;

    while (1)
    {
        NephrosUI_Write(prompt);
        uart_read_line(input, sizeof(input), false);

        if (parse_positive_u32(input, &value) && value <= max_value)
        {
            return value;
        }

        NephrosUI_Write(
            "Invalid input. Enter a positive whole number in range.\r\n"
        );
    }
}
