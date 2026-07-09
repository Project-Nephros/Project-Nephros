/*
 * nephros_ui.h
 *
 *  Created on: Jul 9, 2026
 *      Author: chany
 */

#ifndef INC_NEPHROS_UI_H_
#define INC_NEPHROS_UI_H_

#include "main.h"
#include "nephros_types.h"
#include <stdbool.h>
#include <stdint.h>

/* Button pin */
#define NEPHROS_BTN_PORT        GPIOA
#define NEPHROS_BTN_PIN         GPIO_PIN_12
#define NEPHROS_DEBOUNCE_MS     50U

void NephrosUI_Init(void);

void NephrosUI_Write(const char *text);

void NephrosUI_ShowBoot(void);
void NephrosUI_ShowConsolePrompt(void);
void NephrosUI_RunStartupMenu(NephrosSetup *setup);
void NephrosUI_ShowSetupComplete(const NephrosSetup *setup);

bool NephrosUI_ButtonPressed(uint32_t now_ms);

void NephrosUI_NextView(void);
void NephrosUI_ForceRedraw(void);
void NephrosUI_ShowNormalIfChanged(const NephrosSensorData *sensor);
void NephrosUI_ShowMessage(const char *line1, const char *line2);

#endif /* INC_NEPHROS_UI_H_ */
