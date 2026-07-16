/*
 * nephros_flush.h
 *
 *  Flush menu / controller (post-session line flush).
 *
 *  Purpose:
 *  Adds the missing *trigger* for the flush feature. The pump primitive
 *  (FlushPump) and the STATE_MOTOR_FLUSH state already exist; this module is
 *  the user-facing part that lets an operator choose "flush after use" from
 *  the LCD, confirms it, kicks the state machine into STATE_MOTOR_FLUSH, and
 *  shows a countdown until it finishes.
 *
 *  Design notes:
 *  - Non-blocking. NephrosFlush_Update() is called once per main loop, exactly
 *    like NephrosSafety_Update(), and never uses HAL_Delay(). The safety loop
 *    keeps running the whole time.
 *  - Safety has priority. This module never touches the pump directly and never
 *    overrides a halt. If the safety system halts (air/temp/pressure), it moves
 *    the machine to STATE_EMERGENCY_STOP; this module detects that and aborts
 *    the flush cleanly.
 *  - Single-button friendly. Uses the existing debounced NEPHROS_BTN edge, with
 *    a two-press confirm so a stray press can't start the pumps.
 *
 *  Author: (Infra) - handoff to Manfred for LCD wording / menu placement.
 */

#ifndef INC_NEPHROS_FLUSH_H_
#define INC_NEPHROS_FLUSH_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    FLUSH_UI_IDLE = 0,   /* session over, offering the flush option        */
    FLUSH_UI_ARMED,      /* first press seen, waiting for confirm press     */
    FLUSH_UI_RUNNING,    /* flush in progress, showing countdown           */
    FLUSH_UI_DONE,       /* flush finished normally                        */
    FLUSH_UI_ABORTED     /* flush interrupted by a safety halt             */
} NephrosFlushUiState;

typedef struct
{
    /* True when this module wants to own the LCD (so main.c should NOT draw
     * the normal Temp/Pressure/Air views while a flush menu is on screen). */
    bool owns_lcd;

    /* When valid, main.c should show these two lines. Kept <=16 chars for the
     * 16x2 HD44780. */
    bool lcd_message_valid;
    const char *lcd_line1;
    const char *lcd_line2;

    NephrosFlushUiState ui_state;
} NephrosFlushOutput;

/*
 * Reset the flush menu back to IDLE. Call once when a session has just ended
 * (i.e. on entry to STATE_MOTOR_STOPPED) so the flush option is offered fresh.
 */
void NephrosFlush_Init(void);

/*
 * Main per-loop update.
 *
 *   button_pressed : the SAME debounced edge main.c already reads from
 *                    NephrosUI_ButtonPressed(). Pass it straight through.
 *   safety_halted  : safety_output.halted from NephrosSafety_Update().
 *   session_over   : true when no treatment is running and it's valid to offer
 *                    a flush (e.g. GetState() == STATE_MOTOR_STOPPED). When
 *                    false, the IDLE offer stays silent so the prompt never
 *                    appears mid-treatment. A flush already in progress keeps
 *                    running regardless.
 *   now_ms         : HAL_GetTick().
 *
 * Returns what to display and whether this module currently owns the LCD.
 * When a confirmed flush starts, this internally calls SetState(STATE_MOTOR_FLUSH).
 */
NephrosFlushOutput NephrosFlush_Update(bool button_pressed,
                                       bool safety_halted,
                                       bool session_over,
                                       uint32_t now_ms);

#endif /* INC_NEPHROS_FLUSH_H_ */
