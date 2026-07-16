/*
 * nephros_flush.c
 *
 *  See nephros_flush.h for the overview.
 *
 *  Flow:
 *
 *     IDLE  --btn-->  ARMED  --btn(within confirm window)-->  RUNNING
 *       ^               |                                        |
 *       |          (window times out)                    (FlushPump done)
 *       |               v                                        v
 *       +------------- IDLE                                     DONE
 *
 *     Any state --safety halt--> ABORTED
 *
 *  The actual pump timing lives in PumpMotor.c (FlushPump / kFlushDurationMs).
 *  This module only drives the UI and the state-machine trigger, and reads the
 *  remaining time back for the countdown.
 */

#include "nephros_flush.h"
#include "StateMachine.h"
#include "PumpMotor.h"

#include <stdio.h>

/*
 * How long (ms) the operator has to press the button a second time to confirm
 * the flush after the first press. If it lapses, we return to IDLE so the
 * pumps never start from a single stray press.
 */
#define FLUSH_CONFIRM_WINDOW_MS   4000U

/* How often to repaint the running countdown, in ms. */
#define FLUSH_COUNTDOWN_REDRAW_MS 500U

static NephrosFlushUiState ui_state = FLUSH_UI_IDLE;
static uint32_t armed_at_ms = 0U;
static uint32_t last_countdown_ms = 0U;

/* Holds the "NNNs left" line so we can return a pointer to a stable buffer. */
static char countdown_line[17];

static NephrosFlushOutput make_output(void)
{
    NephrosFlushOutput out;

    out.owns_lcd = false;
    out.lcd_message_valid = false;
    out.lcd_line1 = "";
    out.lcd_line2 = "";
    out.ui_state = ui_state;

    return out;
}

void NephrosFlush_Init(void)
{
    ui_state = FLUSH_UI_IDLE;
    armed_at_ms = 0U;
    last_countdown_ms = 0U;
}

NephrosFlushOutput NephrosFlush_Update(bool button_pressed,
                                       bool safety_halted,
                                       bool session_over,
                                       uint32_t now_ms)
{
    NephrosFlushOutput out = make_output();

    /*
     * Safety always wins. If the safety system has halted the machine at any
     * point during the offer or the flush itself, abort. We reset the pump-side
     * flush timer so a later flush starts clean (see FlushPump_Reset()).
     */
    if (safety_halted &&
        ui_state != FLUSH_UI_IDLE &&
        ui_state != FLUSH_UI_DONE)
    {
        FlushPump_Reset();
        ui_state = FLUSH_UI_ABORTED;
    }

    switch (ui_state)
    {
        case FLUSH_UI_IDLE:
        {
            /*
             * Only offer the flush once the treatment session is actually over.
             * Mid-treatment (session_over == false) this stays silent and hands
             * the LCD back to the normal view cycler.
             */
            if (!session_over)
            {
                break;   /* owns_lcd stays false */
            }

            /*
             * Offer the flush. NOTE: the "disconnect line" wording is a
             * safety prompt - flush is intended only with the patient line
             * disconnected. Manfred: confirm final wording / whether this
             * should instead be a dedicated menu item rather than the
             * post-session prompt.
             */
            out.owns_lcd = true;
            out.lcd_message_valid = true;
            out.lcd_line1 = "Flush after use?";
            out.lcd_line2 = "Press btn = yes";

            if (button_pressed)
            {
                armed_at_ms = now_ms;
                ui_state = FLUSH_UI_ARMED;
            }
            break;
        }

        case FLUSH_UI_ARMED:
        {
            out.owns_lcd = true;
            out.lcd_message_valid = true;
            out.lcd_line1 = "Disconnect line";
            out.lcd_line2 = "Press again=YES";

            /* Confirm window elapsed with no second press -> cancel. */
            if ((uint32_t)(now_ms - armed_at_ms) >= FLUSH_CONFIRM_WINDOW_MS)
            {
                ui_state = FLUSH_UI_IDLE;
                break;
            }

            if (button_pressed)
            {
                /*
                 * Confirmed. Kick the existing state machine into the flush
                 * state. FlushPump() (called from STATE_MOTOR_FLUSH) owns the
                 * fixed-rate run and timing.
                 */
                FlushPump_Reset();
                SetState(STATE_MOTOR_FLUSH);

                last_countdown_ms = 0U;   /* force an immediate first draw */
                ui_state = FLUSH_UI_RUNNING;
            }
            break;
        }

        case FLUSH_UI_RUNNING:
        {
            out.owns_lcd = true;

            /*
             * The state machine flips STATE_MOTOR_FLUSH -> STATE_MOTOR_STOPPED
             * when FlushPump() finishes. Watch for that to know we're done.
             */
            if (GetState() != STATE_MOTOR_FLUSH)
            {
                ui_state = FLUSH_UI_DONE;
                break;
            }

            /* Repaint the countdown at a fixed cadence. */
            if ((uint32_t)(now_ms - last_countdown_ms) >= FLUSH_COUNTDOWN_REDRAW_MS)
            {
                uint32_t remaining_s =
                    (FlushPump_RemainingMs() + 999U) / 1000U;   /* round up */

                snprintf(countdown_line, sizeof(countdown_line),
                         "%lus left", (unsigned long)remaining_s);

                last_countdown_ms = now_ms;

                out.lcd_message_valid = true;
                out.lcd_line1 = "Flushing system";
                out.lcd_line2 = countdown_line;
            }
            break;
        }

        case FLUSH_UI_DONE:
        {
            /*
             * Show completion once. We keep owning the LCD so main.c doesn't
             * immediately overwrite it with the normal view; drop ownership
             * after the message has been posted so normal views resume.
             */
            out.owns_lcd = false;
            out.lcd_message_valid = true;
            out.lcd_line1 = "Flush complete";
            out.lcd_line2 = "";
            break;
        }

        case FLUSH_UI_ABORTED:
        {
            /*
             * Safety halt interrupted the flush. Hand the LCD back to the
             * safety system, which is already displaying the fault. We just
             * post one line for the log-minded operator, then release.
             */
            out.owns_lcd = false;
            out.lcd_message_valid = true;
            out.lcd_line1 = "Flush aborted";
            out.lcd_line2 = "Safety halt";
            break;
        }

        default:
            NephrosFlush_Init();
            break;
    }

    out.ui_state = ui_state;
    return out;
}
