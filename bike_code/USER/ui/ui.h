/*
 * ui.h - OLED pages, the drive/menu mode machine and the remote command map.
 *
 * Everything here runs from the main loop in Init.c only.  Nothing in this
 * module may be called from balance() (UART8 RX ISR, 2.5 ms control loop):
 * a full OLED refresh is milliseconds of SPI and would wreck the balance.
 *
 * Remote protocol: the handset sends "(id,value)" frames on huart7, parsed by
 * the existing vofa_get(); vofa_apply() routes every non-digit id here.  The
 * handset sends PHYSICAL BUTTONS, never actions - this module decides what a
 * button means in the current mode, so the handset cannot get out of sync and
 * needs no state of its own.  Legacy VOFA+ digit keys '0'-'7' are untouched.
 *
 *   id   button   drive mode                       menu mode
 *   ---  -------  -------------------------------  ---------------------
 *   'M'  MODE     -> menu                          -> drive
 *   'U'  up       M1_Ctl += 0.1                    previous item
 *   'D'  down     M1_Ctl -= 0.1                    next item
 *   'L'  left     Servo_Ctl -= UI_SERVO_STEP       value -= step
 *   'R'  right    Servo_Ctl += UI_SERVO_STEP       value += step
 *   'K'  OK       stop: M1_Ctl=0, M1_Flag=0,       run the action item
 *                 steering ramps back to centre    (LOAD calm / LOAD wind)
 *   'H'  (none)   heartbeat, ~10 Hz from the handset, both modes
 *   'V'  (none)   set the selected menu item to value, clamped
 *   'I'  (none)   jump to menu item number value
 *
 * One frame is one step; press-and-hold is the handset repeating the same id.
 */

#ifndef __UI_H
#define __UI_H

#include <stdint.h>

/* --- refresh ------------------------------------------------------------ */

/* Idle refresh cadence.  These are numbers, not animation: 10 Hz reads fine
 * and costs a quarter of what 40 Hz did.  At 40 Hz the render + push (~2.1 ms
 * + ~2.4 ms measured) ate 18% of the main loop for no readability gain.
 * Anything that needs to feel instant calls ui_request_redraw() instead of
 * waiting for the next tick. */
#define UI_PERIOD_MS    100u

/* Minimum interval between ui_render_now() repaints, see its comment. */
#define UI_BOOT_MIN_MS  50u

/* --- bring-up diagnostics, all 0 for normal operation -------------------- */

/* 1 = ui_init() never returns: SCK/MOSI/DC/RST are driven as plain GPIO and
 * toggled 0 V <-> 3.3 V once a second, so a multimeter at the *module* end of
 * the cable proves wiring and pin mapping without a scope.  Bench only. */
#define UI_PIN_TEST     0

/* 1 = right after oled_init(), send 0xA5 "entire display ON" at max contrast
 * and hold it for 2 s.  0xA5 lights every pixel straight from the controller
 * and ignores GRAM, so a white flash at boot proves power + CS + SPI + reset
 * + init sequence all work. */
#define UI_LAMP_TEST    0

/* 1 = two geometry test patterns before the normal page: a border around the
 * full 128 x 64 area with a tick at every text-row boundary, then all five
 * rows filled with a ruler.  Any missing edge names the fault exactly. */
#define UI_GRID_TEST    0

/* --- drive-mode limits --------------------------------------------------- */

#define UI_M1_STEP      0.1f            /* rear-wheel speed per up/down press */
#define UI_M1_MIN       (-3.0f)
#define UI_M1_MAX       3.0f

/* Servo_Ctl units per left/right press.  Servo_Ctl spans +-Servo_Delta (80),
 * and at the handset's ~10 Hz repeat 5 units/press means ~1.6 s from centre to
 * full lock.  balance() still slew-limits the result through
 * Steer_Speed_Limit(), so a larger step here cannot step the servo abruptly. */
#define UI_SERVO_STEP   5

/* No heartbeat for this long cuts the drive (not the balance - see ui.c). */
#define UI_RC_TIMEOUT_MS 500u

/* 1 = replace the bottom row of whichever page is showing with the render/push
 * instrumentation (gram us / pages pushed / render us).
 *
 * Read the numbers, then turn this OFF: the perf row changes every single
 * frame, so it keeps two pages and one row permanently dirty and destroys the
 * property it is measuring.  With it off, an idle menu page changes nothing at
 * all - zero rows re-rendered, zero pages pushed, and ui_task() costs only the
 * eight 128-byte memcmps.  The numbers it shows are therefore the worst case,
 * not the steady state. */
#define UI_SHOW_PERF    1

/* 1 = build the 'V' command (set the selected item to an absolute value).  It
 * is the only user of atof(), which drags in newlib's strtod/dtoa and costs
 * ~8 KB of flash - irrelevant against 2 MB, but it is the one knob here with a
 * real price tag, and the 6-button handset cannot send a float anyway.  Set to
 * 0 if the value only ever comes from the buttons.  'I' is unaffected (atoi is
 * already linked). */
#define UI_ENABLE_SET_CMD 0

/* 1 = up/down refuse to spin the rear wheel while param.M0_Flag is 0.  That
 * state normally means the bike is on its side, either because balance() hit
 * its 5 degree cutout or because balance was never enabled.  Set to 0 to allow
 * driving with the balance loop off. */
#define UI_DRIVE_REQUIRES_BALANCE 1

/* --- modes --------------------------------------------------------------- */

typedef enum
{
    UI_MODE_DRIVE = 0,
    UI_MODE_MENU  = 1
} ui_mode_t;

/* vofa_con() uses this instead of the old !upper_Flag gate. */
ui_mode_t ui_get_mode(void);

/* --- entry points -------------------------------------------------------- */

/* Enables the DWT cycle counter, brings up the panel and paints a splash.
 * Call once from Sys_All_Init(), before the main loop. */
void ui_init(void);

/* Call every main-loop pass; self-throttles to UI_PERIOD_MS. */
void ui_task(void);

/* Redraw on the next ui_task() pass instead of waiting out UI_PERIOD_MS, so a
 * key press or a remote command shows its effect immediately rather than up to
 * 100 ms later.  ISR-safe: it only sets a flag, and the worst a lost race can
 * cost is one extra frame. */
void ui_request_redraw(void);

/* One remote command frame.  Called from vofa_apply(), i.e. the main loop.
 * value is the frame's payload and may be an empty string. */
void ui_command(char id, const char *value);

/* Synchronous build + flush of a boot/status page, bypassing UI_PERIOD_MS.
 *
 * Sys_All_Init() blocks in three places before the main loop ever runs - the
 * wait for the first KEY0 press, the 31 s upright calibration and the 2 s
 * re-stand pause - and ui_task() is not reachable from any of them, so the
 * panel used to sit frozen on the splash for the whole time.  Call this from
 * inside those loops instead.
 *
 *   msg   status line (row 1), e.g. "Press KEY0 to start"
 *   secs  countdown shown on row 2; negative hides the row
 *
 * Row 3 always shows imu.rol, so the IMU is visibly alive while the loop is
 * blocked.  Self-limits to UI_BOOT_MIN_MS so a spin-wait cannot repaint at
 * main-loop speed; that floor is far below the 1 Hz the countdowns need, and
 * it keeps the call sites to a single line each. */
void ui_render_now(const char *msg, int secs);

/* --- instrumentation ----------------------------------------------------- */

/* Measured duration of the last oled_refresh_gram(), in microseconds. */
extern volatile uint32_t ui_refresh_us;

/* Cost of building the frame in GRAM, in microseconds.  Separate from
 * ui_refresh_us because it is pure CPU and does not shrink when the page-level
 * dirty check skips pages - the row-level cache in ui.c is what shrinks it. */
extern volatile uint32_t ui_render_us;

/* How many of the 8 pages the last refresh actually pushed over SPI. */
extern volatile uint8_t  ui_pages_pushed;

/* Main-loop passes per second - how much headroom the loop has.  Not on screen
 * any more (measured at ~1.17 M/s); read it in the debugger. */
extern volatile uint32_t ui_loop_hz;

#endif /* __UI_H */
