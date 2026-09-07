/*
 * ui.h - OLED page rendering, menu model and mode state machine.
 *
 * Everything here runs from the main loop in Init.c only.  Nothing in this
 * module may be called from balance() (UART8 RX ISR, 2.5 ms control loop):
 * a full OLED refresh is milliseconds of SPI and would wreck the balance.
 *
 * Stage 1 scope: bring-up page only (one text line + live imu.rol + measured
 * refresh time).  Pages, menu and drive modes land in later stages.
 */

#ifndef __UI_H
#define __UI_H

#include <stdint.h>

#define UI_PERIOD_MS    25u     /* OLED refresh cadence, ~40 Hz */

/* --- bring-up diagnostics, both 0 for normal operation ------------------- */

/* 1 = ui_init() never returns: SCK/MOSI/DC/RST are driven as plain GPIO and
 * toggled 0 V <-> 3.3 V once a second, so a multimeter at the *module* end of
 * the cable proves wiring and pin mapping without a scope.  Bench only - the
 * main loop never starts, so the key/countdown never runs and the motors stay
 * disabled, but power the ODrives down anyway. */
#define UI_PIN_TEST     0

/* 1 = right after oled_init(), send 0xA5 "entire display ON" at max contrast
 * and hold it for 2 s before resuming from GRAM.  0xA5 lights every pixel
 * straight from the controller and ignores GRAM completely, so a white flash
 * at boot proves power + CS + SPI + reset + init sequence all work and moves
 * any remaining fault into the GRAM/addressing path. */
#define UI_LAMP_TEST    0

/* 1 = before the normal page starts, show two geometry test patterns:
 *
 *   phase 1 (3 s)  a 1 px border around the full 128 x 64 area, plus a tick at
 *                  every text-row boundary (y = 0, 12, 24, 36, 48, 60).  Any
 *                  missing edge names the fault exactly: no top line = row 0
 *                  unreachable, no bottom line = row 63, no right line = the
 *                  SH1106 column offset is wrong.
 *   phase 2 (3 s)  all five text rows filled with a ruler, so it is obvious
 *                  which row lands on which pixels and how many are clipped.
 */
#define UI_GRID_TEST    0

/* Enables the DWT cycle counter, brings up the panel and paints a splash.
 * Call once from Sys_All_Init(), before the main loop. */
void ui_init(void);

/* Call every main-loop pass; self-throttles to UI_PERIOD_MS. */
void ui_task(void);

/* Measured duration of the last full oled_refresh_gram(), in microseconds.
 * Readable from the debugger during bring-up. */
extern volatile uint32_t ui_refresh_us;

/* Main-loop passes since boot, and passes per second - tells us how much
 * headroom the loop actually has once the OLED is in it. */
extern volatile uint32_t ui_loop_hz;

#endif /* __UI_H */
