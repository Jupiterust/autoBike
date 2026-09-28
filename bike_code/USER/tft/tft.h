/*
 * tft.h - 1.8" 160x128 colour TFT (ILI9163B, LongQiu LQ_SGP18T) driver.
 *
 * Ported to C from a C++/FreeRTOS project: the class became tft_* functions,
 * its SpiBus became the shared USER/disp/disp_port.c, and osDelay became
 * HAL_Delay.  The drawing code, the init table and the two font layouts are
 * carried over unchanged - that combination has been made to light a panel on
 * this wiring, so it is not the place to be creative.
 *
 * Wiring is the board's OLED header, identical to the SH1106 except that CS
 * goes to PA6 (header pin 7) instead of being tied to GND.  See disp_port.h.
 *
 * Orientation is set by MADCTL (0x36) in tft_init():
 *   landscape 0xA0 -> 160 wide x 128 high   (what the UI uses)
 *   portrait  0xC0 -> 128 wide x 160 high
 * The controller's RAM is 162x132, larger than the visible area; clears run
 * over the RAM size so power-up noise cannot survive at the edges.
 *
 * NO FRAME BUFFER.  A full screen is 160*128*2 = 40 KB, which at 11.25 MHz
 * takes ~28 ms to push; buffering it would cost 40 KB of RAM and not be any
 * faster.  Everything draws straight to the panel, so the cost of an update is
 * proportional to the area redrawn - redraw only what changed.
 */

#ifndef __TFT_H
#define __TFT_H

#include <stdint.h>

/* RGB565 */
#define TFT_BLACK   0x0000u
#define TFT_WHITE   0xFFFFu
#define TFT_RED     0xF800u
#define TFT_GREEN   0x07E0u
#define TFT_BLUE    0x001Fu
#define TFT_YELLOW  0xFFE0u
#define TFT_CYAN    0x07FFu
#define TFT_PURPLE  0xF81Fu
#define TFT_ORANGE  0xFC08u
#define TFT_GRAY    0x8410u

/* Controller RAM, larger than the visible area */
#define TFT_RAM_W   162u
#define TFT_RAM_H   132u

/* Where the glass starts inside that RAM.
 *
 * The RAM is 132x162 but the panel is only 128x160, so the visible window sits
 * at an offset that varies between modules - this is the same family of quirk
 * as the SH1106's 2-column offset.  Neither the LQ reference nor the C++ port
 * applied any offset, which is why the first pixel row and column were being
 * written outside the visible area and lost.
 *
 * Confirm with UI_GRID_TEST: it outlines the full addressable area.  All four
 * edges visible => correct.  Top and/or left edge missing => raise that offset.
 * A blank stripe down the right or along the bottom => lower it. */
#define TFT_X_OFFSET   1u
#define TFT_Y_OFFSET   1u

/* Visible size in landscape */
#define TFT_W       160u
#define TFT_H       128u

/* Character cells.  Landscape is 160x128, portrait 128x160, so the two
 * orientations give very different column counts - and the column count is
 * what decides whether the UI's strings fit at all. */
#define TFT_COLS_6X8    (TFT_W / 6u)    /* landscape 26 */
#define TFT_ROWS_6X8    (TFT_H / 8u)    /* landscape 16 */
#define TFT_COLS_8X16   (TFT_W / 8u)    /* landscape 20 */
#define TFT_ROWS_8X16   (TFT_H / 16u)   /* landscape 8  */

#define TFT_P_COLS_6X8  (TFT_H / 6u)    /* portrait  21 */
#define TFT_P_ROWS_6X8  (TFT_W / 8u)    /* portrait  20 */
#define TFT_P_COLS_8X16 (TFT_H / 8u)    /* portrait  16 */
#define TFT_P_ROWS_8X16 (TFT_W / 16u)   /* portrait  10 */

/* portrait = 0 for the 160x128 landscape the UI expects. */
void tft_init(uint8_t portrait);

void tft_clear(uint16_t color);
void tft_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
void tft_draw_pixel(uint8_t x, uint8_t y, uint16_t color);
void tft_draw_hline(uint8_t x, uint8_t y, uint8_t w, uint16_t color);
void tft_draw_vline(uint8_t x, uint8_t y, uint8_t h, uint16_t color);
void tft_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);

/* col/row are character cells, not pixels. */
void tft_char6x8(uint8_t col, uint8_t row, char ch, uint16_t fg, uint16_t bg);
void tft_str6x8(uint8_t col, uint8_t row, const char *s, uint16_t fg, uint16_t bg);

void tft_char8x16(uint8_t col, uint8_t row, char ch, uint16_t fg, uint16_t bg);
void tft_str8x16(uint8_t col, uint8_t row, const char *s, uint16_t fg, uint16_t bg);

uint8_t tft_width(void);
uint8_t tft_height(void);

/* Bring-up: flood red, green, blue in turn.  A lit panel proves power, CS,
 * SPI, reset and the init sequence; wrong colours mean RGB/BGR order or
 * MADCTL, not a broken link. */
void tft_lamp_test(uint32_t ms_each);

/* Bring-up, NEVER RETURNS.  Cycles a 3 s stage at a time so the fault can be
 * placed by eye alone:
 *   1 RST held low          panel pinned in reset
 *   2 RST released          out of reset, uninitialised
 *   3 full init + 0x28      display off
 *   4 0x29 + flood red      display on
 *   5 flood blue
 * Whichever stage first looks wrong is where it breaks. */
void tft_diag_loop(void);

#endif /* __TFT_H */
