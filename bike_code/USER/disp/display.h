/*
 * display.h - which panel is fitted.
 *
 * Both panels hang off the same SPI1 + DC + RST lines on the board's OLED
 * header, so only one can be built at a time.  Flip DISPLAY_DRIVER, rebuild,
 * reflash.  Everything above the driver (ui.c) is written against the common
 * API at the bottom of this file and contains no #ifdef.
 *
 *   DISPLAY_OLED_SH1106   1.3" 128x64 mono SH1106, 6x12 font -> 21 cols x 5 rows
 *   DISPLAY_TFT_ILI9163   1.8" ILI9163B (LQ SGP18T); orientation and font are
 *                         chosen below, see TFT_PORTRAIT / TFT_FONT_8X16
 *
 * Wiring differs in exactly one line: the OLED module's CS is tied to GND,
 * the TFT's goes to PA6 (header pin 7, labelled BUTTON_AD on the RM-A board).
 * Nothing else on the cable changes.
 */

#ifndef __DISPLAY_H
#define __DISPLAY_H

#include <stdint.h>

#define DISPLAY_OLED_SH1106   0
#define DISPLAY_TFT_ILI9163   1

#ifndef DISPLAY_DRIVER
#define DISPLAY_DRIVER        DISPLAY_TFT_ILI9163
#endif

/* ---- TFT orientation and font ----------------------------------------- *
 * The panel is 160x128 one way round and 128x160 the other, and the column
 * count that falls out decides whether the UI fits:
 *
 *            6x8 font        8x16 font
 *   landscape 26 x 16        20 x 8
 *   portrait  21 x 20        16 x 10
 *
 * The longest string the UI emits is 19 characters, so portrait only works
 * with the 6x8 font - and 21 columns is exactly what the OLED has, so both
 * panels then render identically with no string changes at all.
 * Portrait + 8x16 builds and degrades gracefully (disp_row truncates at
 * DISP_COLS), but 8 of the strings would lose their tails.
 */
#define TFT_PORTRAIT    1       /* 1 = 128 wide x 160 high */
#define TFT_FONT_8X16   0       /* 0 = 6x8, 1 = 8x16       */

/* Line spacing, in glyph heights.  The UI only has 5 rows of content, and on
 * a 160 px tall portrait screen with an 8 px font those 5 rows would occupy
 * the top 40 px and leave three quarters of the panel empty.  A pitch of N
 * puts N glyph heights between baselines.
 *
 *   portrait 6x8, 20 physical rows:  pitch 1 -> 5 rows in 40 of 160 px
 *                                    pitch 2 -> 5 rows in 80 px   (default)
 *                                    pitch 4 -> 5 rows fill all 160 px
 * Set it to 1 to pack the rows and use the spare ones for more content. */
#define DISP_ROW_PITCH  2u

/* ---- geometry the UI layer sees -------------------------------------- */
#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
  #if TFT_PORTRAIT
    #if TFT_FONT_8X16
      #define DISP_COLS   16u
      #define DISP_PHYS_ROWS 10u
    #else
      #define DISP_COLS   21u
      #define DISP_PHYS_ROWS 20u
    #endif
  #else
    #if TFT_FONT_8X16
      #define DISP_COLS   20u
      #define DISP_PHYS_ROWS 8u
    #else
      #define DISP_COLS   26u
      #define DISP_PHYS_ROWS 16u
    #endif
  #endif
  /* Rows the UI may address, after spacing. */
  #define DISP_ROWS   (DISP_PHYS_ROWS / DISP_ROW_PITCH)
#else
  #define DISP_COLS   21u       /* 128 px / 6 px glyph  */
  #define DISP_ROWS   5u        /* 64 px  / 12 px glyph */
#endif

/* Rows the status/menu pages actually use.  The OLED has exactly 5; the TFT
 * has 8 and the spare ones are left blank so both panels show the same thing.
 * The longest string the UI emits is 19 characters, so 20 columns is enough. */
#define DISP_UI_ROWS  5u

/* ---- the API ui.c is written against -------------------------------------
 * No #ifdef above this layer.  disp.c owns a text cache of what is currently
 * on the panel, so callers may repaint every row every frame and only the
 * characters that actually changed reach the glass.
 */

/* Bring up the transport and the panel, and blank it. */
void disp_init(void);

/* Blank the whole screen and forget the cache. */
void disp_clear_all(void);

/* Draw one row of text, padded/truncated to DISP_COLS.  Cheap when unchanged. */
void disp_row(uint8_t row, const char *text);

/* Finish the frame.  Returns how many units went to the panel since the last
 * call, and resets the count: GRAM pages for the OLED, character cells for the
 * TFT.  Either way it is "how much glass did this frame cost". */
uint8_t disp_flush(void);

/* Forget what is believed to be on screen, so the next frame repaints in full.
 * Call after anything that disturbs the panel behind this layer's back. */
void disp_invalidate(void);

/* Bring-up helpers; see ui.h's UI_LAMP_TEST / UI_GRID_TEST. */
void disp_lamp_test(void);
void disp_grid_test(void);

#endif /* __DISPLAY_H */
