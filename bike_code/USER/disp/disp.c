/*
 * disp.c - the one place that knows which panel is fitted.
 *
 * Holds a text cache of what is currently on the glass, so ui.c can hand over
 * every row every frame and only real changes cost anything.  The two panels
 * want that at different granularities:
 *
 *   SH1106  has a 1 KB GRAM in RAM; drawing is nearly free and the cost is the
 *           SPI push, which is page-granular.  So: redraw a changed row into
 *           GRAM wholesale, then let oled_refresh_gram() push only the pages
 *           whose bytes differ.
 *   ILI9163 has no frame buffer; every character is a 256-byte burst straight
 *           to the panel.  So: compare character by character and send only
 *           the cells that changed.  A frame where two digits move costs
 *           ~0.4 ms instead of ~4 ms for the row.
 */

#include "display.h"
#include "disp_port.h"
#include <string.h>

#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
  #include "tft.h"
  /* Colour is free here; keep it quiet and legible. */
  #define DISP_FG        TFT_WHITE
  #define DISP_BG        TFT_BLACK
  #define DISP_BANNER_FG TFT_CYAN
  #if TFT_FONT_8X16
    #define TFT_CELL(col, row, ch, fg, bg)  tft_char8x16((col), (row), (ch), (fg), (bg))
    #define TFT_LINE(col, row, s, fg, bg)   tft_str8x16((col), (row), (s), (fg), (bg))
    #define TFT_CELL_H  16u
  #else
    #define TFT_CELL(col, row, ch, fg, bg)  tft_char6x8((col), (row), (ch), (fg), (bg))
    #define TFT_LINE(col, row, s, fg, bg)   tft_str6x8((col), (row), (s), (fg), (bg))
    #define TFT_CELL_H  8u
  #endif
#else
  #include "oled.h"
#endif

/* What the panel is believed to be showing. */
static char disp_cache[DISP_ROWS][DISP_COLS + 1];
static uint16_t disp_color[DISP_ROWS];
static uint8_t disp_valid = 0;
static uint8_t disp_units = 0;      /* pages or cells pushed this frame */

#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
/* Keep on_glass across invalidate(): forgetting the cache does not erase the
 * lower half of a large glyph. A later menu/boot row must erase it explicitly. */
static uint8_t heading_on_glass, heading_valid, heading_host;

static void heading_clear(uint16_t color)
{
    tft_fill_rect(0, (uint8_t)(DISP_ROW_PITCH * TFT_CELL_H),
                  tft_width(), (uint8_t)(DISP_ROW_PITCH * TFT_CELL_H), color);
    disp_units++;
}
#endif

static void cache_reset(void)
{
    memset(disp_cache, 0, sizeof disp_cache);
    disp_valid = 0;
#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
    heading_valid = 0;
#endif
}

void disp_invalidate(void)
{
    cache_reset();
#if DISPLAY_DRIVER == DISPLAY_OLED_SH1106
    oled_invalidate();
#endif
}

void disp_init(void)
{
#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
    tft_init(TFT_PORTRAIT);
    heading_on_glass = 0;
#else
    oled_init();
    oled_clear(Pen_Clear);
#endif
    cache_reset();
}

void disp_clear_all(void)
{
#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
    tft_clear(DISP_BG);
    heading_on_glass = 0;
#else
    oled_clear(Pen_Clear);
#endif
    cache_reset();
}

void disp_row(uint8_t row, const char *text)
{
    disp_row_color(row, text, row == 0u ? DISP_CYAN : 0xFFFFu);
}

void disp_row_color(uint8_t row, const char *text, uint16_t fg)
{
    char buf[DISP_COLS + 1];
    uint8_t n, i;

    if (row >= DISP_ROWS)
        return;

#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
    if (row == 1u && heading_on_glass)
    {
        heading_clear(DISP_BG);
        heading_on_glass = heading_valid = 0;
        memset(disp_cache[row], 0, sizeof disp_cache[row]);
    }
#endif

    /* Pad to the full width: that is what lets a shorter string erase what was
       there before without a separate clear. */
    for (n = 0; n < DISP_COLS && text != 0 && text[n] != '\0'; n++)
        buf[n] = text[n];
    for (i = n; i < DISP_COLS; i++)
        buf[i] = ' ';
    buf[DISP_COLS] = '\0';

    if (disp_valid && fg == disp_color[row] &&
        memcmp(buf, disp_cache[row], DISP_COLS) == 0)
        return;

#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
    {
        for (i = 0; i < DISP_COLS; i++)
        {
            if (disp_valid && fg == disp_color[row] && buf[i] == disp_cache[row][i])
                continue;           /* this cell is already right */
            TFT_CELL(i, (uint8_t)(row * DISP_ROW_PITCH), buf[i], fg, DISP_BG);
            disp_units++;
        }
    }
#else
    /* oled_showstring writes into GRAM only; the SPI cost is decided later by
       oled_refresh_gram()'s per-page compare. */
    oled_showstring(row, 0, (uint8_t *)buf);
#endif

    memcpy(disp_cache[row], buf, DISP_COLS + 1);
    disp_color[row] = fg;
}

void disp_control_heading(uint8_t host)
{
#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
    const char *label = host ? "HOST CONTROL" : "REMOTE CTRL";
    const uint16_t bg = host ? TFT_BLUE : TFT_GREEN;
    const uint16_t fg = host ? TFT_WHITE : TFT_BLACK;
    uint8_t col, n = (uint8_t)strlen(label);

    host = host != 0;
    if (disp_valid && heading_valid && heading_on_glass && heading_host == host)
        return;

    heading_clear(bg);
#if TFT_FONT_8X16 || (DISP_ROW_PITCH >= 2u && (DISP_ROW_PITCH % 2u) == 0u)
    col = (uint8_t)(((tft_width() / 8u) - n) / 2u);
    tft_str8x16(col, (uint8_t)(DISP_ROW_PITCH * TFT_CELL_H / 16u), label, fg, bg);
#else
    col = (uint8_t)((DISP_COLS - n) / 2u);
    TFT_LINE(col, DISP_ROW_PITCH, label, fg, bg);
#endif
    disp_units = (uint8_t)(disp_units + n);
    memset(disp_cache[1], 0, sizeof disp_cache[1]);
    heading_on_glass = heading_valid = 1;
    heading_host = host;
#else
    disp_row(1, host ? "*** HOST CONTROL ***" : "=== REMOTE CTRL ===");
#endif
}

uint8_t disp_flush(void)
{
    uint8_t n;

#if DISPLAY_DRIVER == DISPLAY_OLED_SH1106
    disp_units = oled_refresh_gram();
#endif
    /* The TFT is already on the glass; disp_row counted the cells. */

    disp_valid = 1;
    n = disp_units;
    disp_units = 0;
    return n;
}

void disp_lamp_test(void)
{
#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
    tft_lamp_test(1500);
#else
    /* 0xA5 lights every pixel from the controller and ignores GRAM, so a white
       flash proves power + CS + SPI + reset + init all work. */
    oled_write_byte(0x81, OLED_CMD);
    oled_write_byte(0xff, OLED_CMD);
    oled_write_byte(0xa5, OLED_CMD);
    HAL_Delay(2000);
    oled_write_byte(0xa4, OLED_CMD);
    oled_write_byte(0x81, OLED_CMD);
    oled_write_byte(0xcf, OLED_CMD);
#endif
    disp_invalidate();
}

void disp_grid_test(void)
{
#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
    /* Outline of the addressable area plus a tick at every text-row boundary.
       A missing edge names the fault: no right edge => the column range or
       MADCTL is wrong, no bottom edge => the row range. */
    uint8_t r;

    tft_clear(DISP_BG);
    tft_draw_rect(0, 0, tft_width(), tft_height(), TFT_WHITE);
    for (r = 0; r < DISP_ROWS; r++)
        tft_draw_hline(2, (uint8_t)(r * DISP_ROW_PITCH * TFT_CELL_H), 8, TFT_RED);
    HAL_Delay(3000);

    tft_clear(DISP_BG);
    for (r = 0; r < DISP_ROWS; r++)
        TFT_LINE(0, (uint8_t)(r * DISP_ROW_PITCH), "0.23456789ABCDEFGHIJKLMNO",
                 DISP_FG, DISP_BG);
    HAL_Delay(3000);
#else
    uint8_t r;

    oled_clear(Pen_Clear);
    oled_drawline(0,   0,   127, 0,   Pen_Write);
    oled_drawline(0,   63,  127, 63,  Pen_Write);
    oled_drawline(0,   0,   0,   63,  Pen_Write);
    oled_drawline(127, 0,   127, 63,  Pen_Write);
    for (r = 0; r < 6; r++)
        oled_drawline(3, (uint8_t)(r * 12), 9, (uint8_t)(r * 12), Pen_Write);
    oled_refresh_gram();
    HAL_Delay(3000);

    oled_clear(Pen_Clear);
    for (r = 0; r < 5; r++)
        oled_printf(r, 0, "%u.23456789ABCDEFGHIJ", (unsigned)r);
    oled_refresh_gram();
    HAL_Delay(3000);
#endif
    disp_invalidate();
}
