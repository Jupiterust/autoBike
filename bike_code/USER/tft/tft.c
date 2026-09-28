/*
 * tft.c - ILI9163B driver, ported from the LQ_SGP18T reference via a C++
 * project.  See tft.h.
 */

#include "tft.h"
#include "tft_font.h"
#include "disp_port.h"
#include "stm32f4xx_hal.h"

/* One 8x16 cell = 128 px = 256 bytes, the largest single burst in this file.
 * Static, not on the stack. */
static uint8_t g_cell[8 * 16 * 2];

static uint8_t tft_portrait_flag = 0;

/* Reference init sequence, carried over byte for byte.
 * Encoding: {cmd, argc, args...}. */
static const uint8_t kInit[] = {
    0x11, 0,                                    /* sleep out                */
    0x3a, 1, 0x55,                              /* 16 bits/pixel (RGB565)   */
    0x26, 1, 0x04,                              /* gamma curve select       */
    0xf2, 1, 0x01,
    0xe0, 15, 0x3f, 0x25, 0x1c, 0x1e, 0x20, 0x12, 0x2a, 0x90,
              0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00,   /* positive gamma */
    0xe1, 15, 0x20, 0x20, 0x20, 0x20, 0x05, 0x00, 0x15, 0xa7,
              0x3d, 0x18, 0x25, 0x2a, 0x2b, 0x2b, 0x3a,   /* negative gamma */
    0xb1, 2, 0x00, 0x00,                        /* frame rate               */
    0xb4, 1, 0x07,                              /* inversion control        */
    0xc0, 2, 0x0a, 0x02,                        /* power control 1          */
    0xc1, 1, 0x02,                              /* power control 2          */
    0xc5, 2, 0x4f, 0x5a,                        /* VCOM control             */
    0xc7, 1, 0x40,
    0x2a, 4, 0x00, 0x00, 0x00, 0xa8,            /* column address range     */
    0x2b, 4, 0x00, 0x00, 0x00, 0xb3,            /* row address range        */
};

static void tft_cmd(uint8_t c)
{
    disp_set_dc(0);
    disp_write(c);
}

static void tft_data(uint8_t d)
{
    disp_set_dc(1);
    disp_write(d);
}

/* CS stays low for the whole session (disp_port_init holds it there): the
 * 0x2A/0x2B parameters and the 0x2C pixel stream that follows must sit inside
 * one CS-low period, or the controller discards the command.
 *
 * Raw RAM coordinates, no offset applied - used by tft_clear(), which wants to
 * scrub the whole RAM including the parts the glass does not show. */
static void tft_set_window_raw(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye)
{
    tft_cmd(0x2a);
    tft_data(0x00); tft_data(xs);
    tft_data(0x00); tft_data(xe);

    tft_cmd(0x2b);
    tft_data(0x00); tft_data(ys);
    tft_data(0x00); tft_data(ye);

    tft_cmd(0x2c);              /* everything after this is GRAM pixels */
    disp_set_dc(1);
}

/* Logical (visible-area) coordinates -> RAM.  Everything that draws content
 * goes through here; see TFT_X_OFFSET in tft.h. */
static void tft_set_window(uint8_t xs, uint8_t ys, uint8_t xe, uint8_t ye)
{
    tft_set_window_raw((uint8_t)(xs + TFT_X_OFFSET), (uint8_t)(ys + TFT_Y_OFFSET),
                       (uint8_t)(xe + TFT_X_OFFSET), (uint8_t)(ye + TFT_Y_OFFSET));
}

void tft_init(uint8_t portrait)
{
    uint32_t i;

    disp_port_init();
    tft_portrait_flag = portrait ? 1u : 0u;

    /* Datasheet: reset pulse >= 10 us, then >= 120 ms before commands. */
    disp_reset_pulse(50, 120);

    for (i = 0; i < sizeof(kInit);)
    {
        uint8_t c = kInit[i++];
        uint8_t n = kInit[i++];
        uint8_t k;

        tft_cmd(c);
        for (k = 0; k < n; k++)
            tft_data(kInit[i++]);

        /* Sleep Out needs 120 ms: the oscillator and charge pump have to
           settle, and configuration sent during that window is dropped.  The
           reference wrote "10", but that was a bare NOP loop on a 600 MHz
           part - copying the number would be far shorter than its intent. */
        if (c == 0x11)
            HAL_Delay(120);
    }

    /* MADCTL: scan direction, i.e. landscape vs portrait */
    tft_cmd(0x36);
    tft_data(tft_portrait_flag ? 0xc0 : 0xa0);
    tft_cmd(0xb7);
    tft_data(0x00);
    tft_cmd(0x29);              /* display on */

    HAL_Delay(20);

    tft_clear(TFT_BLACK);
}

void tft_clear(uint16_t c)
{
    /* Clear the whole controller RAM, which is larger than the visible area,
       so power-up noise cannot be left around the edges.
       The RAM is 132 columns x 162 rows; MADCTL's row/column exchange is what
       makes landscape 162 wide.  The reference hard-coded the landscape
       window, which would run the X range off the end of the RAM in portrait,
       so the two axes swap with the orientation. */
    const uint8_t xe = tft_portrait_flag ? (TFT_RAM_H - 1u) : (TFT_RAM_W - 1u);
    const uint8_t ye = tft_portrait_flag ? (TFT_RAM_W - 1u) : (TFT_RAM_H - 1u);

    tft_set_window_raw(0, 0, xe, ye);
    disp_fill16(c, (uint32_t)TFT_RAM_W * TFT_RAM_H);
}

void tft_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t c)
{
    if (w == 0u || h == 0u)
        return;

    tft_set_window(x, y, (uint8_t)(x + w - 1u), (uint8_t)(y + h - 1u));
    disp_fill16(c, (uint32_t)w * h);
}

void tft_draw_pixel(uint8_t x, uint8_t y, uint16_t c)
{
    tft_set_window(x, y, x, y);
    disp_fill16(c, 1);
}

void tft_draw_hline(uint8_t x, uint8_t y, uint8_t w, uint16_t c)
{
    tft_fill_rect(x, y, w, 1, c);
}

void tft_draw_vline(uint8_t x, uint8_t y, uint8_t h, uint16_t c)
{
    tft_fill_rect(x, y, 1, h, c);
}

void tft_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t c)
{
    if (w == 0u || h == 0u)
        return;
    tft_draw_hline(x, y, w, c);
    tft_draw_hline(x, (uint8_t)(y + h - 1u), w, c);
    tft_draw_vline(x, y, h, c);
    tft_draw_vline((uint8_t)(x + w - 1u), y, h, c);
}

static uint8_t *put_px(uint8_t *p, uint16_t c)
{
    *p++ = (uint8_t)(c >> 8);
    *p++ = (uint8_t)(c & 0xffu);
    return p;
}

void tft_char6x8(uint8_t col, uint8_t row, char ch, uint16_t fg, uint16_t bg)
{
    const uint8_t *glyph;
    uint8_t *p = g_cell;
    uint8_t i, j, x, y;

    if (ch < ' ' || (uint8_t)ch > 0x7eu)
        ch = ' ';
    glyph = kFont6x8[(uint8_t)ch - 32];

    /* COLUMN major, bit0 is the top row; the window advances by row, so the
       outer loop walks rows. */
    for (j = 0; j < 8u; j++)
        for (i = 0; i < 6u; i++)
            p = put_px(p, (glyph[i] & (1u << j)) ? fg : bg);

    x = (uint8_t)(col * 6u);
    y = (uint8_t)(row * 8u);

    tft_set_window(x, y, (uint8_t)(x + 5u), (uint8_t)(y + 7u));
    disp_write_buf(g_cell, 6u * 8u * 2u);
}

void tft_str6x8(uint8_t col, uint8_t row, const char *s, uint16_t fg, uint16_t bg)
{
    uint8_t c;

    if (s == 0)
        return;
    for (c = col; *s != '\0' && c < (uint8_t)(tft_width() / 6u); c++, s++)
        tft_char6x8(c, row, *s, fg, bg);
}

void tft_char8x16(uint8_t col, uint8_t row, char ch, uint16_t fg, uint16_t bg)
{
    const uint8_t *glyph;
    uint8_t *p = g_cell;
    uint8_t i, j, x, y;

    if (ch < ' ' || (uint8_t)ch > 0x7eu)
        ch = ' ';
    glyph = kFont8x16[(uint8_t)ch - 32];

    /* ROW major here: 16 bytes, one per row, bit i = column i (bit0 left).
       This is the opposite of the 6x8 table above - same reference font, two
       different layouts.  Reusing one loop for both gives garbage. */
    for (j = 0; j < 16u; j++)           /* j = row    */
        for (i = 0; i < 8u; i++)        /* i = column */
            p = put_px(p, (glyph[j] & (1u << i)) ? fg : bg);

    x = (uint8_t)(col * 8u);
    y = (uint8_t)(row * 16u);

    tft_set_window(x, y, (uint8_t)(x + 7u), (uint8_t)(y + 15u));
    disp_write_buf(g_cell, 8u * 16u * 2u);
}

void tft_str8x16(uint8_t col, uint8_t row, const char *s, uint16_t fg, uint16_t bg)
{
    uint8_t c;

    if (s == 0)
        return;
    for (c = col; *s != '\0' && c < (uint8_t)(tft_width() / 8u); c++, s++)
        tft_char8x16(c, row, *s, fg, bg);
}

uint8_t tft_width(void)  { return tft_portrait_flag ? TFT_H : TFT_W; }
uint8_t tft_height(void) { return tft_portrait_flag ? TFT_W : TFT_H; }

void tft_lamp_test(uint32_t ms_each)
{
    static const uint16_t seq[3] = { TFT_RED, TFT_GREEN, TFT_BLUE };
    uint8_t i;

    for (i = 0; i < 3u; i++)
    {
        tft_clear(seq[i]);
        HAL_Delay(ms_each);
    }
    tft_clear(TFT_BLACK);
}

void tft_diag_loop(void)
{
    disp_port_init();

    for (;;)
    {
        /* 1: pin the panel in reset */
        HAL_GPIO_WritePin(DISP_RST_PORT, DISP_RST_PIN, GPIO_PIN_RESET);
        HAL_Delay(3000);

        /* 2: release reset, send nothing */
        HAL_GPIO_WritePin(DISP_RST_PORT, DISP_RST_PIN, GPIO_PIN_SET);
        HAL_Delay(3000);

        /* 3: full init, then display off */
        tft_init(tft_portrait_flag);
        tft_cmd(0x28);
        HAL_Delay(3000);

        /* 4: display on + flood red */
        tft_cmd(0x29);
        tft_clear(TFT_RED);
        HAL_Delay(3000);

        /* 5: flood blue */
        tft_clear(TFT_BLUE);
        HAL_Delay(3000);
    }
}
