/* Real TFT cache; replace the TFT transport with a bounds-checked pixel canvas. */
#include "display.h"
#include "tft.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint16_t glass[160][128];
static unsigned calls, large_calls;
static uint16_t last_bg, last_fg;
static char last_label[32];
static uint8_t last_x, last_y;

uint8_t tft_width(void) { return 128; }
uint8_t tft_height(void) { return 160; }
void tft_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color)
{
    assert((unsigned)x + w <= 128 && (unsigned)y + h <= 160);
    for (unsigned j = y; j < (unsigned)y + h; j++)
        for (unsigned i = x; i < (unsigned)x + w; i++) glass[j][i] = color;
    calls++;
}
void tft_init(uint8_t portrait) { assert(portrait); }
void tft_clear(uint16_t color) { tft_fill_rect(0, 0, 128, 160, color); }
void tft_char6x8(uint8_t col, uint8_t row, char ch, uint16_t fg, uint16_t bg)
{ tft_fill_rect((uint8_t)(col * 6), (uint8_t)(row * 8), 6, 8, ch == ' ' ? bg : fg); }
void tft_char8x16(uint8_t col, uint8_t row, char ch, uint16_t fg, uint16_t bg)
{ tft_fill_rect((uint8_t)(col * 8), (uint8_t)(row * 16), 8, 16, ch == ' ' ? bg : fg); }
void tft_str6x8(uint8_t col, uint8_t row, const char *s, uint16_t fg, uint16_t bg)
{ while (*s) tft_char6x8(col++, row, *s++, fg, bg); }
void tft_str8x16(uint8_t col, uint8_t row, const char *s, uint16_t fg, uint16_t bg)
{
    snprintf(last_label, sizeof last_label, "%s", s);
    last_bg = bg; last_fg = fg;
    last_x = (uint8_t)(col * 8); last_y = (uint8_t)(row * 16);
    large_calls++;
    while (*s) tft_char8x16(col++, row, *s++, fg, bg);
}
void tft_lamp_test(uint32_t ms) { (void)ms; }
void tft_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t c)
{ tft_fill_rect(x, y, w, h, c); }
void tft_draw_hline(uint8_t x, uint8_t y, uint8_t w, uint16_t c)
{ tft_fill_rect(x, y, w, 1, c); }
void HAL_Delay(uint32_t ms) { (void)ms; }
int main(void)
{
    unsigned before;
    disp_init(); disp_clear_all(); disp_control_heading(0);
    assert(!strcmp(last_label, "REMOTE CTRL"));
    assert(last_bg == TFT_GREEN && last_fg == TFT_BLACK);
    assert(last_y == 16 && last_x + strlen(last_label) * 8 <= 128);
    disp_flush(); before = calls;
    disp_control_heading(0); disp_flush(); assert(calls == before);
    disp_control_heading(1);
    assert(!strcmp(last_label, "HOST CONTROL"));
    assert(last_bg == TFT_BLUE && last_fg == TFT_WHITE);
    assert(glass[31][0] == TFT_BLUE);
    disp_row(2, "LINK:WAIT BAL:1 RUN:0");
    assert(glass[31][0] == TFT_BLUE); /* No overlap into the following row. */
    disp_flush(); before = calls;
    disp_control_heading(1); disp_flush(); assert(calls == before);

    disp_invalidate(); disp_row(1, ">av_kp 1.4000");
    assert(glass[28][20] == TFT_BLACK && glass[31][127] == TFT_BLACK);
    disp_flush(); before = calls;
    disp_row(1, ">av_kp 1.4000"); disp_flush(); assert(calls == before);
    disp_control_heading(0); disp_flush();
    disp_invalidate(); disp_control_heading(0); disp_flush();
    assert(!strcmp(last_label, "REMOTE CTRL"));
    disp_row(1, "Press KEY0 to start");
    assert(glass[31][127] == TFT_BLACK);
    disp_clear_all(); disp_control_heading(1); disp_flush();
    assert(last_bg == TFT_BLUE && large_calls >= 5);
    puts("TFT colour, 8x16 geometry, idle cache and transition clearing passed");
    return 0;
}

