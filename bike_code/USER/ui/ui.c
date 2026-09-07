/*
 * ui.c - stage 1: OLED bring-up page.
 *
 * Proves three things on hardware:
 *   1. the panel lights up at all (SPI1 / DC / RST wiring + init sequence),
 *   2. the refresh path works from the main loop while balance() keeps running,
 *   3. how long a full-screen refresh actually costs, so we know the budget
 *      before the real pages go in.
 */

#include "ui.h"
#include "A_include.h"
#include "oled.h"

volatile uint32_t ui_refresh_us   = 0;
volatile uint32_t ui_render_us    = 0;
volatile uint8_t  ui_pages_pushed = 0;
volatile uint32_t ui_loop_hz      = 0;

/* Main-loop pass counter, sampled once a second into ui_loop_hz. */
static volatile uint32_t loop_ticks = 0;

/* Set by ui_request_redraw(), cleared once the frame has been drawn. */
static volatile uint8_t ui_redraw_req = 0;

void ui_request_redraw(void)
{
    ui_redraw_req = 1;
}

/* SYSCLK is 180 MHz (HSE 12 MHz x 180 / 6 / 2), so cycles / 180 = microseconds. */
#define CPU_MHZ     180u

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

#if UI_LAMP_TEST
/* Uses only public driver calls so oled.c stays a verbatim copy of the
 * reference.  0xA5/0xA4 are "entire display ON" / "resume from GRAM". */
static void oled_lamp_test(void)
{
    oled_write_byte(0x81, OLED_CMD);    /* contrast ...        */
    oled_write_byte(0xff, OLED_CMD);    /* ... to maximum      */
    oled_write_byte(0xa5, OLED_CMD);    /* every pixel on      */
    HAL_Delay(2000);
    oled_write_byte(0xa4, OLED_CMD);    /* back to GRAM        */
    oled_write_byte(0x81, OLED_CMD);
    oled_write_byte(0xcf, OLED_CMD);    /* back to init value  */
}
#endif

#if UI_GRID_TEST
static void oled_grid_test(void)
{
    uint8_t r;

    /* Phase 1: the outline of the whole addressable area. */
    oled_clear(Pen_Clear);
    oled_drawline(0,   0,   127, 0,   Pen_Write);   /* top,    y = 0   */
    oled_drawline(0,   63,  127, 63,  Pen_Write);   /* bottom, y = 63  */
    oled_drawline(0,   0,   0,   63,  Pen_Write);   /* left,   x = 0   */
    oled_drawline(127, 0,   127, 63,  Pen_Write);   /* right,  x = 127 */

    /* A stub at each text-row boundary: y = 0, 12, 24, 36, 48, 60. */
    for (r = 0; r < 6; r++)
        oled_drawline(3, r * 12, 9, r * 12, Pen_Write);

    oled_refresh_gram();
    HAL_Delay(3000);

    /* Phase 2: every text row, so clipped or missing rows are countable. */
    oled_clear(Pen_Clear);
    for (r = 0; r < 5; r++)
        oled_printf(r, 0, "%u.23456789ABCDEFGHIJ", (unsigned)r);
    oled_refresh_gram();
    HAL_Delay(3000);
}
#endif

void ui_init(void)
{
    dwt_init();

#if UI_PIN_TEST
    oled_port_pin_test();       /* never returns */
#endif

    oled_init();                /* also does oled_port_init(): SPI + DC/RST */

#if UI_LAMP_TEST
    oled_lamp_test();
#endif

#if UI_GRID_TEST
    oled_grid_test();
#endif

    oled_clear(Pen_Clear);
    oled_printf(0, 0, "==== BIKE CTRL ====");
    oled_printf(1, 0, "starting up ...");
    oled_refresh_gram();
}

void ui_task(void)
{
    static uint32_t next_ms = 0;
    static uint32_t hz_ms   = 0;
    static uint32_t hz_last = 0;
    uint32_t now = HAL_GetTick();
    uint32_t t0;

    loop_ticks++;

    if ((uint32_t)(now - hz_ms) >= 1000u)
    {
        hz_ms      = now;
        ui_loop_hz = loop_ticks - hz_last;
        hz_last    = loop_ticks;
    }

    /* Unsigned wrap-safe comparison; HAL_GetTick() rolls over after 49 days.
     * An explicit request jumps the queue so input feels immediate. */
    if (!ui_redraw_req && (uint32_t)(now - next_ms) < UI_PERIOD_MS)
        return;
    ui_redraw_req = 0;
    next_ms = now;

    t0 = DWT->CYCCNT;

    oled_clear(Pen_Clear);

    /* Row 0 is a fixed banner and never carries a value: the top pixel rows of
     * this panel are faulty, so anything that has to stay readable lives on
     * rows 1-4.  M0 moved onto the roll line, which had room to spare.
     * The banner also keeps pages 0-1 clean, so the dirty-page check skips
     * them every frame. */
    oled_printf(0, 0, "==== BIKE CTRL ====");
    oled_printf(1, 0, "rol %8.3f M0:%d", imu.rol, (int)param.M0_Flag);
    oled_printf(2, 0, "zero%8.3f", param.angular_zero);
    oled_printf(3, 0, "fly %8.2f", odrive.now_speed0);
    /* gram = SPI push, rnd = GRAM render, p = pages pushed of 8. All from the
     * previous frame, which is why the numbers are one frame stale. */
    oled_printf(4, 0, "gram%4lu p%u rnd%4lu",
                (unsigned long)ui_refresh_us,
                (unsigned)ui_pages_pushed,
                (unsigned long)ui_render_us);

    ui_render_us = (DWT->CYCCNT - t0) / CPU_MHZ;

    t0 = DWT->CYCCNT;
    ui_pages_pushed = oled_refresh_gram();
    ui_refresh_us = (DWT->CYCCNT - t0) / CPU_MHZ;
}

