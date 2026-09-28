/*
 * ui.c - OLED pages, drive/menu mode machine, menu model, remote dispatch.
 * See ui.h for the id command table (the contract the handset is written to).
 */

#include "ui.h"
#include "A_include.h"
#include "display.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile uint32_t ui_refresh_us   = 0;
volatile uint32_t ui_render_us    = 0;
volatile uint8_t  ui_pages_pushed = 0;
volatile uint32_t ui_loop_hz      = 0;

/* Main-loop pass counter, sampled once a second into ui_loop_hz. */
static volatile uint32_t loop_ticks = 0;

/* Set by ui_request_redraw(), cleared once the frame has been drawn. */
static volatile uint8_t ui_redraw_req = 0;

static ui_mode_t ui_mode = UI_MODE_DRIVE;

/* SYSCLK is 180 MHz (HSE 12 MHz x 180 / 6 / 2), so cycles / 180 = microseconds. */
#define CPU_MHZ     180u

/* Text columns per row: 128 px / 6 px glyph = 21. */
#define UI_COLS     DISP_COLS

void ui_request_redraw(void)
{
    ui_redraw_req = 1;
}

ui_mode_t ui_get_mode(void)
{
    return ui_mode;
}

/* ====================================================================== */
/* Remote link                                                            */
/* ====================================================================== */

static uint32_t rc_last_ms = 0;
static uint8_t  rc_seen    = 0;     /* a frame has arrived at least once */
static uint8_t  rc_link_ok = 0;

/* Latches on the first recognised button.  Deliberately not set by the
 * heartbeat or by unknown ids - see ui_boot_wait() in ui.h. */
static volatile uint8_t rc_btn_seen = 0;

/* ====================================================================== */
/* Menu model                                                             */
/* ====================================================================== */

#define MENU_ACT_NONE       0
#define MENU_ACT_LOAD_CALM  1
#define MENU_ACT_LOAD_WIND  2
#define MENU_ACT_TEST_STEP  3

typedef struct
{
    const char *name;
    float      *var;        /* NULL => action item, not a value */
    float       min;
    float       max;
    float       step;
    uint8_t     action;
} menu_item_t;

/* min/max are not cosmetic on rows 3..7: those gains have to stay negative in
 * the present control law, and letting one cross zero turns its loop into
 * positive feedback.  The clamp is the guard. */
static const menu_item_t MENU[] =
{
    { "av_kp", &param.angular_v_kp,          0.0f,  5.0f,   0.05f,   MENU_ACT_NONE },
    { "av_ki", &param.angular_v_ki,          0.0f,  1.0f,   0.005f,  MENU_ACT_NONE },
    { "av_kd", &param.angular_v_kd,          0.0f,  5.0f,   0.05f,   MENU_ACT_NONE },
    { "an_kp", &param.angular_kp,          -20.0f,  0.0f,   0.1f,    MENU_ACT_NONE },
    { "an_ki", &param.angular_ki,           -2.0f,  0.0f,   0.01f,   MENU_ACT_NONE },
    { "an_kd", &param.angular_kd,           -5.0f,  0.0f,   0.05f,   MENU_ACT_NONE },
    { "fw_kp", &param.fly_wheel_speed_kp,   -1.0f,  0.0f,   0.005f,  MENU_ACT_NONE },
    { "fw_ki", &param.fly_wheel_speed_ki,   -0.5f,  0.0f,   0.002f,  MENU_ACT_NONE },
    { "fw_kd", &param.fly_wheel_speed_kd,   -0.5f,  0.5f,   0.002f,  MENU_ACT_NONE },
    { "zero",  &param.angular_zero,         -5.0f,  5.0f,   0.01f,   MENU_ACT_NONE },
    { "sgain", &Servo_Gain_K,                0.0f,  0.01f,  0.0001f, MENU_ACT_NONE },
    { "LOADcalm", NULL, 0.0f, 0.0f, 0.0f, MENU_ACT_LOAD_CALM },
    { "LOADwind", NULL, 0.0f, 0.0f, 0.0f, MENU_ACT_LOAD_WIND },
    /* Fires the reproducible angle-setpoint step defined in Balane.h.  Put on
     * the menu rather than on a remote button so the handset firmware, which is
     * written against a published contract, does not have to change. */
    { "TEST step", NULL, 0.0f, 0.0f, 0.0f, MENU_ACT_TEST_STEP },
};

#define MENU_N      ((uint8_t)(sizeof(MENU) / sizeof(MENU[0])))
#define MENU_WINDOW 3       /* items visible at once (rows 1..3) */

static uint8_t menu_sel = 0;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* The two gain sets, the same numbers vofa_apply()'s '2'/'3' keys load, so a
 * menu load and a VOFA key leave the bike in exactly the same state.
 *
 * These ~13 stores have to land as one unit: balance() runs from the UART8 RX
 * ISR every 2.5 ms and would otherwise be able to execute one tick on a mix of
 * old and new gains.  A PRIMASK critical section is the whole fix - well under
 * 1 us against a 1.78 ms IMU frame, and unlike a "skip this tick" flag it does
 * not disturb the loop at all: no control tick is dropped, and Read_Encoder()
 * keeps consuming TIM8->CNT every tick, so no doubled-count spike is injected
 * into the speed loop.  PRIMASK is saved and restored rather than blindly
 * re-enabled, so this stays correct if it is ever called with interrupts
 * already masked. */
static void menu_load_preset(uint8_t wind)
{
    uint32_t pri = __get_PRIMASK();

    __disable_irq();

    if (wind)
    {
        param.angular_v_kp       =  1.2f;
        param.angular_v_ki       =  0.0f;
        param.angular_v_kd       =  1.16f;
        param.angular_kp         = -7.9f;
        param.angular_ki         =  0.0f;
        param.angular_kd         = -1.6f;
        param.fly_wheel_speed_kp = -0.17f;
        param.fly_wheel_speed_ki = -0.065f;
        param.fly_wheel_speed_kd =  0.0f;
    }
    else
    {
        param.angular_v_kp       =  1.4f;
        param.angular_v_ki       =  0.0f;
        param.angular_v_kd       =  1.115f;
        param.angular_kp         = -7.3f;
        param.angular_ki         =  0.0f;
        param.angular_kd         = -1.2f;
        param.fly_wheel_speed_kp = -0.16f;
        param.fly_wheel_speed_ki = -0.081f;   /* iter2 keep, no-wind set only */
        param.fly_wheel_speed_kd =  0.0f;
    }

    /* Mechanical zero, not a gain: both presets load the current one so a
       preset load cannot silently revert it. See param_init() in Balane.c. */
    param.angular_zero = -1.40f;
    param.Steer_Kp     =  1.0f;
    param.Steer_Ki     =  0.0f;
    param.Steer_Kd     =  0.0f;

    __set_PRIMASK(pri);
}

/* ====================================================================== */
/* Row-level dirty tracking                                               */
/* ====================================================================== */

/* ui_row only formats.  disp.c owns the cache of what is on the glass and
 * decides what that costs: for the OLED a changed row is redrawn into GRAM and
 * the per-page compare picks the SPI; for the TFT, which has no frame buffer,
 * only the character cells that differ are pushed. */
static void ui_row(uint8_t row, const char *fmt, ...)
{
    char    buf[UI_COLS + 1];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    disp_row(row, buf);
}

/* ====================================================================== */
/* Pages                                                                  */
/* ====================================================================== */

/* Row 0 is a fixed banner and never carries a value: the top pixel rows of
 * this panel are faulty.  It also never changes, so the row cache skips it and
 * the page-level check skips page 0 with it. */
#define UI_BANNER   "==== BIKE CTRL ===="

#if UI_SHOW_PERF
/* gram = SPI push us, p = pages pushed of 8, rnd = GRAM render us.  All from
 * the previous frame, so the numbers are one frame stale. */
static void ui_perf_row(uint8_t row)
{
    /* d = odrive_tx_drops, speed commands the USART3 DMA refused.  It must stay
     * at 0; a climbing count means a motor is being commanded less often than
     * the code believes, which is exactly how the rear wheel used to be dead. */
    ui_row(row, "g%lu p%u r%lu d%lu i%lu",
           (unsigned long)ui_refresh_us,
           (unsigned)ui_pages_pushed,
           (unsigned long)ui_render_us,
           (unsigned long)odrive_tx_drops,
           (unsigned long)uart_err_cnt[UART_ERR_IMU]);
}
#endif

static void ui_page_drive(void)
{
    ui_row(0, UI_BANNER);
    /* The number after the link state is how many times UART7 reception had to
     * be restarted.  Non-zero means the link really did break and recovered by
     * itself; still zero while the handset is silent means the trouble is on
     * the radio side, not here. */
    ui_row(1, "DRIVE M0:%d M1:%d %s%lu",
           (int)param.M0_Flag, (int)param.M1_Flag,
           rc_seen ? (rc_link_ok ? "RC" : "??") : "--",
           (unsigned long)uart_err_cnt[UART_ERR_REMOTE]);
    ui_row(2, "spd%6.2f srv%+4d", M1_Ctl, Servo_Ctl);
    ui_row(3, "rol %8.3f", imu.rol);
#if UI_SHOW_PERF
    ui_perf_row(4);
#else
    ui_row(4, "fly %8.2f", odrive.now_speed0);
#endif
}

static void ui_page_menu(void)
{
    const menu_item_t *m;
    uint8_t top, i;

    /* Three-item window that keeps the selection inside it. */
    if (menu_sel == 0)
        top = 0;
    else if (menu_sel >= (uint8_t)(MENU_N - 1))
        top = (uint8_t)(MENU_N - MENU_WINDOW);
    else
        top = (uint8_t)(menu_sel - 1);

    ui_row(0, UI_BANNER);

    for (i = 0; i < MENU_WINDOW; i++)
    {
        m = &MENU[top + i];

        if (m->var)
            ui_row((uint8_t)(1 + i), "%c%-6s%10.4f",
                   (top + i == menu_sel) ? '>' : ' ', m->name, *m->var);
        else
            ui_row((uint8_t)(1 + i), "%c%-9s  [OK]",
                   (top + i == menu_sel) ? '>' : ' ', m->name);
    }

#if UI_SHOW_PERF
    ui_perf_row(4);
#else
    m = &MENU[menu_sel];
    if (m->var)
        ui_row(4, "M0:%d step %.4f", (int)param.M0_Flag, m->step);
    else if (m->action == MENU_ACT_TEST_STEP)
        /* The pulse is refused unless the balance loop is closed, so say so
         * instead of letting OK look broken. */
        ui_row(4, "M0:%d %s", (int)param.M0_Flag,
               test_pulse_active() ? "TEST RUNNING"
                                   : (!param.M0_Flag        ? "need M0=1"
                                   : (fabs(error_zero) > TEST_ARM_MAX_DEG
                                                            ? "not settled"
                                                            : "press OK")));
    else
        ui_row(4, "M0:%d  press OK", (int)param.M0_Flag);
#endif
}

/* ====================================================================== */
/* Command handling                                                       */
/* ====================================================================== */

static void ui_drive_speed(float d)
{
#if UI_DRIVE_REQUIRES_BALANCE
    if (!param.M0_Flag)
        return;
#endif
    M1_Ctl = clampf(M1_Ctl + d, UI_M1_MIN, UI_M1_MAX);
    param.M1_Flag = 1;
}

static void ui_drive_steer(int d)
{
    int v = Servo_Ctl + d;

    if (v >  Servo_Delta) v =  Servo_Delta;
    if (v < -Servo_Delta) v = -Servo_Delta;

    Servo_Ctl = v;
    servo_con = 0;          /* a manual step cancels an auto-centre in flight */
}

static void ui_drive_stop(void)
{
    M1_Ctl        = 0.0f;
    param.M1_Flag = 0;
    servo_con     = 1;      /* ramp the steering back to centre */
}

static void ui_menu_nudge(int dir)
{
    const menu_item_t *m = &MENU[menu_sel];

    if (m->var == NULL)
        return;

    *m->var = clampf(*m->var + (float)dir * m->step, m->min, m->max);
}

static void ui_menu_action(void)
{
    switch (MENU[menu_sel].action)
    {
    case MENU_ACT_LOAD_CALM: menu_load_preset(0); break;
    case MENU_ACT_LOAD_WIND: menu_load_preset(1); break;
    case MENU_ACT_TEST_STEP: (void)test_pulse_start(); break;
    default:                                      break;
    }
}

void ui_command(char id, const char *value)
{
#if UI_ENABLE_SET_CMD
    const menu_item_t *m;
#endif
    int n;

    /* Any frame counts as a sign of life, including ones this build ignores. */
    rc_last_ms = HAL_GetTick();
    rc_seen    = 1;
    rc_link_ok = 1;

    switch (id)
    {
    case 'H':                   /* heartbeat: the timestamp above is the point */
        return;                 /* no redraw - it would defeat the 10 Hz cap   */

    case 'M':
        ui_mode = (ui_mode == UI_MODE_DRIVE) ? UI_MODE_MENU : UI_MODE_DRIVE;
        disp_invalidate();      /* the whole page changed, not just its values */
        break;

    case 'U':
        if (ui_mode == UI_MODE_DRIVE)
            ui_drive_speed(+UI_M1_STEP);
        else if (menu_sel > 0)
            menu_sel--;
        break;

    case 'D':
        if (ui_mode == UI_MODE_DRIVE)
            ui_drive_speed(-UI_M1_STEP);
        else if ((uint8_t)(menu_sel + 1) < MENU_N)
            menu_sel++;
        break;

    case 'L':
        if (ui_mode == UI_MODE_DRIVE)
            ui_drive_steer(-UI_SERVO_STEP);
        else
            ui_menu_nudge(-1);
        break;

    case 'R':
        if (ui_mode == UI_MODE_DRIVE)
            ui_drive_steer(+UI_SERVO_STEP);
        else
            ui_menu_nudge(+1);
        break;

    case 'K':
        if (ui_mode == UI_MODE_DRIVE)
            ui_drive_stop();
        else
            ui_menu_action();
        break;

#if UI_ENABLE_SET_CMD
    case 'V':                   /* absolute set of the selected item */
        m = &MENU[menu_sel];
        if (ui_mode == UI_MODE_MENU && m->var && value && value[0])
            *m->var = clampf((float)atof(value), m->min, m->max);
        break;
#endif

    case 'I':                   /* jump to menu item */
        if (value && value[0])
        {
            n = atoi(value);
            if (n >= 0 && n < (int)MENU_N)
                menu_sel = (uint8_t)n;
        }
        break;

    default:
        return;                 /* unknown id: no state change, no redraw */
    }

    /* Only reached by a recognised button: 'H' returned early above and an
     * unknown id returned from default. */
    rc_btn_seen = 1;

    ui_request_redraw();
}

/* Link lost cuts the drive but deliberately NOT the balance: dropping M0 on a
 * radio glitch would put the bike on the ground, which is worse than anything
 * a lost link can cause.  Balance stays owned by KEY_0 and by balance()'s own
 * 5 degree cutout. */
static void ui_rc_check(void)
{
    if (!rc_seen || !rc_link_ok)
        return;
    if ((uint32_t)(HAL_GetTick() - rc_last_ms) < UI_RC_TIMEOUT_MS)
        return;

    rc_link_ok    = 0;
    M1_Ctl        = 0.0f;
    param.M1_Flag = 0;
    servo_con     = 1;
    ui_request_redraw();
}

/* ====================================================================== */
/* Bring-up diagnostics (see ui.h)                                        */
/* ====================================================================== */

/* ====================================================================== */
/* Entry points                                                           */
/* ====================================================================== */

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void ui_init(void)
{
    dwt_init();

#if UI_PIN_TEST
    disp_port_pin_test();       /* never returns */
#endif

    disp_init();                /* transport + panel, whichever is fitted */

#if UI_LAMP_TEST
    disp_lamp_test();
#endif
#if UI_GRID_TEST
    disp_grid_test();
#endif

    disp_clear_all();
    ui_row(0, UI_BANNER);
    ui_row(1, "starting up ...");
    (void)disp_flush();
}

static void ui_flush(void)
{
    uint32_t t0 = DWT->CYCCNT;

    ui_pages_pushed = disp_flush();
    ui_refresh_us = (DWT->CYCCNT - t0) / CPU_MHZ;
}

void ui_boot_skip_reset(void)
{
    rc_btn_seen = 0;
}

uint8_t ui_boot_wait(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - t0) < ms)
    {
        led_task();
        vofa_apply();       /* so the handset actually works during the wait */
        vofa_con();

        if (rc_btn_seen)
            return 1;
    }
    return 0;
}

void ui_render_now(const char *msg, int secs)
{
    static uint32_t last_ms = 0;
    uint32_t now = HAL_GetTick();
    uint32_t t0;

    if ((uint32_t)(now - last_ms) < UI_BOOT_MIN_MS)
        return;
    last_ms = now;

    led_task();                 /* keep the chase alive while the boot blocks */

    t0 = DWT->CYCCNT;

    ui_row(0, UI_BANNER);
    ui_row(1, "%s", (msg != NULL) ? msg : "");

    if (secs >= 0)
        ui_row(2, "%d s left", secs);
    else
        ui_row(2, " ");

    ui_row(3, "rol %8.3f", imu.rol);
    ui_row(4, " ");

    ui_render_us = (DWT->CYCCNT - t0) / CPU_MHZ;

    ui_flush();
}

void ui_task(void)
{
    static uint32_t next_ms = 0;
    static uint32_t hz_ms   = 0;
    static uint32_t hz_last = 0;
    static uint32_t wd_ms   = 0;
    uint32_t now = HAL_GetTick();
    uint32_t t0;

    loop_ticks++;

    if ((uint32_t)(now - hz_ms) >= 1000u)
    {
        hz_ms      = now;
        ui_loop_hz = loop_ticks - hz_last;
        hz_last    = loop_ticks;
    }

    ui_rc_check();

    /* Re-arm any link whose reception died. Cheap, but no point doing it at
     * main-loop speed. */
    if ((uint32_t)(now - wd_ms) >= 100u)
    {
        wd_ms = now;
        uart_rx_watchdog();
    }

    /* Unsigned wrap-safe comparison; HAL_GetTick() rolls over after 49 days.
     * An explicit request jumps the queue so input feels immediate. */
    if (!ui_redraw_req && (uint32_t)(now - next_ms) < UI_PERIOD_MS)
        return;
    ui_redraw_req = 0;
    next_ms = now;

    t0 = DWT->CYCCNT;

    if (ui_mode == UI_MODE_MENU)
        ui_page_menu();
    else
        ui_page_drive();

    ui_render_us = (DWT->CYCCNT - t0) / CPU_MHZ;

    ui_flush();
}
