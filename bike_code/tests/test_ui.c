/* Real UI, protocol and handset parser; hardware and display transport stubbed. */
#include "A_include.h"
#include "display.h"
#include <assert.h>
#include <stdio.h>

param_t param;
odrive_t odrive;
UART_HandleTypeDef huart2;
test_core_t test_core;
test_dwt_t test_dwt;
test_imu_t imu;
float M1_Ctl, Servo_Gain_K, error_zero, odrive_vel1_turns, odrive_pos1_turns;
int Servo_Ctl, Servoduty;
bool restart_flag;
uint32_t uart_err_cnt[3], odrive_tx_drops;
uint32_t odrive_fb_tx, odrive_fb_ok, odrive_fb_bad, odrive_fb_eol, odrive_fb_ovf;
static uint32_t now_ms = 100, mask;
static int last_steer;
static char rows[DISP_ROWS][DISP_COLS + 1];

uint32_t HAL_GetTick(void) { return now_ms; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1; }
void __set_PRIMASK(uint32_t value) { mask = value; }
void ServoCtrl(int value) { Servoduty = value; }
int Steer_Speed_Limit(float value, float limit)
{
    if (value - last_steer > limit) value = last_steer + limit;
    if (value - last_steer < -limit) value = last_steer - limit;
    return last_steer = (int)value;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,
                                      const uint8_t *data, uint16_t n)
{
    assert(uart == &huart2 && (n == 9 || n == 12));
    assert(data[n - 1] == 0x5A);
    return HAL_OK;
}
void led_task(void) {}
void uart_rx_watchdog(void) {}
uint8_t test_pulse_start(void) { return 0; }
uint8_t test_pulse_active(void) { return 0; }
float odrive_odom_m(void) { return 0; }
void disp_init(void) { memset(rows, 0, sizeof rows); }
void disp_clear_all(void) { memset(rows, 0, sizeof rows); }
void disp_invalidate(void) {}
void disp_row(uint8_t row, const char *text)
{
    assert(strlen(text) <= DISP_COLS);
    if (row < DISP_ROWS) snprintf(rows[row], sizeof rows[row], "%s", text);
}
void disp_control_heading(uint8_t host)
{ disp_row(1, host ? "HOST CONTROL" : "REMOTE CTRL"); }
uint8_t disp_flush(void) { return 0; }
void disp_lamp_test(void) {}
void disp_grid_test(void) {}

static void render(void) { ui_request_redraw(); ui_task(); assert(mask == 0); }
static void remote(const char *frame)
{
    while (*frame) vofa_get((uint8_t)*frame++);
    vofa_apply(); render();
}
static void command(int16_t angle, int16_t rpm)
{
    uint8_t f[8] = {0xAA, 0x55, (uint8_t)angle,
                   (uint8_t)((uint16_t)angle >> 8), (uint8_t)rpm,
                   (uint8_t)((uint16_t)rpm >> 8), 0, 0x5A};
    f[6] = upper_crc8(f, 6);
    for (unsigned i = 0; i < sizeof f; i++) Upper(f[i]);
}
static void tick(float assist)
{
    (void)upper_control_tick();
    Servo_Ctl = Steer_Speed_Limit(Servo_Ctl, 5);
    odrive.set_speed1 = param.M1_Flag ? upper_limit_drive(M1_Ctl + assist) : 0;
}
static void contains(unsigned row, const char *text)
{
    assert(row < DISP_ROWS);
    if (!strstr(rows[row], text)) {
        fprintf(stderr, "row %u: [%s], expected [%s]\n", row, rows[row], text);
        assert(0);
    }
}
static void details_cleared(unsigned first)
{
    for (unsigned row = first; row < DISP_ROWS; row++)
        assert(rows[row][0] == ' ' || rows[row][0] == '\0');
}
int main(void)
{
    upper_init(); ui_init();
    contains(4, "CTRL:REMOTE");
    render(); contains(1, "REMOTE CTRL"); contains(2, "RC:WAIT");
    remote("(S,)");
    contains(1, "HOST CONTROL"); contains(2, "LINK:WAIT");
    contains(4, "BALANCE OFF - STOP");
    param.M0_Flag = 1;
    render(); contains(4, "WAIT HOST COMMAND");

    now_ms = 201; tick(0); render();
    contains(2, "LINK:LOST"); contains(4, "TIMEOUT - SAFE STOP");
    command(0, 0); tick(0); render();
    contains(2, "LINK:OK"); contains(4, "STOPPED");
    if (DISP_ROWS > 6) contains(6, "TMO:SEEN");

    command(60, 60);
    for (unsigned i = 0; i < 15; i++) tick(0.5f);
    render(); contains(3, "RPM: 90 ANG:R 6.0"); contains(4, "RUNNING");
    remote("(M,)"); remote("(L,)"); remote("(0,)");
    assert(upper_Flag && ui_get_mode() == UI_MODE_DRIVE);
    assert(Servo_Ctl == -32 && M1_Ctl == 1.0f);

    now_ms = 301; render(); contains(2, "LINK:OK");
    now_ms = 302;
    /* Drawing neither refreshes freshness nor mutates control. */
    for (unsigned i = 0; i < 20; i++) render();
    contains(4, "COMMAND STALE"); assert(M1_Ctl == 1.0f);
    tick(0); render();
    contains(4, "TIMEOUT - SAFE STOP");
    contains(3, "RPM:  0 ANG:C 0.0"); contains(2, "RUN:0");

    command(-60, 60);
    for (unsigned i = 0; i < 15; i++) tick(0);
    render(); contains(3, "RPM: 60 ANG:L 6.0");
    contains(4, "RUNNING"); /* Sticky timeout history is not an active fault. */
    if (DISP_ROWS > 6) contains(6, "TMO:SEEN");
    command(INT16_MAX, INT16_MAX);
    for (unsigned i = 0; i < 30; i++) tick(0.5f);
    render(); contains(3, "RPM:180 ANG:R12.0");
    contains(4, "RUNNING - LIMITED");
    if (DISP_ROWS > 6) contains(6, "LIMIT:ON");

    param.M0_Flag = 0; tick(0); render();
    contains(4, "BALANCE OFF - STOP"); contains(2, "RUN:0");
    param.M0_Flag = 1; upper_stop_control(); restart_flag = true;
    render(); contains(4, "RE-STAND - STOP");
    restart_flag = false; render(); contains(4, "WAIT HOST COMMAND");
    now_ms += 60;
    ui_render_now("re-stand", 2);
    contains(1, "re-stand"); contains(4, "CTRL:HOST");
    details_cleared(DISP_UI_ROWS);

    remote("(S,)"); contains(1, "REMOTE CTRL");
    assert(!upper_Flag && M1_Ctl == 0 && !param.M1_Flag);
    remote("(M,)");
    assert(ui_get_mode() == UI_MODE_MENU);
    contains(1, ">av_kp"); details_cleared(7);
    remote("(S,)"); contains(1, "HOST CONTROL");
    assert(ui_get_mode() == UI_MODE_DRIVE);
    remote("(S,)"); contains(1, "REMOTE CTRL");
    printf("UI integration passed: %u rows, PERF=%u\n",
           (unsigned)DISP_ROWS, (unsigned)UI_SHOW_PERF);
    return 0;
}

