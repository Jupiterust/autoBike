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
volatile uint32_t ch100_attitude_ms;
volatile uint8_t ch100_attitude_valid;
float M1_Ctl, Servo_Gain_K, error_zero, odrive_pos1_turns;
float Servo_zhongzhi_Gain;
volatile float odrive_vel1_turns;
volatile uint32_t odrive_vel1_ms;
volatile uint8_t odrive_vel1_valid;
int Servo_Ctl, Servoduty;
bool restart_flag;
uint32_t uart_err_cnt[3], odrive_tx_drops;
uint32_t odrive_fb_tx, odrive_fb_ok, odrive_fb_bad, odrive_fb_eol, odrive_fb_ovf;
static uint32_t now_ms = 100, mask;
static int last_steer;
static char rows[DISP_ROWS][DISP_COLS + 1];
static uint16_t colors[DISP_ROWS];

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
    assert(uart == &huart2 && (n == 9 || n == 11 || n == 12));
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
void disp_row_color(uint8_t row, const char *text, uint16_t color)
{
    disp_row(row, text);
    if (row < DISP_ROWS) colors[row] = color;
}
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
static void drive_zero(const char *value)
{
    if (DISP_ROWS >= 10) {
        contains(8, "zero");
        contains(9, value);
        details_cleared(10);
    } else if (DISP_ROWS > 7) {
        contains(7, value);
        details_cleared(8);
    }
}

int main(void)
{
    upper_init(); ui_init();
    contains(1, "starting up");
    imu.rol = -2.0f; imu.pit = 3.0f; imu.yaw = 91.0f;
    param.angular_zero = -1.4f;
    ui_render_now("Press KEY0 to start", -1);
    contains(1, "Press KEY0 to start");
    contains(2, "ROLL     -2.00 deg");
    contains(3, "PITCH    +3.00 deg");
    contains(4, "YAW     +91.00 deg");
    assert(colors[2] == DISP_GREEN && colors[3] == DISP_YELLOW &&
           colors[4] == DISP_CYAN && mask == 0);
    assert(!param.M0_Flag && !param.M1_Flag);
    now_ms += 60;
    ui_render_now("re-stand", 2);
    contains(1, "re-stand 2s"); contains(4, "YAW");
    details_cleared(DISP_UI_ROWS);
    render(); contains(1, "RC M0:0"); contains(2, "ROLL");
    drive_zero("Zservo   -1.400 deg");
    Servo_zhongzhi_Gain = 0.25f;
    render();
    drive_zero("Zservo   -1.150 deg");
    assert(param.angular_zero == -1.4f && Servo_zhongzhi_Gain == 0.25f);
    remote("(U,)"); assert(M1_Ctl == 0 && !param.M1_Flag);
    param.M0_Flag = 1;
    remote("(U,)"); assert(fabsf(M1_Ctl - 0.1f) < 0.001f);
    /* RC actual-angle estimate stays within 15 deg; shared host cap is 120. */
    assert(Servo_Delta == 120 && SERVO_RC_LIMIT_UNITS == 85);
    assert(SERVO_RC_LIMIT_UNITS * SERVO_DEG_PER_UNIT <= 15.0f);
    for (unsigned i = 0; i < 40; i++) remote("(L,)");
    assert(Servo_Ctl == 85);
    if (DISP_ROWS >= 10) contains(6, "SERVO~ -14.9deg");
    for (unsigned i = 0; i < 80; i++) remote("(R,)");
    assert(Servo_Ctl == -85);
    if (DISP_ROWS >= 10) contains(6, "SERVO~ +14.9deg");
    remote("(K,)");
    assert(servo_con == 1 && !param.M1_Flag && M1_Ctl == 0);
    for (unsigned i = 0; i < 180; i++) {
        now_ms += 10; vofa_con();
        if (i % 10 == 0) remote("(H,)");
    }
    render(); assert(Servo_Ctl == 0 && servo_con == 0);
    if (DISP_ROWS >= 10) contains(6, "SERVO~  +0.0deg");
    remote("(M,)");
    assert(ui_get_mode() == UI_MODE_MENU);
    contains(1, ">av_kp");
    if (DISP_ROWS >= 8) {
        contains(5, "ROLL"); contains(6, "PITCH"); contains(7, "YAW");
        assert(colors[5] == DISP_GREEN && colors[6] == DISP_YELLOW &&
               colors[7] == DISP_CYAN);
        if (DISP_ROWS > 8) contains(8, "Zservo   -1.150 deg");
        details_cleared(9);
    }
    remote("(S,)"); contains(1, "HOST M0:1");
    assert(ui_get_mode() == UI_MODE_DRIVE);
    command(60, 240);
    for (unsigned i = 0; i < 30; i++) tick(0);
    render(); assert(M1_Ctl == 4.0f && odrive.set_speed1 == 4.0f);
    contains(2, "ROLL"); contains(3, "PITCH"); contains(4, "YAW");
    drive_zero("Zservo   -1.150 deg");
    if (DISP_ROWS >= 10) {
        contains(5, "SPD S+4.00 A");
        contains(6, "SERVO~"); contains(7, "FLY");
        assert(colors[5] == DISP_ORANGE && colors[6] == DISP_CYAN &&
               colors[7] == DISP_MAGENTA);
    }
    remote("(S,)"); contains(1, "RC M0:1");
    assert(!upper_Flag && M1_Ctl == 0 && !param.M1_Flag);
    imu.rol = 12.5f; imu.pit = -8.25f; imu.yaw = -179.5f;
    render(); contains(2, "+12.50"); contains(3, "-8.25"); contains(4, "-179.50");
    Servo_zhongzhi_Gain = -0.25f;
    render();
    drive_zero("Zservo   -1.650 deg");
    param.angular_zero = -1.0f;
    render();
    drive_zero("Zservo   -1.250 deg");
    assert(Servo_zhongzhi_Gain == -0.25f);
    M1_Ctl = 2.5f; odrive_vel1_turns = -1.25f; odrive.now_speed0 = -123.5f;
    render();
    if (DISP_ROWS >= 10) {
        contains(5, "SPD S+2.50 A-1.25");
        contains(6, "SERVO~");
        contains(7, "FLY  -123.5 enc/T");
        assert(colors[5] == DISP_ORANGE && colors[6] == DISP_CYAN &&
               colors[7] == DISP_MAGENTA);
    }
    assert(M1_Ctl == 2.5f && odrive_vel1_turns == -1.25f && odrive.now_speed0 == -123.5f);
    remote("(M,)"); /* clear drive's last row when entering the menu */
    if (DISP_ROWS >= 10) details_cleared(9);
    /* Six-key handset: each left/right frame changes zero by 0.005 degrees. */
    for (unsigned i = 0; i < 9; i++) remote("(D,)");
    contains(2, ">zero");
#if !UI_SHOW_PERF
    contains(4, "step 0.0050");
#endif
    param.angular_zero = -1.4f;
    remote("(R,)");
    assert(fabsf(param.angular_zero - (-1.395f)) < 0.000001f);
    contains(2, "-1.3950");
    remote("(L,)");
    assert(fabsf(param.angular_zero - (-1.4f)) < 0.000001f);
    for (unsigned i = 0; i < 10; i++) remote("(R,)");
    assert(fabsf(param.angular_zero - (-1.35f)) < 0.000002f);
    param.angular_zero = 4.998f;
    remote("(R,)"); assert(param.angular_zero == 5.0f);
    param.angular_zero = -4.998f;
    remote("(L,)"); assert(param.angular_zero == -5.0f);
    printf("UI integration passed: %u rows, PERF=%u\n",
           (unsigned)DISP_ROWS, (unsigned)UI_SHOW_PERF);
    return 0;
}

