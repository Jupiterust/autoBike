/* Host-only integration tests for the actual upper.c and vofa.c.
 * Build/run commands are in tests/run_upper_tests.sh. No board required. */
#include "A_include.h"
#include "ch100.h"
#include <assert.h>
#include <stdio.h>

param_t param;
odrive_t odrive;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart8;
float M1_Ctl;
int Servo_Ctl, Servoduty;
bool restart_flag;
volatile float odrive_vel1_turns;
volatile uint32_t odrive_vel1_ms;
volatile uint8_t odrive_vel1_valid;
static uint32_t now_ms, mask;
static int last_steer;
static unsigned sends, ui_calls;
static uint8_t sent[12];
static uint16_t sent_size;
static const uint8_t *inflight;
static HAL_StatusTypeDef tx_result;

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{ assert(u == &huart8 && p == CH100_buf && n == CH100_DATA_LEN); return HAL_OK; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u)
{ assert(u == &huart8); return HAL_OK; }
void telem_note_imu_crc_err(void) {}
float low_pass_filter(float value) { return value; }

uint32_t HAL_GetTick(void) { return now_ms; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1; }
void __set_PRIMASK(uint32_t value) { mask = value; }
void ServoCtrl(int value) { Servoduty = value; }
int Steer_Speed_Limit(float value, float limit)
{
    if (value - last_steer > limit) value = last_steer + limit;
    if (value - last_steer < -limit) value = last_steer - limit;
    last_steer = (int)value;
    return last_steer;
}
int ui_get_mode(void) { return UI_MODE_DRIVE; }
void ui_request_redraw(void) {}
void ui_command(char id, const char *value)
{
    (void)value;
    ui_calls++;
    if (id == 'S') upper_toggle_mode();
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,
                                      const uint8_t *data, uint16_t n)
{
    assert(uart == &huart2);
    assert(n <= sizeof sent);
    if (tx_result != HAL_OK) return tx_result;
    assert(uart->gState == HAL_UART_STATE_READY);
    memcpy(sent, data, n);
    sent_size = n;
    inflight = data;
    uart->gState = 1;
    sends++;
    return HAL_OK;
}
static void tx_done(void)
{
    if (inflight) assert(memcmp(sent, inflight, sent_size) == 0);
    huart2.gState = HAL_UART_STATE_READY;
    inflight = NULL;
}
static int16_t read_i16(const uint8_t *p)
{
    uint16_t u = p[0] | ((uint16_t)p[1] << 8);
    return (int16_t)((u & 0x8000) ? (int32_t)u - 65536 : u);
}
static void feed(const uint8_t *data, unsigned n)
{
    while (n--) Upper(*data++);
}
static void command(int16_t angle, int16_t rpm)
{
    uint8_t f[8] = {0xAA, 0x55, (uint8_t)angle,
                   (uint8_t)((uint16_t)angle >> 8), (uint8_t)rpm,
                   (uint8_t)((uint16_t)rpm >> 8), 0, 0x5A};
    f[6] = upper_crc8(f, 6);
    feed(f, sizeof f);
}
static void remote(const char *frame)
{
    while (*frame) vofa_get((uint8_t)*frame++);
}
static void reset(void)
{
    tx_done();
    memset(&param, 0, sizeof param);
    memset(&odrive, 0, sizeof odrive);
    restart_flag = false;
    odrive_vel1_turns = 0;
    odrive_vel1_ms = 0;
    odrive_vel1_valid = 0;
    memset(&imu, 0, sizeof imu);
    ch100_attitude_ms = 0;
    CH100_USART_Init();
    now_ms = 0;
    sends = ui_calls = 0;
    tx_result = HAL_OK;
    upper_init();
    (void)upper_control_tick();
    assert(mask == 0);
}
static void host(void)
{
    upper_toggle_mode();
    (void)upper_control_tick();
    param.M0_Flag = 1;
}
static void tick(float assist)
{
    (void)upper_control_tick();
    Servo_Ctl = Steer_Speed_Limit(Servo_Ctl, 5);
    ServoCtrl(Servo_Center_Mid + Servo_Ctl);
    odrive.set_speed1 = param.M1_Flag ? upper_limit_drive(M1_Ctl + assist) : 0;
}
static uint8_t echo(void)
{
    tx_done();
    upper_task();
    assert(sent_size == 9 && sent[0] == 0xAA && sent[1] == 0x55);
    assert(sent[8] == 0x5A && sent[7] == upper_crc8(sent, 7));
    return sent[6];
}

static void test_crc_boot(void)
{
    reset();
    assert(upper_crc8((const uint8_t *)"123456789", 9) == 0xF4);
    assert(!upper_Flag && !param.M1_Flag && Servo_Ctl == 0 && M1_Ctl == 0);
    assert(echo() == 0);
    now_ms = 1001;
    tick(0);
    assert(echo() == 0); /* no host timeout in default handset mode */
}
static void test_owner_and_switch_mailbox(void)
{
    reset();
    param.M0_Flag = 1;
    command(100, 120);
    tick(0);
    assert(M1_Ctl == 0 && Servo_Ctl == 0);
    remote("(S,)"); remote("(H,)");
    vofa_apply();
    assert(upper_Flag && !param.M1_Flag);
    command(0, 120); tick(0);
    remote("(0,)"); vofa_apply();
    assert(param.angular_zero == 0); /* legacy tuning blocked in host mode */
    remote("(S,)"); remote("(S,)"); vofa_apply();
    assert(upper_Flag && M1_Ctl == 0 && !param.M1_Flag);
    remote("(S,)"); vofa_apply();
    assert(!upper_Flag);
    remote("(SS,)"); vofa_apply();
    remote("(S,1)"); vofa_apply();
    assert(!upper_Flag); /* malformed/non-four-byte switch is not accepted */
    host();
    remote("(4,)"); remote("(S,)"); vofa_apply();
    assert(!upper_Flag && servo_con == 0); /* no queued pre-switch steering */
}
static void test_absolute_slew_and_direction(void)
{
    reset(); host();
    for (unsigned i = 0; i < 100; i++) { command(-120, 120); tick(0); }
    assert(Servo_Ctl == 120 && M1_Ctl == 2.0f);
    assert(param.M1_Flag && (echo() & UPPER_MOTOR_EN));
    assert(read_i16(sent + 2) == -120 && read_i16(sent + 4) == 120);
    command(120, 120);
    for (unsigned i = 0; i < 60; i++) tick(0);
    assert(Servo_Ctl == -120); /* target survives the intermediate slew values */
    assert(!(echo() & (UPPER_MEASURED_RPM | UPPER_MEASURED_ANGLE)));
    /* RC is now limited to 85 units; host range/mapping MUST stay +/-120. */
    command(171, 120);
    for (unsigned i = 0; i < 80; i++) tick(0);
    assert(Servo_Ctl == -120);
    assert(echo() & UPPER_CLAMPED);
    assert(read_i16(sent + 2) == 120);
    command(-171, 120);
    for (unsigned i = 0; i < 80; i++) tick(0);
    assert(Servo_Ctl == 120);
    assert(echo() & UPPER_CLAMPED);
    assert(read_i16(sent + 2) == -120);
}
static void test_clamp_assist_and_stop(void)
{
    reset(); host();
    command(INT16_MAX, INT16_MAX);
    for (unsigned i = 0; i < 30; i++) tick(0.5f);
    assert(M1_Ctl == 4.0f && odrive.set_speed1 == 4.0f);
    assert(echo() & UPPER_CLAMPED);
    command(0, 120); tick(0.5f);
    assert(odrive.set_speed1 == 2.5f);
    assert(!(echo() & UPPER_CLAMPED));
    assert(read_i16(sent + 4) == 150); /* echo includes retained turn assist */
    command(0, 0);
    assert(upper_control_tick()); /* zero speed schedules the next rear tick */
    assert(!param.M1_Flag && M1_Ctl == 0);
}
static void test_invalid_and_nested_frames(void)
{
    const uint8_t bad_crc[] = {0xAA,0x55,0,0,0,0,0,0x5A};
    const uint8_t bad_tail[] = {0xAA,0x55,0,0,0,0,0xDD,0};
    const uint8_t nested[] = {0xAA,0x55,0xAA,0x57,0x38,0x5A,0,0};
    reset(); host();
    command(-120, 120); tick(0);
    feed(bad_crc, sizeof bad_crc);
    feed(bad_tail, sizeof bad_tail);
    command(0, -1);
    tick(0);
    assert(M1_Ctl == 2.0f && Servo_Ctl > 0);
    feed(nested, sizeof nested);
    upper_task();
    assert(sent_size == 12 && sent[1] == 0x56); /* skip 2, not whole bad frame */
    tx_done();
    /* A complete command begins inside the bad outer command's payload. */
    const uint8_t prefix[] = {0xAA,0x55};
    feed(prefix, sizeof prefix);
    command(0, 60); tick(0);
    assert(M1_Ctl == 1.0f);
}
static void test_query_bounds_and_freshness(void)
{
    const uint8_t query[] = {0xAA,0x57,0x38,0x5A};
    reset();
    feed(query, sizeof query); upper_task();
    assert(sent_size == 12 && sent[1] == 0x56);
    assert(read_i16(sent + 2) == -120 && read_i16(sent + 4) == 120);
    assert(read_i16(sent + 6) == 0 && read_i16(sent + 8) == 240);
    assert(sent[10] == upper_crc8(sent, 10) && sent[11] == 0x5A);
    tx_done(); host();
    command(0, 120); tick(0);
    now_ms = 100;
    feed(query, sizeof query);
    command(0, -1); /* even a CRC-valid negative RPM must not refresh time */
    tick(0);
    assert(param.M1_Flag); /* the specified comparison is strictly >100 ms */
    now_ms = 101;
    assert(upper_control_tick());
    assert(!param.M1_Flag && Servo_Ctl == 0 && odrive.set_speed1 == 0);
    upper_task(); tx_done(); /* queued query does not refresh the watchdog */
    assert(echo() == UPPER_TIMEOUT_SEEN);
    command(0, 60); tick(0);
    assert(echo() == (UPPER_LINK_OK | UPPER_TIMEOUT_SEEN | UPPER_MOTOR_EN));
}
static void test_balance_and_restart_gates(void)
{
    reset(); host();
    param.M0_Flag = 0; command(-120, 120); tick(0);
    assert(!param.M1_Flag && M1_Ctl == 0 && Servo_Ctl == 0);
    param.M0_Flag = 1; tick(0);
    assert(M1_Ctl == 0); /* no resurrection of a command received while off */
    command(-120, 120); tick(0);
    param.M0_Flag = 0; tick(0);
    assert(M1_Ctl == 0 && odrive.set_speed1 == 0);
    param.M0_Flag = 1; tick(0);
    assert(!param.M1_Flag);
    restart_flag = true; command(-120, 120); tick(0);
    restart_flag = false; tick(0);
    assert(M1_Ctl == 0 && Servo_Ctl == 0);
}
static void test_busy_period_and_wrap(void)
{
    const uint8_t query[] = {0xAA,0x57,0x38,0x5A};
    reset();
    upper_task();
    assert(sends == 1);
    feed(query, sizeof query); upper_task();
    assert(sends == 1); tx_done(); /* in-flight bytes must remain unchanged */
    tx_result = HAL_BUSY; upper_task();
    assert(sends == 1);
    tx_result = HAL_OK; upper_task();
    assert(sends == 2 && sent_size == 12); tx_done();
    now_ms = 99; upper_task(); assert(sends == 2);
    now_ms = 100; upper_task(); assert(sends == 3); tx_done();
    now_ms = UINT32_MAX - 20u;
    host(); command(0, 120); tick(0);
    now_ms += 100u; tick(0);
    assert(param.M1_Flag);
    now_ms++; tick(0);
    assert(!param.M1_Flag && (echo() & UPPER_TIMEOUT_SEEN));
}
static void test_noise_and_partial_reset(void)
{
    uint32_t rng = 0xABCDEF01u;
    const uint8_t flush[] = {0,0,0,0,0,0,0,0};
    const uint8_t partial[] = {0xAA,0x55,0,0};
    reset(); host();
    for (unsigned i = 0; i < 100000; i++) {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        Upper((uint8_t)rng);
    }
    feed(flush, sizeof flush);
    command(0, 120); tick(0);
    assert(M1_Ctl == 2.0f);
    feed(partial, sizeof partial);
    upper_rx_reset();
    command(0, 60); tick(0);
    assert(M1_Ctl == 1.0f);
}
static void test_measured_rpm_and_fallback(void)
{
    uint8_t status;
    reset(); host(); command(0, 120); tick(0.5f);
    assert(!(echo() & UPPER_MEASURED_RPM));
    assert(read_i16(sent + 4) == 150); /* never received: final command */

    odrive_vel1_turns = 1.25f;
    odrive_vel1_ms = now_ms;
    odrive_vel1_valid = 1;
    status = echo();
    assert(status == (UPPER_LINK_OK | UPPER_MOTOR_EN | UPPER_MEASURED_RPM));
    assert(read_i16(sent + 4) == 75); /* measured, not the 150 rpm command */

    command(0, 0); tick(0);
    status = echo();
    assert(!(status & UPPER_MOTOR_EN) && (status & UPPER_MEASURED_RPM));
    assert(read_i16(sent + 4) == 75); /* wheel still coasting after stop */
    odrive_vel1_turns = -0.5f;
    assert(echo() & UPPER_MEASURED_RPM);
    assert(read_i16(sent + 4) == -30); /* negative feedback is valid */
    odrive_vel1_turns = 5.0f;
    assert(echo() & UPPER_MEASURED_RPM);
    assert(read_i16(sent + 4) == 300); /* do not clamp measurement to target limit */

    now_ms = UPPER_RPM_FEEDBACK_MAX_AGE_MS;
    assert(echo() & UPPER_MEASURED_RPM); /* inclusive boundary */
    now_ms++;
    assert(!(echo() & UPPER_MEASURED_RPM));
    assert(read_i16(sent + 4) == 0);

    command(0, 120); tick(0.5f);
    odrive_vel1_ms = now_ms;
    odrive_vel1_turns = NAN;
    assert(!(echo() & UPPER_MEASURED_RPM));
    assert(read_i16(sent + 4) == 150);
    odrive_vel1_turns = INFINITY;
    assert(!(echo() & UPPER_MEASURED_RPM));
    odrive_vel1_turns = 1000.0f;
    assert(!(echo() & UPPER_MEASURED_RPM)); /* int16 would overflow */
    odrive_vel1_turns = 0.0f;
    assert(echo() & UPPER_MEASURED_RPM); /* measured zero is not missing data */
    assert(read_i16(sent + 4) == 0);
    odrive_vel1_valid = 0; /* UART RX restarted */
    assert(!(echo() & UPPER_MEASURED_RPM));
    assert(read_i16(sent + 4) == 150);

    odrive_vel1_valid = 1;
    odrive_vel1_ms = UINT32_MAX - 100u;
    now_ms = 50u;
    assert(echo() & UPPER_MEASURED_RPM); /* wrap-safe age = 151 ms */
    now_ms = 500u;
    assert(!(echo() & UPPER_MEASURED_RPM));
    assert(!(echo() & UPPER_MEASURED_ANGLE));
}

static void test_protection_stop_status(void)
{
    reset(); host(); command(30, 120); tick(0);
    odrive_vel1_turns = 1.0f;
    odrive_vel1_valid = 1;
    param.M0_Flag = 0; tick(0);
    assert(echo() & UPPER_TIMEOUT_SEEN);
    assert(!(echo() & UPPER_MOTOR_EN));
    assert(echo() & UPPER_MEASURED_RPM);
    assert(read_i16(sent + 4) == 60);
    param.M0_Flag = 1; command(0, 120); tick(0);
    assert(echo() & UPPER_TIMEOUT_SEEN); /* sticky after recovery */
}

static void test_240rpm_limits(void)
{
    reset(); host();
    command(0, 240); tick(0);
    assert(M1_Ctl == 4.0f && odrive.set_speed1 == 4.0f);
    assert(!(echo() & UPPER_CLAMPED));
    assert(read_i16(sent + 4) == 240);
    tick(0.5f);
    assert(odrive.set_speed1 == 4.0f && (echo() & UPPER_CLAMPED));
    command(0, 241); tick(0);
    assert(M1_Ctl == 4.0f && (echo() & UPPER_CLAMPED));
    assert(upper_limit_drive(-0.1f) == 0.0f); /* host cannot reverse */
    upper_toggle_mode();
    param.M0_Flag = param.M1_Flag = 1;
    assert(upper_limit_drive(4.5f) == 4.0f);
    assert(echo() & UPPER_CLAMPED);
    assert(upper_limit_drive(-4.5f) == -4.0f);
    assert(echo() & UPPER_CLAMPED);
    assert(upper_limit_drive(-4.0f) == -4.0f);
    assert(!(echo() & UPPER_CLAMPED));
}

/* Exercise the production CH100 CRC/parser, not just synthetic metadata. */
static void imu_sample(float pitch, float roll, float yaw)
{
    uint16_t crc = 0;
    memset(CH100_buf, 0, CH100_DATA_LEN);
    CH100_buf[0] = 0x5A; CH100_buf[1] = 0xA5;
    CH100_buf[2] = 76; CH100_buf[6] = 0x91;
    memcpy(CH100_buf + 54, &pitch, 4);
    memcpy(CH100_buf + 58, &roll, 4);
    memcpy(CH100_buf + 62, &yaw, 4);
    for (unsigned i = 0; i < CH100_DATA_LEN; i++) {
        if (i == 4 || i == 5) continue;
        crc ^= (uint16_t)CH100_buf[i] << 8;
        for (unsigned k = 0; k < 8; k++)
            crc = (uint16_t)((crc & 0x8000u) ? (crc << 1) ^ 0x1021u : crc << 1);
    }
    CH100_buf[4] = (uint8_t)crc; CH100_buf[5] = (uint8_t)(crc >> 8);
    assert(CH100_ProcAll(CH100_buf) == 1);
}
static const uint8_t imu_query[] = {0xAA, 0x57, 0x49, 0x5A};
static void attitude_reply(void)
{
    tx_done(); feed(imu_query, sizeof imu_query); upper_task();
    assert(sent_size == 11 && sent[0] == 0xAA && sent[1] == 0x58);
    assert(sent[9] == upper_crc8(sent, 9) && sent[10] == 0x5A);
    assert(!(sent[8] & ~UPPER_IMU_VALID));
    assert(mask == 0);
}
static void assert_invalid_attitude(void)
{
    attitude_reply();
    assert(sent[8] == 0);
    for (unsigned i = 2; i < 8; i++) assert(sent[i] == 0);
}
static void test_attitude_mapping_and_validity(void)
{
    const uint8_t example[] = {0xAA,0x58,0x2C,0x01,0x38,0xFF,0x8C,0x23,1,0x97,0x5A};
    reset(); assert_invalid_attitude(); /* boot: no IMU sample */
    imu_sample(3.0f, 2.0f, -91.0f); /* raw fields -> pitch +3, roll -2, yaw +91 */
    attitude_reply(); assert(!memcmp(sent, example, sizeof example));
    assert(imu.pit == 3 && imu.rol == 2 && imu.yaw == -91); /* no control mutation */
    now_ms = 100; attitude_reply(); assert(sent[8] == UPPER_IMU_VALID);
    now_ms = 101; assert_invalid_attitude();
    imu_sample(-1.125f, -1.125f, 179.5f); attitude_reply();
    assert(read_i16(sent + 2) == -113 && read_i16(sent + 4) == 113);
    assert(read_i16(sent + 6) == -17950);
    CH100_buf[54] ^= 1;
    assert(CH100_ProcAll(CH100_buf) == 0);
    assert_invalid_attitude(); /* a bad CRC invalidates even before timeout */
    imu_sample(NAN, 0, 0); assert_invalid_attitude();
    imu_sample(0, INFINITY, 0); assert_invalid_attitude();
    imu_sample(0, 0, -INFINITY); assert_invalid_attitude();
    imu_sample(400, 0, 0); assert_invalid_attitude(); /* no int16 wrap */
    imu_sample(0, 0, 0); attitude_reply(); assert(sent[8] == UPPER_IMU_VALID);
    CH100_USART_Init(); assert_invalid_attitude(); /* receive restart */
    now_ms = UINT32_MAX - 50u; imu_sample(1, 2, 3);
    now_ms = 49; attitude_reply(); assert(sent[8] == UPPER_IMU_VALID);
    now_ms = 50; assert_invalid_attitude(); /* wrap-safe 101 ms */
}
static void test_attitude_queue_and_parser(void)
{
    const uint8_t boundary[] = {0xAA,0x57,0x38,0x5A};
    const uint8_t bad[] = {0xAA,0x57,0x48,0x5A,0xAA,0x57,0x49,0};
    const uint8_t nested[] = {0xAA,0x55,0xAA,0x57,0x49,0x5A,0,0};
    unsigned before;
    reset(); imu_sample(3, 2, -91);
    upper_task(); /* ordinary echo in flight */
    feed(imu_query, 4); feed(imu_query, 4); feed(boundary, 4);
    before = sends; upper_task(); assert(sends == before); tx_done();
    tx_result = HAL_BUSY; upper_task(); assert(sends == before);
    tx_result = HAL_OK; upper_task(); assert(sent_size == 12); tx_done();
    tx_result = HAL_ERROR; upper_task(); assert(sends == before + 1);
    tx_result = HAL_OK; upper_task(); assert(sent_size == 11); tx_done();
    upper_task(); assert(sent_size == 11); tx_done();
    before = sends; upper_task(); assert(sends == before); /* no periodic IMU */
    feed(bad, sizeof bad); upper_task(); assert(sends == before);
    feed(nested, sizeof nested); upper_task();
    assert(sends == before + 1 && sent_size == 11); tx_done();
    feed(imu_query, 2); upper_rx_reset(); feed(imu_query + 2, 2);
    before = sends; upper_task(); assert(sends == before);
    host(); command(0, 120); tick(0);
    now_ms = 101; feed(imu_query, 4); tick(0);
    assert(!param.M1_Flag && M1_Ctl == 0); /* queries cannot keep drive alive */
    upper_task(); assert(sent_size == 11); tx_done();
    assert(echo() & UPPER_TIMEOUT_SEEN);
    /* Counter saturation must not wrap to zero. */
    reset();
    for (unsigned i = 0; i < 65536u; i++) feed(imu_query, 4);
    for (unsigned i = 0; i < 65535u; i++) {
        upper_task(); assert(sent_size == 11); tx_done();
    }
    assert(sends == 65535u);
    upper_task(); assert(sent_size == 9); /* exactly UINT16_MAX queued replies */
}

int main(void)
{
    test_crc_boot();
    test_owner_and_switch_mailbox();
    test_absolute_slew_and_direction();
    test_clamp_assist_and_stop();
    test_invalid_and_nested_frames();
    test_query_bounds_and_freshness();
    test_balance_and_restart_gates();
    test_busy_period_and_wrap();
    test_noise_and_partial_reset();
    test_measured_rpm_and_fallback();
    test_protection_stop_status();
    test_240rpm_limits();
    test_attitude_mapping_and_validity();
    test_attitude_queue_and_parser();
    puts("14 upper/remote/CH100 protocol test groups passed");
    return 0;
}
