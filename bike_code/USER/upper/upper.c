#include "A_include.h"
#include "upper.h"

extern bool restart_flag; /* Init.c; the legacy Init.h does not declare it */

volatile uint8_t upper_Flag = 0;

static uint8_t rx[8];
static uint8_t rx_size;
static volatile int16_t target_servo, target_rpm;
static volatile uint32_t command_ms, armed_ms;
static volatile uint8_t command_seen, timed_out, timeout_seen;
static volatile uint8_t command_clamped, output_clamped, stop_pending;
static volatile uint16_t boundary_pending;
static volatile uint16_t attitude_pending;

typedef struct {
    int16_t servo, rpm;
    uint8_t status;
} echo_t;

/* USART2 IT reads this buffer until gState returns to READY. */
static uint8_t tx[12];
static echo_t last_echo;
static uint8_t echo_sent;
static uint32_t echo_ms;

uint8_t upper_crc8(const uint8_t *data, uint16_t size)
{
    uint8_t crc = 0;
    uint16_t i;
    uint8_t bit;
    for (i = 0; i < size; i++) {
        crc ^= data[i];
        for (bit = 0; bit < 8; bit++)
            crc = (uint8_t)((crc & 0x80u) ? (crc << 1) ^ 0x07u : crc << 1);
    }
    return crc;
}

static int16_t get_i16(const uint8_t *p)
{
    uint16_t u = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    return (int16_t)((u & 0x8000u) ? (int32_t)u - 65536 : (int32_t)u);
}

static void put_i16(uint8_t *p, int16_t value)
{
    uint16_t u = (uint16_t)value;
    p[0] = (uint8_t)u;
    p[1] = (uint8_t)(u >> 8);
}

/* Right-positive shaft centidegrees -> left-positive PWM units.
 * Round the magnitude so positive/negative commands are symmetric. */
static int cdeg_to_units(int16_t angle)
{
    int32_t mag = angle < 0 ? -(int32_t)angle : angle;
    int units = (int)((mag * UPPER_STEER_REFERENCE_UNITS
                   + UPPER_STEER_REFERENCE_CDEG / 2)
                  / UPPER_STEER_REFERENCE_CDEG);
    if (units > Servo_Delta) units = Servo_Delta;
    return angle > 0 ? -units : units;
}

/* Applied PWM output -> estimated shaft centidegrees, half away from zero.
 * Use the post-slew output, not the requested angle; no position sensor. */
static int16_t units_to_cdeg(int units)
{
    int32_t mag = units < 0 ? -(int32_t)units : units;
    int16_t angle = (int16_t)((mag * UPPER_STEER_REFERENCE_CDEG
                            + UPPER_STEER_REFERENCE_UNITS / 2)
                           / UPPER_STEER_REFERENCE_UNITS);
    return units > 0 ? (int16_t)-angle : angle;
}

/* Called with interrupts masked, or from the highest-priority balance ISR.
 * Reset the existing limiter's history as well as the public variable:
 * otherwise its previous non-zero value could steer again after a stop. */
static void clear_drive(void)
{
    target_servo = target_rpm = 0;
    command_clamped = output_clamped = 0;
    M1_Ctl = 0.0f;
    param.M1_Flag = 0;
    odrive.set_speed1 = 0.0f;
    Servo_Ctl = Steer_Speed_Limit(0, 2 * Servo_Delta);
    Servoduty = Servo_Center_Mid;
    servo_con = 0;
    ServoCtrl(Servo_Center_Mid);
    stop_pending = 1;
}

void upper_stop_control(void)
{
    uint32_t pri = __get_PRIMASK();
    __disable_irq();
    command_seen = timed_out = 0;
    armed_ms = HAL_GetTick();
    clear_drive();
    __set_PRIMASK(pri);
}

void upper_init(void)
{
    upper_Flag = 0;
    upper_rx_reset();
    command_ms = timeout_seen = boundary_pending = 0;
    attitude_pending = 0;
    echo_sent = 0;
    echo_ms = 0;
    upper_stop_control();
}

void upper_toggle_mode(void)
{
    uint32_t pri = __get_PRIMASK();
    __disable_irq();
    upper_Flag ^= 1u;
    upper_rx_reset(); /* a frame started before the switch cannot take over */
    upper_stop_control();
    __set_PRIMASK(pri);
}

void upper_rx_reset(void)
{
    rx_size = 0;
}

static void accept_command(int16_t servo, int16_t rpm)
{
    uint8_t clamped = 0;
    uint32_t pri;

    /* Handset mode ignores host driving commands, including their freshness.
     * Queries are still answered. Switching always requires a NEW command. */
    if (!upper_Flag) return;
    if (servo < -UPPER_STEER_LIMIT_CDEG) {
        servo = -UPPER_STEER_LIMIT_CDEG;
        clamped = 1;
    } else if (servo > UPPER_STEER_LIMIT_CDEG) {
        servo = UPPER_STEER_LIMIT_CDEG;
        clamped = 1;
    }
    if (rpm > UPPER_RPM_MAX) {
        rpm = UPPER_RPM_MAX;
        clamped = 1;
    }

    pri = __get_PRIMASK();
    __disable_irq();
    /* Do not retain a moving target through a balance fault/re-stand. */
    if (!param.M0_Flag || restart_flag) servo = rpm = 0;
    if (target_rpm != 0 && rpm == 0) stop_pending = 1;
    target_servo = servo;
    target_rpm = rpm;
    command_clamped = clamped;
    command_ms = HAL_GetTick();
    command_seen = 1;
    timed_out = 0;
    __set_PRIMASK(pri);
}

static void rx_drop(uint8_t n)
{
    rx_size = (uint8_t)(rx_size - n);
    memmove(rx, rx + n, rx_size);
}

void  Upper(uint8_t byte)
{
    uint8_t size;
    int16_t rpm;
    rx[rx_size++] = byte;
    while (rx_size) {
        if (rx[0] != 0xAAu) { rx_drop(1); continue; }
        if (rx_size < 2) return;
        if (rx[1] == 0x55u) size = 8;
        else if (rx[1] == 0x57u) size = 4;
        else { rx_drop(1); continue; }
        if (rx_size < size) return;
        /* Legacy boundary query uses CRC=0x38; the new query uses item=0x49
         * instead of CRC. Never relax CRC validation for driving commands. */
        if (rx[size - 1] != 0x5Au
         || (size == 8 ? upper_crc8(rx, 6) != rx[6]
                       : (rx[2] != 0x38u && rx[2] != 0x49u))) {
            rx_drop(2); /* preserve all payload bytes for resynchronisation */
            continue;
        }
        if (size == 8) {
            rpm = get_i16(rx + 4);
            if (rpm < 0) { rx_drop(2); continue; }
            accept_command(get_i16(rx + 2), rpm);
        } else if (rx[2] == 0x49u) {
            if (attitude_pending != UINT16_MAX) attitude_pending++;
        } else {
            if (boundary_pending != UINT16_MAX) boundary_pending++;
        }
        rx_drop(size);
    }
}

uint8_t upper_control_tick(void)
{
    uint8_t stop;
    uint32_t now = HAL_GetTick();
    if (upper_Flag) {
        uint32_t since = command_seen ? command_ms : armed_ms;
        if (!timed_out && (uint32_t)(now - since) > UPPER_TIMEOUT_MS) {
            timed_out = timeout_seen = 1;
            clear_drive();
        }
        if (!param.M0_Flag || restart_flag) {
            if (target_servo || target_rpm || M1_Ctl != 0.0f
             || odrive.set_speed1 != 0.0f || Servo_Ctl || param.M1_Flag) {
                timeout_seen = 1; /* sticky: protection also entered safe state */
                clear_drive();
            }
        } else if (command_seen && !timed_out) {
            /* Reapply the absolute target every tick: balance() overwrites
             * Servo_Ctl with the intermediate slew-limited position. */
            Servo_Ctl = cdeg_to_units(target_servo);
            M1_Ctl = (float)target_rpm / 60.0f;
            param.M1_Flag = target_rpm > 0;
        }
    }
    stop = stop_pending;
    stop_pending = 0;
    return stop;
}

float upper_limit_drive(float speed)
{
    const float limit = (float)UPPER_RPM_MAX / 60.0f;
    const float minimum = upper_Flag ? 0.0f : -limit;
    output_clamped = 0;
    /* Limit AFTER turn assistance in both modes; handset permits reverse. */
    if (speed > limit) {
        output_clamped = 1;
        return limit;
    }
    if (speed < minimum) {
        output_clamped = 1;
        return minimum;
    }
    return speed;
}

static echo_t current_echo(void)
{
    echo_t result;
    int units;
    float speed, feedback, rpm;
    uint32_t feedback_ms, now, last, pri = __get_PRIMASK();
    uint8_t feedback_valid, seen, enabled;

    __disable_irq();
    now = HAL_GetTick();
    last = command_ms;
    seen = command_seen && upper_Flag;
    enabled = param.M0_Flag && !restart_flag && param.M1_Flag;
    units = (param.M0_Flag && !restart_flag) ? Servo_Ctl : 0;
    speed = enabled ? odrive.set_speed1 : 0.0f;
    feedback = odrive_vel1_turns;
    feedback_ms = odrive_vel1_ms;
    feedback_valid = odrive_vel1_valid;
    result.status = (timeout_seen ? UPPER_TIMEOUT_SEEN : 0)
                  | (enabled ? UPPER_MOTOR_EN : 0)
                  | ((command_clamped || output_clamped) ? UPPER_CLAMPED : 0);
    __set_PRIMASK(pri);
    if (seen && (uint32_t)(now - last) <= UPPER_TIMEOUT_MS)
        result.status |= UPPER_LINK_OK;
    result.servo = units_to_cdeg(units);
    rpm = feedback * 60.0f;
    if (feedback_valid
     && (uint32_t)(now - feedback_ms) <= UPPER_RPM_FEEDBACK_MAX_AGE_MS
     && isfinite(rpm) && rpm >= -32768.0f && rpm <= 32767.0f) {
        /* Keep measured coasting/reverse/overspeed even with MOTOR_EN=0.
         * Command limits are NOT limits on what the encoder can measure. */
        result.status |= UPPER_MEASURED_RPM;
    } else {
        rpm = speed * 60.0f; /* absent/stale/invalid feedback: command fallback */
    }
    result.rpm = (int16_t)(rpm + (rpm < 0.0f ? -0.5f : 0.5f));
    /* No steering sensor: MEASURED_ANGLE remains 0 in every frame. */
    return result;
}

/* Foreground only. Copy a coherent ISR-published sample under a short mask;
 * float conversion and UART transmission run with the prior IRQ state restored.
 * Invalid samples reply with all angles zero and VALID=0 (not a zero pose). */
static void pack_attitude(void)
{
    float pitch, roll, yaw;
    uint32_t sample_ms, now, pri = __get_PRIMASK();
    uint8_t valid;
    __disable_irq();
    pitch = imu.pit;
    roll = -imu.rol;
    yaw = -imu.yaw;
    sample_ms = ch100_attitude_ms;
    valid = ch100_attitude_valid;
    now = HAL_GetTick();
    __set_PRIMASK(pri);

    /* roundf handles negative halves symmetrically. Check the rounded floats
     * BEFORE casting: NaN, infinity and int16 overflow must never reach a cast. */
    pitch = roundf(pitch * 100.0f);
    roll = roundf(roll * 100.0f);
    yaw = roundf(yaw * 100.0f);
    valid = valid && (uint32_t)(now - sample_ms) <= UPPER_IMU_MAX_AGE_MS
          && isfinite(pitch) && pitch >= -32768.0f && pitch <= 32767.0f
          && isfinite(roll) && roll >= -32768.0f && roll <= 32767.0f
          && isfinite(yaw) && yaw >= -32768.0f && yaw <= 32767.0f;
    tx[0] = 0xAA; tx[1] = 0x58;
    put_i16(tx + 2, valid ? (int16_t)pitch : 0);
    put_i16(tx + 4, valid ? (int16_t)roll : 0);
    put_i16(tx + 6, valid ? (int16_t)yaw : 0);
    tx[8] = valid ? UPPER_IMU_VALID : 0; /* CALIBRATED stays 0 */
    tx[9] = upper_crc8(tx, 9);
    tx[10] = 0x5A;
}

void upper_task(void)
{
    echo_t echo;
    uint32_t now, pri;
    if (huart2.gState != HAL_UART_STATE_READY) return;

    if (boundary_pending) {
        tx[0] = 0xAA; tx[1] = 0x56;
        put_i16(tx + 2, -UPPER_STEER_LIMIT_CDEG);
        put_i16(tx + 4, UPPER_STEER_LIMIT_CDEG);
        put_i16(tx + 6, 0); /* minimum stable speed not calibrated */
        put_i16(tx + 8, UPPER_RPM_MAX);
        tx[10] = upper_crc8(tx, 10);
        tx[11] = 0x5A;
        if (HAL_UART_Transmit_IT(&huart2, tx, 12) == HAL_OK) {
            pri = __get_PRIMASK();
            __disable_irq();
            boundary_pending--;
            __set_PRIMASK(pri);
        }
        return;
    }

    /* Boundary replies have priority. Retain pending requests on HAL_BUSY or
     * HAL_ERROR; never overwrite the shared buffer while USART2 is in flight. */
    if (attitude_pending) {
        pack_attitude();
        if (HAL_UART_Transmit_IT(&huart2, tx, 11) == HAL_OK) {
            pri = __get_PRIMASK();
            __disable_irq();
            attitude_pending--;
            __set_PRIMASK(pri);
        }
        return;
    }

    now = HAL_GetTick();
    echo = current_echo();
    if (echo_sent && echo.servo == last_echo.servo
     && echo.rpm == last_echo.rpm && echo.status == last_echo.status
     && (uint32_t)(now - echo_ms) < UPPER_ECHO_PERIOD_MS) return;
    tx[0] = 0xAA; tx[1] = 0x55;
    put_i16(tx + 2, echo.servo);
    put_i16(tx + 4, echo.rpm);
    tx[6] = echo.status;
    tx[7] = upper_crc8(tx, 7);
    tx[8] = 0x5A;
    if (HAL_UART_Transmit_IT(&huart2, tx, 9) == HAL_OK) {
        last_echo = echo;
        echo_ms = now;
        echo_sent = 1;
    }
}
