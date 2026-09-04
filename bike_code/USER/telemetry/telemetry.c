#include "telemetry.h"

#if TELEM_ENABLE

/* Globals owned by Balane.c that no header declares. Read-only here. */
extern float Fly_Gain;        /* flywheel zero-point compensation, deg */
extern float Servo_Lim;       /* current allowed steering slew rate    */
extern bool  restart_flag;

volatile uint32_t telem_tx_drop = 0;

static uint32_t loop_cnt      = 0;   /* incremented every balance() call */
static uint16_t imu_crc_err   = 0;   /* cumulative CH100 CRC failures    */
static uint16_t imu_crc_err_p = 0;   /* value at previous frame          */

/* Values latched by the taps in PID.c. The angle loop and the flywheel
   speed loop run every 6th / 60th control cycle, so between their updates
   these correctly hold the setpoint the inner loop is still acting on. */
static float tap_angle_target = 0.0f;
static float tap_pwm_x        = 0.0f;
static float tap_set0_raw     = 0.0f;
static float tap_inner_p      = 0.0f;
static float tap_inner_i      = 0.0f;
static float tap_inner_d      = 0.0f;
static float tap_pwm_accel    = 0.0f;

static uint8_t frame[TELEM_FRAME_LEN];

/* Fails to compile if the field list below stops summing to 73 bytes. */
typedef char telem_payload_size_check[
    ((1 + 4 + 4 + 15 * 4 + 2 + 2) == (int)TELEM_PAYLOAD_LEN) ? 1 : -1];


/* ---- CRC16-CCITT (0x1021, init 0xFFFF), same as the bridge ---- */
uint16_t telem_crc16_ccitt(const uint8_t *d, uint32_t n)
{
    uint16_t crc = 0xFFFF;
    uint32_t i;
    int k;

    for (i = 0; i < n; i++)
    {
        crc ^= (uint16_t)d[i] << 8;
        for (k = 0; k < 8; k++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}


/* ---- taps (called from PID.c, latch only) ---- */
void telem_tap_angle_target(float angle_target)
{
    tap_angle_target = angle_target;
}

void telem_tap_inner(float p, float i, float d, float pwm_x, float raw_out)
{
    tap_inner_p  = p;
    tap_inner_i  = i;
    tap_inner_d  = d;
    tap_pwm_x    = pwm_x;
    tap_set0_raw = raw_out;
}

void telem_tap_pwm_accel(float pwm_accel)
{
    tap_pwm_accel = pwm_accel;
}

void telem_note_imu_crc_err(void)
{
    imu_crc_err++;
}


/* ---- little endian writers (no struct packing, no alignment traps) ---- */
static uint16_t put_u8(uint8_t *b, uint16_t o, uint8_t v)
{
    b[o] = v;
    return (uint16_t)(o + 1);
}

static uint16_t put_u16(uint8_t *b, uint16_t o, uint16_t v)
{
    b[o]     = (uint8_t)(v & 0xFF);
    b[o + 1] = (uint8_t)(v >> 8);
    return (uint16_t)(o + 2);
}

static uint16_t put_u32(uint8_t *b, uint16_t o, uint32_t v)
{
    b[o]     = (uint8_t)(v & 0xFF);
    b[o + 1] = (uint8_t)((v >> 8) & 0xFF);
    b[o + 2] = (uint8_t)((v >> 16) & 0xFF);
    b[o + 3] = (uint8_t)((v >> 24) & 0xFF);
    return (uint16_t)(o + 4);
}

static uint16_t put_f32(uint8_t *b, uint16_t o, float v)
{
    uint32_t u;
    memcpy(&u, &v, 4);          /* no aliasing games */
    return put_u32(b, o, u);
}


/* ---- payload, in SCHEMA order ---- */
static uint16_t telem_pack(uint8_t *p)
{
    uint16_t o     = 0;
    uint16_t flags = 0;
    float    set0  = odrive.set_speed0;   /* after clamp and after the
                                             over-angle cutout */

    if (param.M0_Flag) flags |= TELEM_FLAG_M0;
    if (param.M1_Flag) flags |= TELEM_FLAG_M1;
    if (restart_flag)  flags |= TELEM_FLAG_RESTART;
    if (upper_Flag)    flags |= TELEM_FLAG_UPPER;

    /* Saturation, not fault: when M0_Flag is clear the cutout has already
       forced set0 to 0, which is not the rate limiter acting. Guarding on
       M0_Flag also means no magic number has to be duplicated from
       fly_wheel_rate_limit in Balane.c -- raw != clamped is exact. */
    if (param.M0_Flag && (tap_set0_raw != set0)) flags |= TELEM_FLAG_SAT;

    if (imu_crc_err != imu_crc_err_p) flags |= TELEM_FLAG_IMU_CRC;
    imu_crc_err_p = imu_crc_err;

    o = put_u8 (p, o, (uint8_t)TELEM_PROTO_VERSION);   /*  0 ver             */
    o = put_u32(p, o, loop_cnt);                       /*  1 loop_cnt        */
    o = put_u32(p, o, HAL_GetTick());                  /*  5 t_ms            */
    o = put_f32(p, o, imu.rol);                        /*  9 imu_rol         */
    o = put_f32(p, o, tap_angle_target);               /* 13 angle_target    */
    o = put_f32(p, o, imu.vx);                         /* 17 imu_vx          */
    o = put_f32(p, o, tap_pwm_x);                      /* 21 pwm_x           */
    o = put_f32(p, o, tap_set0_raw);                   /* 25 set_speed0_raw  */
    o = put_f32(p, o, set0);                           /* 29 set_speed0      */
    o = put_f32(p, o, odrive.now_speed0);              /* 33 now_speed0      */
    o = put_f32(p, o, tap_pwm_accel);                  /* 37 pwm_accel       */
    o = put_f32(p, o, Fly_Gain);                       /* 41 fly_gain        */
    o = put_f32(p, o, Servo_zhongzhi_Gain);            /* 45 servo_zhongzhi  */
    o = put_f32(p, o, (float)Servo_Ctl);               /* 49 servo_ctl       */
    o = put_f32(p, o, Servo_Lim);                      /* 53 servo_lim       */
    o = put_f32(p, o, tap_inner_p);                    /* 57 inner_p         */
    o = put_f32(p, o, tap_inner_i);                    /* 61 inner_i         */
    o = put_f32(p, o, tap_inner_d);                    /* 65 inner_d         */
    o = put_u16(p, o, imu_crc_err);                    /* 69 crc_err_cnt     */
    o = put_u16(p, o, flags);                          /* 71 flags           */

    return o;
}


/* ---- called at the end of balance(), i.e. in the UART8 RX ISR ---- */
void telem_task(void)
{
    uint16_t crc;

    loop_cnt++;
    if ((loop_cnt % TELEM_DECIM) != 0u) return;

    /* If the previous frame is still on the wire, skip this one rather than
       overwrite the buffer DMA is reading. 78 bytes at 460800 take ~1.7ms,
       the frame period at TELEM_DECIM=2 is 5ms, so this should not trigger.
       Checking gState (TX) leaves the RX side of huart6 alone. */
    if (TELEM_UART->gState != HAL_UART_STATE_READY)
    {
        telem_tx_drop++;
        return;
    }

    frame[0] = TELEM_MAGIC0;
    frame[1] = TELEM_MAGIC1;
    frame[2] = (uint8_t)TELEM_PAYLOAD_LEN;

    (void)telem_pack(&frame[3]);

    crc = telem_crc16_ccitt(&frame[2], 1u + TELEM_PAYLOAD_LEN);  /* LEN+payload */
    frame[3 + TELEM_PAYLOAD_LEN] = (uint8_t)(crc & 0xFF);
    frame[4 + TELEM_PAYLOAD_LEN] = (uint8_t)(crc >> 8);

    if (HAL_UART_Transmit_DMA(TELEM_UART, frame, TELEM_FRAME_LEN) != HAL_OK)
        telem_tx_drop++;
}

#endif /* TELEM_ENABLE */
