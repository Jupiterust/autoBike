#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#include "A_include.h"

/* ------------------------------------------------------------------------
 * PID tuning telemetry:  STM32 -> PC binary stream.
 *
 * Contract with pid_tuning_bridge.py is exactly three things:
 *   TELEM_PROTO_VERSION + the payload layout below + the CRC algorithm.
 *
 * This module is telemetry ONLY. It never writes a control variable, and
 * the taps below only copy values the control code already computed.
 *
 * Frame (little endian):
 *   [0xAA][0x55][LEN][payload, LEN bytes][CRC16_L][CRC16_H]
 *   CRC16-CCITT (poly 0x1021, init 0xFFFF) computed over LEN + payload.
 *
 * Payload = 73 bytes, packed, no padding, byte-identical to SCHEMA:
 *   off type field                  off type field
 *    0  u8   ver                     37  f32  pwm_accel
 *    1  u32  loop_cnt                41  f32  fly_gain
 *    5  u32  t_ms                    45  f32  servo_zhongzhi_gain
 *    9  f32  imu_rol                 49  f32  servo_ctl
 *   13  f32  angle_target            53  f32  servo_lim
 *   17  f32  imu_vx                  57  f32  inner_p
 *   21  f32  pwm_x                   61  f32  inner_i
 *   25  f32  set_speed0_raw          65  f32  inner_d
 *   29  f32  set_speed0              69  u16  crc_err_cnt
 *   33  f32  now_speed0              71  u16  flags
 * ------------------------------------------------------------------------ */

/* ---- compile-time switches ---- */
#define TELEM_ENABLE          1        /* 0 -> all hooks compile to nothing  */
#define TELEM_DECIM           2        /* 1 frame per N balance() calls      */
                                       /* 400Hz / 2 = 200Hz                  */
#define TELEM_PROTO_VERSION   1
#define TELEM_UART            (&huart6)   /* USART6, 460800 8N1 (PG9/PG14)   */

/* ---- wire format ---- */
#define TELEM_MAGIC0          0xAAu
#define TELEM_MAGIC1          0x55u
#define TELEM_PAYLOAD_LEN     73u
#define TELEM_FRAME_LEN       (3u + TELEM_PAYLOAD_LEN + 2u)   /* 78 bytes */

/* ---- flags bitfield (payload offset 71) ---- */
#define TELEM_FLAG_M0         (1u << 0)   /* param.M0_Flag  (balance on)     */
#define TELEM_FLAG_M1         (1u << 1)   /* param.M1_Flag  (rear wheel on)  */
#define TELEM_FLAG_RESTART    (1u << 2)   /* restart_flag                    */
#define TELEM_FLAG_UPPER      (1u << 3)   /* upper_Flag                      */
#define TELEM_FLAG_SAT        (1u << 4)   /* flywheel command was clamped    */
#define TELEM_FLAG_IMU_CRC    (1u << 5)   /* IMU CRC error since last frame  */

#if TELEM_ENABLE

/* Called once at the very end of balance(), after the control output has
   been computed, clamped and dispatched. Increments loop_cnt, applies
   TELEM_DECIM, packs and starts the DMA transmit. */
void telem_task(void);

/* Taps, called from PID.c. They only latch values for the next frame. */
void telem_tap_angle_target(float angle_target);
void telem_tap_inner(float p, float i, float d, float pwm_x, float raw_out);
void telem_tap_pwm_accel(float pwm_accel);

/* Called from CH100.c when an IMU frame fails its CRC check. */
void telem_note_imu_crc_err(void);

uint16_t telem_crc16_ccitt(const uint8_t *d, uint32_t n);

/* Frames skipped because the previous DMA transmit had not finished.
   Not on the wire; read it in the debugger. Gaps are visible on the PC
   side anyway, as jumps in loop_cnt. */
extern volatile uint32_t telem_tx_drop;

#else  /* telemetry compiled out */

#define telem_task()                       do {} while (0)
#define telem_tap_angle_target(a)          do {} while (0)
#define telem_tap_inner(p, i, d, x, r)     do {} while (0)
#define telem_tap_pwm_accel(a)             do {} while (0)
#define telem_note_imu_crc_err()           do {} while (0)

#endif /* TELEM_ENABLE */

#endif /* __TELEMETRY_H */
