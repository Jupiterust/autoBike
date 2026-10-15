#ifndef BIKE_UPPER_H
#define BIKE_UPPER_H

#include <stdint.h>
#include "../Servo/servo_limits.h"

/* Command/echo/bounds: signed little-endian int16, servo-shaft deg * 100.
 * Positive RIGHT, relative to the existing straight-ahead pose. Calibration
 * is shared with the TFT: 120 PWM units ~=21 deg. Commands round to the
 * nearest PWM unit; echoes round the applied output to 0.01 deg (half away
 * from zero). The host envelope remains +/-120 PWM units (+/-2100 on wire).
 * RC L/R keeps its separate 114-unit cap; its endpoint echo is +/-1995.
 * This replaces the legacy 1:1 wire/PWM scale; hosts must migrate together.
 * No steering position sensor: MEASURED_ANGLE remains zero. */
#define UPPER_STEER_REFERENCE_UNITS SERVO_CAL_UNITS
#define UPPER_STEER_REFERENCE_CDEG  (SERVO_CAL_DEG * 100)
#define UPPER_STEER_LIMIT_CDEG      (Servo_Delta * UPPER_STEER_REFERENCE_CDEG / UPPER_STEER_REFERENCE_UNITS)
#define UPPER_RPM_MAX               240 /* 4 motor turns/s; final drive limit */
#ifndef UPPER_TIMEOUT_MS
#define UPPER_TIMEOUT_MS            100u
#endif
#define UPPER_ECHO_PERIOD_MS        100u
/* Speed replies alternate with position (~5 Hz each). Allow two missed
 * speed updates before falling back to the command; independent of RC timeout. */
#define UPPER_RPM_FEEDBACK_MAX_AGE_MS 500u

/* AA 57 49 5A query -> AA 58 pitch/roll/yaw status CRC 5A (11 bytes).
 * Body angles are signed LE int16 in 0.01 deg, independent of the servo axis.
 * Positive: nose up / lean right / clockwise viewed from above. */
#define UPPER_IMU_MAX_AGE_MS 100u
#define UPPER_IMU_VALID      0x01u
#define UPPER_IMU_CALIBRATED 0x02u /* reserved until calibration is confirmed */

#define UPPER_LINK_OK      (1u << 0)
#define UPPER_TIMEOUT_SEEN (1u << 1)
#define UPPER_MOTOR_EN     (1u << 2)
#define UPPER_MEASURED_RPM (1u << 3) /* 0x08: fresh ODrive speed feedback */
#define UPPER_CLAMPED      (1u << 4)
#define UPPER_MEASURED_ANGLE (1u << 5) /* 0x20: no steering sensor, always 0 */
/* Bits 6..7 reserved, always 0. TIMEOUT_SEEN also latches a protection stop. */

/* 0 = handset, 1 = onboard host. Only the handset S command changes it. */
extern volatile uint8_t upper_Flag;

/* Read-only foreground snapshot. servo is shaft deg * 100, positive RIGHT;
 * rpm is measured motor revolutions/minute when MEASURED_RPM is set, otherwise
 * the final speed command. servo is calibrated from the applied PWM output (no sensor).
 * Both match the protocol echo. timed_out is the current timeout, not TIMEOUT_SEEN's
 * sticky history, so the display can show recovery correctly. */
typedef struct {
    int16_t servo, rpm;
    uint8_t status;
    uint8_t command_seen, timed_out, balance_on, restarting;
} upper_state_t;

void upper_get_state(upper_state_t *state);
void upper_init(void);
void Upper(uint8_t byte);           /* USART2 RX ISR: bounded frame parser */
void upper_rx_reset(void);          /* discard a partial frame on UART error */
void upper_task(void);              /* foreground: non-blocking replies */
void upper_toggle_mode(void);       /* called by the handset dispatcher */
void upper_stop_control(void);      /* centre/stop, discard old host targets */
uint8_t upper_control_tick(void);   /* balance ISR; 1 = send rear stop now */
float upper_limit_drive(float speed); /* limit AFTER the existing turn assist */
uint8_t upper_crc8(const uint8_t *data, uint16_t size);

#endif
