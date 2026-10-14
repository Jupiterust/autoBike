#ifndef BIKE_UPPER_H
#define BIKE_UPPER_H

#include <stdint.h>

/* LEGACY HOST SCALE, intentionally unchanged by the RC actual-angle update.
 * Command/echo/bounds: signed little-endian int16, NO x10 scaling, right +.
 * Despite the historical *_DEG names, one wire unit maps to one PWM control
 * unit with opposite sign; it is NOT one physical servo-shaft degree.
 * Measured 120 PWM units ~=21 physical deg about the existing straight pose.
 * Thus wire +120 -> Servo_Ctl -120 -> estimated +21 deg, NOT +120 deg.
 * RC L/R uses its own 85-unit (~14.875 deg) cap and calibrated TFT display.
 * Shared Servo_Delta stays 120 to keep host commands/echo behavior unchanged.
 * No steering position sensor: MEASURED_ANGLE remains zero. */
#define UPPER_STEER_REFERENCE_UNITS 120
#define UPPER_STEER_REFERENCE_DEG   120
#define UPPER_STEER_LIMIT_DEG       120
#define UPPER_RPM_MAX               240 /* 4 motor turns/s; final drive limit */
#ifndef UPPER_TIMEOUT_MS
#define UPPER_TIMEOUT_MS            100u
#endif
#define UPPER_ECHO_PERIOD_MS        100u
/* Speed replies alternate with position (~5 Hz each). Allow two missed
 * speed updates before falling back to the command; independent of RC timeout. */
#define UPPER_RPM_FEEDBACK_MAX_AGE_MS 500u

/* AA 57 49 5A query -> AA 58 pitch/roll/yaw status CRC 5A (11 bytes).
 * Angles are signed LE int16 in 0.01 deg, NOT the legacy servo wire units.
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

/* Read-only foreground snapshot. servo is legacy wire units, positive RIGHT;
 * rpm is measured motor revolutions/minute when MEASURED_RPM is set, otherwise
 * the final speed command. servo remains a software output (no sensor).
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
