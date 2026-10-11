#ifndef BIKE_UPPER_H
#define BIKE_UPPER_H

#include <stdint.h>

/* Protocol angles describe the front steering SERVO shaft relative to centre,
 * not the handlebar or front-wheel angle after the mechanical linkage.
 * Positive means right steering; +120 internal units means -120 servo degrees.
 * Wire values are whole degrees: -120..+120 means -120..+120 deg.
 * Servo_Delta (currently 120) remains the independent mechanical limit. */
#define UPPER_STEER_REFERENCE_UNITS 120
#define UPPER_STEER_REFERENCE_DEG   120
#define UPPER_STEER_LIMIT_DEG       120
#define UPPER_RPM_MAX               180 /* existing 3 motor turns/s */
#ifndef UPPER_TIMEOUT_MS
#define UPPER_TIMEOUT_MS            100u
#endif
#define UPPER_ECHO_PERIOD_MS        100u
/* Speed replies alternate with position (~5 Hz each). Allow two missed
 * speed updates before falling back to the command; independent of RC timeout. */
#define UPPER_RPM_FEEDBACK_MAX_AGE_MS 500u

#define UPPER_LINK_OK      (1u << 0)
#define UPPER_TIMEOUT_SEEN (1u << 1)
#define UPPER_MOTOR_EN     (1u << 2)
#define UPPER_MEASURED_RPM (1u << 3) /* 0x08: fresh ODrive speed feedback */
#define UPPER_CLAMPED      (1u << 4)
#define UPPER_MEASURED_ANGLE (1u << 5) /* 0x20: no steering sensor, always 0 */
/* Bits 6..7 reserved, always 0. TIMEOUT_SEEN also latches a protection stop. */

/* 0 = handset, 1 = onboard host. Only the handset S command changes it. */
extern volatile uint8_t upper_Flag;

/* Read-only foreground snapshot. servo is whole degrees, positive RIGHT;
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
