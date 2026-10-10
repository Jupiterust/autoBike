#ifndef BIKE_UPPER_H
#define BIKE_UPPER_H

#include <stdint.h>

/* Protocol angles describe the handlebar/front wheel, positive RIGHT.
 * Hardware-confirmed calibration: +90 internal units turns LEFT 90 deg.
 * Wire values are whole degrees: -90..+90 means -90..+90 deg.
 * Servo_Delta (currently 90) remains the independent mechanical limit. */
#define UPPER_STEER_REFERENCE_UNITS 90
#define UPPER_STEER_REFERENCE_DEG   90
#define UPPER_STEER_LIMIT_DEG       90
#define UPPER_RPM_MAX               180 /* existing 3 motor turns/s */
#ifndef UPPER_TIMEOUT_MS
#define UPPER_TIMEOUT_MS            100u
#endif
#define UPPER_ECHO_PERIOD_MS        100u

#define UPPER_LINK_OK      (1u << 0)
#define UPPER_TIMEOUT_SEEN (1u << 1)
#define UPPER_MOTOR_EN     (1u << 2)
#define UPPER_MEASURED     (1u << 3)
#define UPPER_CLAMPED      (1u << 4)

/* 0 = handset, 1 = onboard host. Only the handset S command changes it. */
extern volatile uint8_t upper_Flag;

/* Read-only foreground snapshot. servo/rpm are the same SOFTWARE values as
 * the protocol echo. timed_out is the current fault, not TIMEOUT_SEEN's
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
