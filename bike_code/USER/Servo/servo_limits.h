#ifndef BIKE_SERVO_LIMITS_H
#define BIKE_SERVO_LIMITS_H

/* Measured at the servo shaft relative to the EXISTING straight-ahead pose:
 * 120 internal PWM units ~= 21 degrees. Linear, symmetric extrapolation is
 * an estimate, NOT position feedback or verification of the new endpoints.
 * Host command/echo scaling is intentionally unchanged in this RC-only step. */
#define SERVO_CAL_UNITS       120
#define SERVO_CAL_DEG         21
#define SERVO_RC_LIMIT_DEG    20
#define SERVO_DEG_PER_UNIT    ((float)SERVO_CAL_DEG / SERVO_CAL_UNITS)
/* Actual servo-shaft travel target about the existing straight-ahead pose.
 * Floor to 114 PWM units (~19.95 physical deg) to stay within +/-20 deg.
 * This is calibration-based open-loop positioning, not measured feedback. */
#define SERVO_RC_LIMIT_UNITS  (SERVO_RC_LIMIT_DEG * SERVO_CAL_UNITS / SERVO_CAL_DEG)
/* Shared PWM envelope stays at 120 units to preserve the HOST path.
 * RC L/R commands use SERVO_RC_LIMIT_UNITS separately. Do not reduce this
 * shared envelope to the RC limit: that would silently clamp host commands.
 * Neither this value nor Servo_Ctl is a physical degree count. */
#define Servo_Delta          120

#endif
