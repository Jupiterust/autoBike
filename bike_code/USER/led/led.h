/*
 * led.h - the 8 LEDs on the development board, PG1..PG8.
 *
 * Ported to C from the same C++ project the TFT driver came from.
 *
 * Numbering: the LEDs are addressed A..H, where A is the physically first one.
 * CubeMX's LED_1..LED_8 labels map to PG1..PG8, but the physical order is
 * reversed - PG8 is the first LED - so A is PG8 and H is PG1.  The letters
 * exist precisely so "which one is number 1" never has to be asked.
 *
 * Polarity: active low.  Driving a pin low lights its LED (common anode).
 *
 * Unlike the project this came from, CubeMX here knows nothing about these
 * pins: the .ioc only assigns PG9/PG14 (USART6) on port G and gpio.c never
 * touches PG1..PG8, so they sit in their reset state (floating inputs) until
 * led_init() configures them.  Writing the ODR before that does nothing.
 */

#ifndef __LED_H
#define __LED_H

#include <stdint.h>

#define LED_COUNT       8u

/* Bit positions for led_write_mask(): bit0 = A (PG8) ... bit7 = H (PG1). */
#define LED_A           (1u << 0)
#define LED_B           (1u << 1)
#define LED_C           (1u << 2)
#define LED_D           (1u << 3)
#define LED_E           (1u << 4)
#define LED_F           (1u << 5)
#define LED_G           (1u << 6)
#define LED_H           (1u << 7)

/* Display modes.  Start simple; the point of routing everything through
 * led_task() is that swapping in a status-bar or fault-indicator mode later
 * touches one switch statement and nothing else. */
#define LED_MODE_OFF    0u
#define LED_MODE_CHASE  1u      /* one lit, bouncing A -> H -> A */

/* Milliseconds per chase step; 14 steps make one there-and-back lap. */
#define LED_CHASE_MS    80u

/* Configure PG1..PG8 as push-pull outputs and turn everything off.
 * Call once from Sys_All_Init(). */
void led_init(void);

/* Call every main-loop pass; self-throttles to LED_CHASE_MS.
 * Does nothing in LED_MODE_OFF. */
void led_task(void);

/* Exactly HAL_Delay(), except it keeps led_task() running.  Sys_All_Init()
 * blocks for 31 s of upright calibration and 2 s of re-stand before the main
 * loop ever starts, and a chase light that only begins a minute after power-on
 * is not much of a sign of life. */
void led_delay(uint32_t ms);

void led_set_mode(uint8_t mode);
uint8_t led_get_mode(void);

/* Drive all eight at once.  Also switches to LED_MODE_OFF so the animation
 * does not immediately overwrite what was just written. */
void led_write_mask(uint8_t mask);

/* What is currently lit. */
uint8_t led_mask(void);

#endif /* __LED_H */
