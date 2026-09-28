/*
 * disp_port.h - SPI1 transport shared by both panel drivers.
 *
 * Was USER/oled/oled_port.*; renamed when the ILI9163 TFT was added, because
 * both panels sit on the same bus and the same DC/RST lines and only one of
 * them can be fitted at a time (see display.h).
 *
 * Driven through SPI1's registers rather than HAL_SPI: stm32f4xx_hal_spi.c is
 * not part of this project and the version shipped with the reference examples
 * does not match this project's HAL.  ~60 lines, no new dependencies, and no
 * TxCplt callback to fight over.
 *
 * Wiring (the board's OLED header, J17):
 *   PB3  SPI1_SCK   AF5   -> SCL / D0
 *   PA7  SPI1_MOSI  AF5   -> SDA / D1
 *   PB9  DC               -> 0 = command, 1 = data
 *   PB10 RST              -> active low
 *   PA6  CS               -> TFT only, held low for good; the OLED module's
 *                            CS is tied to GND and PA6 is left alone.
 * MISO is never configured - both panels are write-only - so PB4 stays free
 * as NJTRST.
 *
 * Two hardware facts, both established on this board and carried over from the
 * reference implementation; do not "simplify" either away:
 *
 *  1. DC must be stable for the whole byte it labels, so the in-flight byte has
 *     to land before DC moves.  That wait belongs at the DC transition, not
 *     after every byte: waiting per byte serialises the transfer and roughly
 *     doubled a full-screen refresh when it was measured on the OLED.
 *  2. A transfer is not over when the last byte has been handed to the shift
 *     register.  Anything that follows a block write - a DC change, raising CS -
 *     must wait for BSY to clear first.
 */

#ifndef __DISP_PORT_H
#define __DISP_PORT_H

#include "stm32f4xx.h"
#include <stdint.h>
#include "display.h"

/* SPI1 is on APB2, which this project clocks at 90 MHz (SYSCLK 180 / 2).
 * BR field = bits 5:3, divider = 2^(BR+1).
 *   0x2 -> /8  = 11.25 MHz     0x3 -> /16 = 5.63 MHz
 *   0x4 -> /32 = 2.81 MHz      0x5 -> /64 = 1.41 MHz
 * (The reference project quotes 10.5 MHz for /8 because its APB2 was 84 MHz.) */
#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
/* ILI9163B takes well over 10 MHz.  It matters here: the TFT has no frame
 * buffer, so every character is 256 bytes on the wire and the refresh cost is
 * proportional to the area redrawn. */
#define DISP_SPI_BR         (0x2u << 3)     /* 11.25 MHz */
#else
/* SH1106's datasheet gives a 250 ns minimum serial clock cycle => 4 MHz
 * ceiling, so /32 is the fastest in-spec step. */
#define DISP_SPI_BR         (0x4u << 3)     /* 2.81 MHz */
#endif

#define DISP_SCK_PORT       GPIOB
#define DISP_SCK_PIN        GPIO_PIN_3      /* also JTDO -> SWO trace is lost */
#define DISP_MOSI_PORT      GPIOA
#define DISP_MOSI_PIN       GPIO_PIN_7
#define DISP_DC_PORT        GPIOB
#define DISP_DC_PIN         GPIO_PIN_9
#define DISP_RST_PORT       GPIOB
#define DISP_RST_PIN        GPIO_PIN_10
#define DISP_CS_PORT        GPIOA
#define DISP_CS_PIN         GPIO_PIN_6

/* Bring up SPI1, DC/RST, and (TFT only) CS.  Call once before the driver's
 * own init.  CS, where present, is taken low and stays there: there is one
 * device on the bus, and raising CS between a command and its parameters makes
 * the controller discard the command. */
void disp_port_init(void);

/* Low/high on RST with the two settling delays the panel datasheet asks for,
 * in milliseconds.  Also forgets the cached DC state - after a reset the panel
 * side is unknown. */
void disp_reset_pulse(uint32_t low_ms, uint32_t settle_ms);

/* DC control.  data = 0 selects command, 1 selects pixel/parameter data.
 * Cheap when nothing changes; pays one BSY wait when it does (fact 1 above). */
void disp_set_dc(uint8_t data);

/* Block until the shift register is empty (fact 2 above). */
void disp_sync(void);

/* Queue one byte.  Returns once the byte has reached the shift register, so
 * consecutive calls pipeline. */
void disp_write(uint8_t d);

/* Block write.  Waits for BSY once at the end, not per byte. */
void disp_write_buf(const uint8_t *p, uint32_t n);

/* Write a 16-bit value n times, MSB first - the RGB565 pixel fill the TFT
 * needs for clears and rectangles.  Built out of a small static buffer so it
 * costs no RAM proportional to the area. */
void disp_fill16(uint16_t v, uint32_t n);

/* BRING-UP DIAGNOSTIC, NEVER RETURNS.  Releases SCK/MOSI/DC/RST (and CS) from
 * the SPI block, drives them as plain push-pull outputs and toggles them
 * together once a second.  Probe at the *module* end with a multimeter: each
 * should swing 0 V <-> 3.3 V.  Proves pin mapping, cable continuity and
 * connector orientation without a scope.  Selected by UI_PIN_TEST in ui.h. */
void disp_port_pin_test(void);

#endif /* __DISP_PORT_H */
