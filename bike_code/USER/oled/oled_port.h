/*
 * oled_port.h - board/transport layer for the SH1106 OLED on the RM-A board.
 *
 * The OLED is write-only, so instead of pulling in the whole HAL SPI module
 * (stm32f4xx_hal_spi.c is not part of this project, and the version shipped
 * with the RM_OLED example does not match this project's HAL) the SPI block is
 * driven directly through its registers.  ~40 lines, no new dependencies.
 *
 * There is NO chip-select line.  The OLED header (J2) carries only VCC, GND,
 * RST, DC, SCK, MOSI and the 5-way button's ADC tap - the panel's CS is tied
 * to GND on the module, i.e. permanently selected.  That is also why the SPI
 * block below runs with software NSS held high (SSM | SSI) and never drives a
 * CS pin: do not wire CS to anything but ground.
 *
 * Which SPI block?  The official RM_OLED example drives this header from SPI1
 * (PB3/PA7), but some board revisions label the header's nets SPI2_*.  Flip
 * OLED_USE_SPI2 if the panel stays dark and the schematic routes SCK to PB13.
 *
 *   OLED_USE_SPI2 = 0     OLED_USE_SPI2 = 1
 *   PB3  SPI1_SCK  AF5    PB13 SPI2_SCK  AF5   (also CAN2_TX, but CAN is
 *   PA7  SPI1_MOSI AF5    PB15 SPI2_MOSI AF5    never initialised here)
 *
 * Common to both: PB9 = OLED_DC (0 command / 1 data), PB10 = OLED_RST (active
 * low).  MISO is never configured - the panel is write-only - so PB4 stays
 * free as NJTRST.
 */

#ifndef __OLED_PORT_H
#define __OLED_PORT_H

#include "stm32f4xx.h"
#include <stdint.h>

#define OLED_USE_SPI2       0       /* 0 -> SPI1 (RM_OLED example default) */
#define OLED_SCK_ON_PA5     0       /* SPI1 only: alternate SCK mapping */

/* SH1106's datasheet gives a 250 ns minimum serial clock cycle => 4 MHz
 * ceiling.  Both dividers below land on ~1.4 MHz - deliberately slow for
 * bring-up.  Now that the panel is proven they are back at 2.81 MHz, the
 * fastest in-spec step, which halves the blocking time of a full refresh.
 * If the display ever shows tearing or noise, step back one notch: SPI1 to
 * 0x5 (/64) and SPI2 to 0x4 (/32), both ~1.41 MHz. */
#if OLED_USE_SPI2

#define OLED_SPI            SPI2
#define OLED_SPI_CLK_EN()   __HAL_RCC_SPI2_CLK_ENABLE()
#define OLED_SPI_AF         GPIO_AF5_SPI2
#define OLED_SPI_BR         (0x3u << 3)     /* APB1 45 MHz / 16 = 2.81 MHz */
#define OLED_SCK_PORT       GPIOB
#define OLED_SCK_PIN        GPIO_PIN_13
#define OLED_MOSI_PORT      GPIOB
#define OLED_MOSI_PIN       GPIO_PIN_15

#else

#define OLED_SPI            SPI1
#define OLED_SPI_CLK_EN()   __HAL_RCC_SPI1_CLK_ENABLE()
#define OLED_SPI_AF         GPIO_AF5_SPI1
#define OLED_SPI_BR         (0x4u << 3)     /* APB2 90 MHz / 32 = 2.81 MHz */
#if OLED_SCK_ON_PA5
#define OLED_SCK_PORT       GPIOA
#define OLED_SCK_PIN        GPIO_PIN_5
#else
#define OLED_SCK_PORT       GPIOB
#define OLED_SCK_PIN        GPIO_PIN_3      /* also JTDO -> SWO trace is lost */
#endif
#define OLED_MOSI_PORT      GPIOA
#define OLED_MOSI_PIN       GPIO_PIN_7

#endif

#define OLED_DC_GPIO_Port   GPIOB
#define OLED_DC_Pin         GPIO_PIN_9
#define OLED_RST_GPIO_Port  GPIOB
#define OLED_RST_Pin        GPIO_PIN_10

/* Bring up the SPI block + the DC/RST GPIOs.  Call once before oled_init(). */
void oled_port_init(void);

/* BRING-UP DIAGNOSTIC, NEVER RETURNS.  Releases SCK/MOSI/DC/RST from the SPI
 * block, drives all four as plain push-pull outputs and toggles them together
 * once a second.  Probe them at the *module* end of the cable with a
 * multimeter: each should swing 0 V <-> 3.3 V.  That proves clock/AF setup,
 * connector orientation and cable continuity without needing a scope.
 * Selected by UI_PIN_TEST in ui.h. */
void oled_port_pin_test(void);

/* Blocking single-byte write.  Returns only once the byte has fully left the
 * shift register, so the caller may change DC immediately afterwards. */
void oled_spi_write(uint8_t d);

/* Blocking block write.  Same as calling oled_spi_write() n times but without
 * waiting for BSY between bytes, which is what makes a full-screen refresh
 * affordable.  DC must already be correct for the whole block. */
void oled_spi_write_buf(const uint8_t *p, uint32_t n);

#endif /* __OLED_PORT_H */
