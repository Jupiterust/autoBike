/*
 * oled_port.c - register-level SPI transport for the SH1106 OLED.
 * See oled_port.h for the pin map, the SPI1/SPI2 switch, and why there is no
 * chip-select handling anywhere in this driver.
 */

#include "oled_port.h"
#include "stm32f4xx_hal.h"

void oled_port_init(void)
{
    GPIO_InitTypeDef g;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    OLED_SPI_CLK_EN();

    /* --- SCK / MOSI --------------------------------------------------- */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = OLED_SPI_AF;

    g.Pin = OLED_SCK_PIN;
    HAL_GPIO_Init(OLED_SCK_PORT, &g);

    g.Pin = OLED_MOSI_PIN;
    HAL_GPIO_Init(OLED_MOSI_PORT, &g);

    /* MISO deliberately left unconfigured: the panel is write-only. */

    /* --- DC / RST ----------------------------------------------------- */
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port,  OLED_DC_Pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET);

    g.Mode      = GPIO_MODE_OUTPUT_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = 0;
    g.Pin       = OLED_DC_Pin | OLED_RST_Pin;
    HAL_GPIO_Init(GPIOB, &g);

    /* --- SPI ---------------------------------------------------------- */
    OLED_SPI->CR1 = 0;                  /* SPE = 0 while reconfiguring */
    OLED_SPI->CR2 = 0;                  /* no interrupts, no DMA, no SS output */

    /* Full-duplex master, 8-bit, MSB first, software NSS held high (SSM | SSI
     * - without SSI a master immediately mode-faults back to slave), SH1106 is
     * SPI mode 0: CPOL = 0, CPHA = 0.
     *
     * Full-duplex rather than half-duplex TX-only on purpose: this is bit-for-
     * bit the direction the RM_OLED reference uses (SPI_DIRECTION_2LINES), and
     * during bring-up matching the known-good configuration beats saving a pin
     * we were not using anyway.  MISO stays unconfigured; the RX side just
     * overruns into a flag nobody reads. */
    OLED_SPI->CR1 = SPI_CR1_SSM
                  | SPI_CR1_SSI
                  | SPI_CR1_MSTR
                  | OLED_SPI_BR;
    /* DFF = 0 (8-bit), LSBFIRST = 0 (MSB), CPOL = 0, CPHA = 0 all left clear. */

    OLED_SPI->CR1 |= SPI_CR1_SPE;
}

void oled_spi_sync(void)
{
    while (OLED_SPI->SR & SPI_SR_BSY) { }
}

void oled_spi_write(uint8_t d)
{
    while (!(OLED_SPI->SR & SPI_SR_TXE)) { }
    *(volatile uint8_t *)&OLED_SPI->DR = d;     /* 8-bit access: one byte only */
    /* No BSY wait here on purpose.  TXE means the byte has reached the shift
     * register, so the next one can be queued immediately and the hardware
     * pipelines them back to back - which is the whole point of a shift
     * register.  Waiting for BSY after every byte serialised the transfer and
     * roughly doubled a full-screen refresh.  The one place the in-flight byte
     * genuinely has to land first is a DC change, and oled_set_dc() in oled.c
     * calls oled_spi_sync() for exactly that. */
}

void oled_spi_write_buf(const uint8_t *p, uint32_t n)
{
    while (n--)
    {
        while (!(OLED_SPI->SR & SPI_SR_TXE)) { }
        *(volatile uint8_t *)&OLED_SPI->DR = *p++;
    }
    while (OLED_SPI->SR & SPI_SR_BSY) { }
}

void oled_port_pin_test(void)
{
    GPIO_InitTypeDef g;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    OLED_SPI->CR1 = 0;              /* hand the pins back from the SPI block */

    g.Mode      = GPIO_MODE_OUTPUT_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = 0;

    g.Pin = OLED_SCK_PIN;
    HAL_GPIO_Init(OLED_SCK_PORT, &g);
    g.Pin = OLED_MOSI_PIN;
    HAL_GPIO_Init(OLED_MOSI_PORT, &g);
    g.Pin = OLED_DC_Pin | OLED_RST_Pin;
    HAL_GPIO_Init(GPIOB, &g);

    for (;;)
    {
        HAL_GPIO_WritePin(OLED_SCK_PORT,  OLED_SCK_PIN,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(OLED_MOSI_PORT, OLED_MOSI_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin | OLED_RST_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(OLED_SCK_PORT,  OLED_SCK_PIN,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OLED_MOSI_PORT, OLED_MOSI_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, OLED_DC_Pin | OLED_RST_Pin, GPIO_PIN_RESET);
        HAL_Delay(1000);
    }
}
