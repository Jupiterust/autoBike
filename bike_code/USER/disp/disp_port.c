/*
 * disp_port.c - register-level SPI1 transport shared by the SH1106 and the
 * ILI9163 drivers.  See disp_port.h for the pin map and the two hardware
 * facts this implementation is built around.
 */

#include "disp_port.h"
#include "stm32f4xx_hal.h"

/* Mirrors the DC line so the BSY guard is paid only when DC actually moves.
 * 0xff = unknown, force a write. */
static uint8_t disp_dc_state = 0xff;

void disp_port_init(void)
{
    GPIO_InitTypeDef g;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* --- SCK / MOSI --------------------------------------------------- */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI1;

    g.Pin = DISP_SCK_PIN;
    HAL_GPIO_Init(DISP_SCK_PORT, &g);
    g.Pin = DISP_MOSI_PIN;
    HAL_GPIO_Init(DISP_MOSI_PORT, &g);

    /* MISO deliberately left unconfigured: both panels are write-only. */

    /* --- DC / RST ----------------------------------------------------- */
    HAL_GPIO_WritePin(DISP_DC_PORT,  DISP_DC_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DISP_RST_PORT, DISP_RST_PIN, GPIO_PIN_RESET);

    g.Mode      = GPIO_MODE_OUTPUT_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = 0;
    g.Pin       = DISP_DC_PIN | DISP_RST_PIN;
    HAL_GPIO_Init(GPIOB, &g);

#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
    /* --- CS ----------------------------------------------------------- *
     * Low before it becomes an output, so the panel never sees a glitch.
     * It then stays low for good.  Only configured for the TFT: on the OLED
     * this pin carries the module's 5-way button tap and must stay an input. */
    HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS_PIN, GPIO_PIN_RESET);
    g.Pin = DISP_CS_PIN;
    HAL_GPIO_Init(DISP_CS_PORT, &g);
#endif

    /* --- SPI1 --------------------------------------------------------- */
    SPI1->CR1 = 0;                  /* SPE = 0 while reconfiguring */
    SPI1->CR2 = 0;                  /* no interrupts, no DMA, no SS output */

    /* Full-duplex master, 8-bit, MSB first, software NSS held high
     * (SSM | SSI - without SSI a master immediately mode-faults back to
     * slave).  Both panels are SPI mode 0: CPOL = 0, CPHA = 0.
     *
     * Full-duplex rather than half-duplex TX-only because that is the
     * direction the working reference drivers use; MISO stays unconfigured
     * and the RX side just overruns into a flag nobody reads. */
    SPI1->CR1 = SPI_CR1_SSM
              | SPI_CR1_SSI
              | SPI_CR1_MSTR
              | DISP_SPI_BR;
    /* DFF = 0 (8-bit), LSBFIRST = 0 (MSB), CPOL = 0, CPHA = 0 all left clear. */

    SPI1->CR1 |= SPI_CR1_SPE;

    disp_dc_state = 0xff;           /* panel side unknown after a re-init */
}

void disp_reset_pulse(uint32_t low_ms, uint32_t settle_ms)
{
    HAL_GPIO_WritePin(DISP_RST_PORT, DISP_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(DISP_RST_PORT, DISP_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(low_ms);
    HAL_GPIO_WritePin(DISP_RST_PORT, DISP_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(settle_ms);
    disp_dc_state = 0xff;
}

void disp_sync(void)
{
    while (SPI1->SR & SPI_SR_BSY) { }
}

void disp_set_dc(uint8_t data)
{
    if (disp_dc_state == data)
        return;

    disp_sync();                    /* let the in-flight byte land first */

    HAL_GPIO_WritePin(DISP_DC_PORT, DISP_DC_PIN,
                      data ? GPIO_PIN_SET : GPIO_PIN_RESET);
    disp_dc_state = data;
}

void disp_write(uint8_t d)
{
    while (!(SPI1->SR & SPI_SR_TXE)) { }
    *(volatile uint8_t *)&SPI1->DR = d;     /* 8-bit access: one byte only */
    /* No BSY wait here on purpose - see disp_port.h, fact 1. */
}

void disp_write_buf(const uint8_t *p, uint32_t n)
{
    while (n--)
    {
        while (!(SPI1->SR & SPI_SR_TXE)) { }
        *(volatile uint8_t *)&SPI1->DR = *p++;
    }
    disp_sync();
}

void disp_fill16(uint16_t v, uint32_t n)
{
    /* One panel row's worth (160 px), so a full-screen clear is 132 bursts
     * rather than 21120 byte writes, at a fixed 320 bytes of .bss. */
    static uint8_t fill[160 * 2];
    static uint16_t fill_val = 0;
    static uint8_t  fill_ok  = 0;
    uint32_t i;

    if (n == 0u)
        return;

    if (!fill_ok || fill_val != v)
    {
        for (i = 0; i < 160u; i++)
        {
            fill[i * 2]     = (uint8_t)(v >> 8);
            fill[i * 2 + 1] = (uint8_t)(v & 0xFF);
        }
        fill_val = v;
        fill_ok  = 1;
    }

    while (n > 0u)
    {
        uint32_t batch = (n > 160u) ? 160u : n;
        disp_write_buf(fill, batch * 2u);
        n -= batch;
    }
}

void disp_port_pin_test(void)
{
    GPIO_InitTypeDef g;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    SPI1->CR1 = 0;                  /* hand the pins back from the SPI block */

    g.Mode      = GPIO_MODE_OUTPUT_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = 0;

    g.Pin = DISP_SCK_PIN;
    HAL_GPIO_Init(DISP_SCK_PORT, &g);
    g.Pin = DISP_MOSI_PIN;
    HAL_GPIO_Init(DISP_MOSI_PORT, &g);
    g.Pin = DISP_DC_PIN | DISP_RST_PIN;
    HAL_GPIO_Init(GPIOB, &g);
#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
    g.Pin = DISP_CS_PIN;
    HAL_GPIO_Init(DISP_CS_PORT, &g);
#endif

    for (;;)
    {
        HAL_GPIO_WritePin(DISP_SCK_PORT,  DISP_SCK_PIN,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(DISP_MOSI_PORT, DISP_MOSI_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, DISP_DC_PIN | DISP_RST_PIN, GPIO_PIN_SET);
#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
        HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS_PIN, GPIO_PIN_SET);
#endif
        HAL_Delay(1000);
        HAL_GPIO_WritePin(DISP_SCK_PORT,  DISP_SCK_PIN,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DISP_MOSI_PORT, DISP_MOSI_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, DISP_DC_PIN | DISP_RST_PIN, GPIO_PIN_RESET);
#if DISPLAY_DRIVER == DISPLAY_TFT_ILI9163
        HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS_PIN, GPIO_PIN_RESET);
#endif
        HAL_Delay(1000);
    }
}
