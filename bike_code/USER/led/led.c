#include "led.h"
#include "stm32f4xx_hal.h"

/* Order is A..H, i.e. physically first to last.  CubeMX's LED_8 is the
 * physically first one, so this table counts down. */
static const uint16_t led_pin[LED_COUNT] = {
    GPIO_PIN_8,     /* A  PG8 - physically first */
    GPIO_PIN_7,     /* B  PG7 */
    GPIO_PIN_6,     /* C  PG6 */
    GPIO_PIN_5,     /* D  PG5 */
    GPIO_PIN_4,     /* E  PG4 */
    GPIO_PIN_3,     /* F  PG3 */
    GPIO_PIN_2,     /* G  PG2 */
    GPIO_PIN_1,     /* H  PG1 */
};

#define LED_PORT        GPIOG
#define LED_ALL_PINS    (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | \
                         GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8)

static uint8_t led_cur_mask = 0;
static uint8_t led_mode     = LED_MODE_CHASE;
static int8_t  led_step     = 0;
static int8_t  led_dir      = 1;

void led_write_mask(uint8_t mask)
{
    uint8_t i;

    for (i = 0; i < LED_COUNT; i++)
        HAL_GPIO_WritePin(LED_PORT, led_pin[i],
                          /* active low: lit means drive the pin low */
                          (mask & (1u << i)) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    led_cur_mask = mask;
}

uint8_t led_mask(void)     { return led_cur_mask; }
uint8_t led_get_mode(void) { return led_mode; }

void led_set_mode(uint8_t mode)
{
    led_mode = mode;
    led_step = 0;
    led_dir  = 1;
    if (mode == LED_MODE_OFF)
        led_write_mask(0);
}

void led_init(void)
{
    GPIO_InitTypeDef g;

    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* Off before the pins become outputs, so nothing flashes on the way. */
    HAL_GPIO_WritePin(LED_PORT, LED_ALL_PINS, GPIO_PIN_SET);

    g.Pin       = LED_ALL_PINS;
    g.Mode      = GPIO_MODE_OUTPUT_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = 0;
    HAL_GPIO_Init(LED_PORT, &g);

    led_write_mask(0);
    led_step = 0;
    led_dir  = 1;
}

void led_delay(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - t0) < ms)
        led_task();
}

void led_task(void)
{
    static uint32_t next_ms = 0;
    uint32_t now = HAL_GetTick();

    if (led_mode == LED_MODE_OFF)
        return;

    /* Unsigned wrap-safe; HAL_GetTick() rolls over after 49 days. */
    if ((uint32_t)(now - next_ms) < LED_CHASE_MS)
        return;
    next_ms = now;

    switch (led_mode)
    {
    case LED_MODE_CHASE:
        /* Bounce rather than wrap: A..H then H..A.  Turning at the ends rather
           than after them means each end LED is shown once per lap, not twice,
           so the sweep looks even. */
        led_write_mask((uint8_t)(1u << led_step));
        led_step = (int8_t)(led_step + led_dir);
        if (led_step >= (int8_t)(LED_COUNT - 1))
        {
            led_step = (int8_t)(LED_COUNT - 1);
            led_dir  = -1;
        }
        else if (led_step <= 0)
        {
            led_step = 0;
            led_dir  = 1;
        }
        break;

    default:
        break;
    }
}
