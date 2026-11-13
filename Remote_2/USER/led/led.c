/*
 * led.c - 见 led.h。
 */

#include "led.h"
#include "main.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} led_pin_t;

static const led_pin_t led_pins[LED_COUNT] =
{
    { GPIOB, GPIO_PIN_4 },      /* LED0 */
    { GPIOB, GPIO_PIN_5 }       /* LED1 */
};

static volatile uint8_t led_state[LED_COUNT];
static volatile uint16_t led_timer[LED_COUNT];

#if LED_ACTIVE_HIGH
#define LED_LEVEL(on)   ((on) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#else
#define LED_LEVEL(on)   ((on) ? GPIO_PIN_RESET : GPIO_PIN_SET)
#endif

void led_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
        g.Pin = led_pins[i].pin;
        HAL_GPIO_Init(led_pins[i].port, &g);
        led_set((led_id_t)i, 0);
        led_timer[i] = 0;
    }
}

void led_set(led_id_t id, uint8_t on)
{
    if (id >= LED_COUNT) return;

    led_state[id] = on ? 1u : 0u;
    HAL_GPIO_WritePin(led_pins[id].port, led_pins[id].pin, LED_LEVEL(led_state[id]));
}

void led_toggle(led_id_t id)
{
    if (id >= LED_COUNT) return;

    led_set(id, (uint8_t)!led_state[id]);
}

uint8_t led_get(led_id_t id)
{
    return (id < LED_COUNT) ? led_state[id] : 0u;
}

void led_blink(led_id_t id, uint16_t ms)
{
    if (id >= LED_COUNT) return;

    led_set(id, 1);
    led_timer[id] = ms;
}

void led_tick(void)
{
    for (uint8_t i = 0; i < LED_COUNT; i++)
    {
        if (led_timer[i] > 0)
        {
            led_timer[i]--;
            if (led_timer[i] == 0)
            {
                led_set((led_id_t)i, 0);
            }
        }
    }
}
