/*
 * led.h - 板上两颗 LED：LED0 = PB4，LED1 = PB5。
 *
 * 这两个脚不在 Remote_2.ioc 里，由 led_init() 自己配成推挽输出。
 * PB4 默认是 JTAG 的 NJTRST，靠 HAL_MspInit() 里已有的
 * __HAL_AFIO_REMAP_SWJ_NOJTAG() 释放出来；不做这个重映射 PB4 推不动。
 */

#ifndef __LED_H
#define __LED_H

#include <stdint.h>

typedef enum
{
    LED0 = 0,       /* PB4 */
    LED1,           /* PB5 */
    LED_COUNT
} led_id_t;

/* 灯的接法：0 = 低电平点亮（引脚灌电流，常见接法），1 = 高电平点亮。
 * 上电后两颗灯应该都是灭的，如果反过来常亮，把这里改成 1。 */
#define LED_ACTIVE_HIGH  0

void led_init(void);
void led_set(led_id_t id, uint8_t on);
void led_toggle(led_id_t id);
uint8_t led_get(led_id_t id);

/* 让某颗灯亮 ms 毫秒后自动灭。需要 led_tick() 每 1 ms 走一次。 */
void led_blink(led_id_t id, uint16_t ms);
void led_tick(void);

#endif /* __LED_H */
