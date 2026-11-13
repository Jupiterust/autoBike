/*
 * key.h - 8 个手柄按键的扫描与事件。
 *
 * 引脚（全部上拉输入，按下为低电平）：
 *
 *      左边   上 PB9    左 PB8    右 PB7    下 PB6
 *      右边   上 PA11   左 PA15   右 PB3    下 PA12
 *
 * 这 8 个脚在 Remote_2.ioc 里已经配成 GPIO_MODE_INPUT + PULLUP，key_init()
 * 不重复配置，只做软件初始化。PB3/PA15 靠 HAL_MspInit() 里的
 * __HAL_AFIO_REMAP_SWJ_NOJTAG() 从 JTAG 手里释放出来。
 *
 * key_tick() 必须放在 1 ms 定时中断里（TIM1 更新中断）。事件走一个环形队列，
 * 主循环用 key_poll() 取，所以主循环慢一点也不会丢按键。
 */

#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

typedef enum
{
    KEY_L_UP = 0,
    KEY_L_LEFT,
    KEY_L_RIGHT,
    KEY_L_DOWN,
    KEY_R_UP,
    KEY_R_LEFT,
    KEY_R_RIGHT,
    KEY_R_DOWN,
    KEY_COUNT
} key_id_t;

typedef enum
{
    KEY_EV_DOWN = 0,    /* 消抖后确认按下 */
    KEY_EV_REPEAT,      /* 长按后按 KEY_REPEAT_MS 的间隔连发 */
    KEY_EV_LONG,        /* 按住到 KEY_LONG_MS，只发一次 */
    KEY_EV_UP           /* 松开 */
} key_event_type_t;

typedef struct
{
    uint8_t  id;        /* key_id_t */
    uint8_t  type;      /* key_event_type_t */
    uint16_t hold_ms;   /* 事件发生时该键已按住多久 */
} key_event_t;

/* 消抖时间，按下和松开分开。
 *
 * 实测 10 次点按正好产生 20 个事件（10 个 DOWN + 10 个 UP），说明这块板子的
 * 触点其实不怎么抖，不需要很长的消抖。
 *
 * 松开方向**故意不敢放太长**：认松开越慢，key_is_down() 就多撑越久，等于把
 * "按住时长"凭空拉长，反而更容易触发 remote.c 的连发。曾经设成 50 ms，正是
 * 帮倒忙。25 ms 留一点余量，又不至于把一次点按撑进连发的门槛里。
 *
 * 如果以后真的换了会抖的按键（表现是一次点按产生 4 个事件而不是 2 个），
 * 再加大 KEY_DEBOUNCE_RELEASE_MS。 */
#define KEY_DEBOUNCE_PRESS_MS    15u
#define KEY_DEBOUNCE_RELEASE_MS  25u

/* 判定长按的时间。 */
#define KEY_LONG_MS     600u

/* 长按之后连发的间隔。100 ms = 10 Hz，正好是整车 ui.h 期望手柄按住时
 * 重复发同一个 id 的节奏，后面接蓝牙那一层可以直接用。 */
#define KEY_REPEAT_MS   100u

void        key_init(void);
void        key_tick(void);                 /* 每 1 ms 调用一次 */
uint8_t     key_poll(key_event_t *ev);      /* 取出一个事件，队列空返回 0 */
uint8_t     key_is_down(key_id_t id);       /* 当前是否按下（已消抖） */
uint16_t    key_hold_ms(key_id_t id);       /* 已按住多久，松开时为 0 */
uint8_t     key_any_down(void);
const char *key_name(key_id_t id);          /* "LU" / "RD" ... */
const char *key_event_name(key_event_type_t t);

#endif /* __KEY_H */
