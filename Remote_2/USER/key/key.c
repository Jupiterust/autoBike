/*
 * key.c - 按键扫描，跑在 1 ms 定时中断里。见 key.h 的引脚表。
 *
 * 每个键一个独立的小状态机，互不干扰：先做电平计数消抖，确认按下后开始
 * 计时，到 KEY_LONG_MS 发一次 KEY_EV_LONG，之后每 KEY_REPEAT_MS 发一次
 * KEY_EV_REPEAT，松开发 KEY_EV_UP。
 *
 * 事件写进环形队列由主循环取走，而不是留一个"标志位数组"给主循环轮询。
 * v1 遥控器那种 Key_Flag[] 的写法，主循环一慢就会把两次按下并成一次，
 * 而且清标志和置标志分处中断内外，天然有竞态。
 */

#include "key.h"
#include "main.h"

/* --- 引脚表 -------------------------------------------------------------- */

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    const char   *name;
} key_pin_t;

/* 顺序必须和 key_id_t 一致。 */
static const key_pin_t key_pins[KEY_COUNT] =
{
    { GPIOB, GPIO_PIN_9,  "LU" },   /* 左 上 */
    { GPIOB, GPIO_PIN_8,  "LL" },   /* 左 左 */
    { GPIOB, GPIO_PIN_7,  "LR" },   /* 左 右 */
    { GPIOB, GPIO_PIN_6,  "LD" },   /* 左 下 */
    { GPIOA, GPIO_PIN_11, "RU" },   /* 右 上 */
    { GPIOA, GPIO_PIN_15, "RL" },   /* 右 左 */
    { GPIOB, GPIO_PIN_3,  "RR" },   /* 右 右 */
    { GPIOA, GPIO_PIN_12, "RD" }    /* 右 下 */
};

/* 上拉输入，按下把引脚拉到地。 */
#define KEY_PRESSED_LEVEL   GPIO_PIN_RESET

/* --- 每个键的状态 -------------------------------------------------------- */

typedef struct
{
    uint8_t  stable;        /* 消抖后的电平：1 = 按下 */
    uint8_t  raw_last;      /* 上一次采样的原始电平 */
    uint8_t  debounce;      /* 原始电平保持不变的毫秒数 */
    uint8_t  long_sent;     /* KEY_EV_LONG 是否已经发过 */
    uint16_t hold_ms;       /* 按住的毫秒数 */
    uint16_t next_repeat;   /* 下一次连发的 hold_ms 阈值 */
} key_state_t;

static volatile key_state_t keys[KEY_COUNT];

/* --- 事件队列（单生产者中断 / 单消费者主循环） ---------------------------- */

#define KEY_QUEUE_LEN   16u             /* 必须是 2 的幂 */

static volatile key_event_t queue[KEY_QUEUE_LEN];
static volatile uint8_t     q_head;     /* 只由 key_tick() 写 */
static volatile uint8_t     q_tail;     /* 只由 key_poll() 写 */

static void key_push(uint8_t id, uint8_t type, uint16_t hold_ms)
{
    uint8_t next = (uint8_t)((q_head + 1u) & (KEY_QUEUE_LEN - 1u));

    if (next == q_tail)
    {
        return;         /* 队列满：丢掉最新的一个，宁可少一次也不要覆盖旧的 */
    }

    queue[q_head].id      = id;
    queue[q_head].type    = type;
    queue[q_head].hold_ms = hold_ms;
    q_head = next;
}

uint8_t key_poll(key_event_t *ev)
{
    if (q_tail == q_head)
    {
        return 0;
    }

    ev->id      = queue[q_tail].id;
    ev->type    = queue[q_tail].type;
    ev->hold_ms = queue[q_tail].hold_ms;
    q_tail = (uint8_t)((q_tail + 1u) & (KEY_QUEUE_LEN - 1u));

    return 1;
}

/* --- 接口 ---------------------------------------------------------------- */

void key_init(void)
{
    /* 8 个引脚的 GPIO 模式由 CubeMX 生成的 MX_GPIO_Init() 配置成上拉输入，
     * 这里不再重复配置，免得和 .ioc 出现两份互相矛盾的设置。 */
    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        keys[i].stable      = 0;
        keys[i].raw_last    = 0;
        keys[i].debounce    = 0;
        keys[i].long_sent   = 0;
        keys[i].hold_ms     = 0;
        keys[i].next_repeat = 0;
    }

    q_head = 0;
    q_tail = 0;
}

void key_tick(void)
{
    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        uint8_t raw = (HAL_GPIO_ReadPin(key_pins[i].port, key_pins[i].pin)
                       == KEY_PRESSED_LEVEL) ? 1u : 0u;

        /* 1. 消抖：原始电平必须连续保持够久才认，按下和松开的门槛不一样。
         *    任何一次跳变都把计数清零，所以要求的是"连续稳定"，不是"累计"。 */
        uint8_t need = raw ? KEY_DEBOUNCE_PRESS_MS : KEY_DEBOUNCE_RELEASE_MS;

        if (raw != keys[i].raw_last)
        {
            keys[i].raw_last = raw;
            keys[i].debounce = 0;
        }
        else if (keys[i].debounce < KEY_DEBOUNCE_RELEASE_MS)
        {
            keys[i].debounce++;     /* 上限取两者中大的那个 */
        }

        if (keys[i].debounce >= need && raw != keys[i].stable)
        {
            keys[i].stable = raw;

            if (raw)
            {
                keys[i].hold_ms     = 0;
                keys[i].long_sent   = 0;
                keys[i].next_repeat = KEY_LONG_MS + KEY_REPEAT_MS;
                key_push(i, KEY_EV_DOWN, 0);
            }
            else
            {
                key_push(i, KEY_EV_UP, keys[i].hold_ms);
                keys[i].hold_ms = 0;
            }
        }

        /* 2. 按住期间的计时、长按和连发。 */
        if (keys[i].stable)
        {
            if (keys[i].hold_ms < 0xFFFFu)
            {
                keys[i].hold_ms++;
            }

            if (!keys[i].long_sent && keys[i].hold_ms >= KEY_LONG_MS)
            {
                keys[i].long_sent = 1;
                key_push(i, KEY_EV_LONG, keys[i].hold_ms);
            }

            if (keys[i].long_sent && keys[i].hold_ms >= keys[i].next_repeat)
            {
                keys[i].next_repeat = (uint16_t)(keys[i].hold_ms + KEY_REPEAT_MS);
                key_push(i, KEY_EV_REPEAT, keys[i].hold_ms);
            }
        }
    }
}

uint8_t key_is_down(key_id_t id)
{
    return (id < KEY_COUNT) ? keys[id].stable : 0u;
}

uint16_t key_hold_ms(key_id_t id)
{
    return (id < KEY_COUNT) ? keys[id].hold_ms : 0u;
}

uint8_t key_any_down(void)
{
    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        if (keys[i].stable) return 1;
    }
    return 0;
}

const char *key_name(key_id_t id)
{
    return (id < KEY_COUNT) ? key_pins[id].name : "??";
}

const char *key_event_name(key_event_type_t t)
{
    switch (t)
    {
        case KEY_EV_DOWN:   return "DOWN";
        case KEY_EV_REPEAT: return "RPT ";
        case KEY_EV_LONG:   return "LONG";
        case KEY_EV_UP:     return "UP  ";
        default:            return "??  ";
    }
}
