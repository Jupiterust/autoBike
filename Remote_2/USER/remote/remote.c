/*
 * remote.c - 见 remote.h 与 REMOTE_PROTOCOL.md。
 */

#include "remote.h"
#include "bt.h"
#include "main.h"

/* 按键 -> 协议 id。0 表示这个键不发东西。顺序必须和 key_id_t 一致。
 * 协议明确禁止发 '0'~'7'（那是 VOFA+ 上位机调参用的），这张表里也不该出现。 */
static const char key_id_map[KEY_COUNT] =
{
    'U',    /* KEY_L_UP    左上  加速 / 菜单上一项 */
    'L',    /* KEY_L_LEFT  左左  左转 / 值减一步 */
    'R',    /* KEY_L_RIGHT 左右  右转 / 值加一步 */
    'D',    /* KEY_L_DOWN  左下  减速 / 菜单下一项 */
    'M',    /* KEY_R_UP    右上  模式切换 */
    'S',    /* KEY_R_LEFT  右左  切换上位机 / 遥控控制源（单次发送） */
    0,      /* KEY_R_RIGHT 右右  空闲 */
    'K'     /* KEY_R_DOWN  右下  OK / 停车回中 */
};

static uint32_t next_repeat_ms[KEY_COUNT];
static uint32_t last_hb_ms;

/* 连发的"武装"标志：只有收到 DOWN 事件才置 1，收到 UP 事件才清 0。
 *
 * 这个标志是防重复发送的关键。之前 remote_task() 直接拿 key_is_down() 的电平
 * 做连发判断，而电平是 1 ms 中断里更新的、事件是主循环里取的，两者不同步：
 * 按下如果正好落在 key_poll() 排空之后、remote_task() 之前，remote_task() 会
 * 看到电平已经变 1、而 next_repeat_ms[] 还停在上一次按键留下的过期时间戳上，
 * 于是立刻发一帧；下一轮 key_poll() 取到 DOWN 事件又发一帧——一次按下发两次。
 *
 * 现在连发只能被 DOWN 事件武装，电平只用来提前结束连发，就不可能抢在事件前面。 */
static uint8_t  repeat_armed[KEY_COUNT];

static uint32_t stat_frames;
static uint32_t stat_btn_frames;    /* 只统计按键帧，不含心跳 */
static uint32_t stat_heartbeats;
static char     stat_last_id;

/* 发一帧 "(id,)"。
 *
 * 逗号不能省：整车的解析器在收集 id 的状态里不把 ')' 当结束符，"(U)" 会让
 * id 变成 "U)" 并把状态机卡住，那一帧被静默丢掉（REMOTE_PROTOCOL.md 2.3）。 */
static void remote_send_id(char id)
{
    uint8_t frame[4];

    if (id == 0) return;

    frame[0] = '(';
    frame[1] = (uint8_t)id;
    frame[2] = ',';
    frame[3] = ')';

    if (bt_write(frame, 4))
    {
        stat_frames++;
        stat_last_id = id;
    }
}

void remote_init(void)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        next_repeat_ms[i] = 0;
        repeat_armed[i]   = 0;
    }

    /* 上电就开始发心跳，不等第一次按键。 */
    last_hb_ms      = now;
    stat_frames     = 0;
    stat_btn_frames = 0;
    stat_heartbeats = 0;
    stat_last_id    = 0;
}

void remote_on_key_event(const key_event_t *ev)
{
    if (ev->id >= KEY_COUNT) return;

    /* 只用"按下"和"松开"的边沿。key.c 自己那套 LONG / REPEAT 在这里不合用：
     * 连发时序由协议层控制，控制源切换只在 DOWN 时发送一次。 */
    if (ev->type == KEY_EV_DOWN)
    {
        if (key_id_map[ev->id] != 0)
        {
            remote_send_id(key_id_map[ev->id]);
            stat_btn_frames++;
        }
        /* 首次连发要等 REMOTE_REPEAT_DELAY_MS，之后才是 REMOTE_REPEAT_MS 的节奏。 */
        next_repeat_ms[ev->id] = HAL_GetTick() + REMOTE_REPEAT_DELAY_MS;
        /* S 是切换命令，长按连发会让主控反复切换控制源。 */
        repeat_armed[ev->id]   = (ev->id != KEY_R_LEFT);
    }
    else if (ev->type == KEY_EV_UP)
    {
        repeat_armed[ev->id] = 0;
    }
}

void remote_task(void)
{
    uint32_t now = HAL_GetTick();

    /* 1. 按住重发。
     *
     * 两个条件缺一不可：
     *   repeat_armed[i] —— 这一次按下的 DOWN 事件已经被处理过了。没有它，
     *                      电平会抢在事件前面多发一帧（见上面的注释）。
     *   key_is_down(i)  —— 松手立刻停，不用等 UP 事件被取走。 */
    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        if (key_id_map[i] == 0) continue;
        if (!repeat_armed[i])   continue;

        if (!key_is_down((key_id_t)i))
        {
            repeat_armed[i] = 0;
            continue;
        }

        if ((int32_t)(now - next_repeat_ms[i]) >= 0)
        {
            remote_send_id(key_id_map[i]);
            stat_btn_frames++;
            next_repeat_ms[i] = now + REMOTE_REPEAT_MS;
        }
    }

    /* 2. 心跳，无条件。整车 500 ms 收不到任何帧就把动力切了。 */
    if ((uint32_t)(now - last_hb_ms) >= REMOTE_HEARTBEAT_MS)
    {
        last_hb_ms = now;
        remote_send_id('H');
        stat_heartbeats++;
    }
}

char remote_id_for_key(key_id_t id)
{
    return (id < KEY_COUNT) ? key_id_map[id] : 0;
}

uint32_t remote_frames_sent(void)  { return stat_frames; }
uint32_t remote_btn_frames(void)   { return stat_btn_frames; }
uint32_t remote_heartbeats(void)   { return stat_heartbeats; }
char     remote_last_id(void)      { return stat_last_id; }
