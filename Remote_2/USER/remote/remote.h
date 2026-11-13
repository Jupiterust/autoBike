/*
 * remote.h - 整车遥控协议，实现 REMOTE_PROTOCOL.md。
 *
 * 帧格式：ASCII  "(" id "," value ")"    逗号是必须的，没有结束符。
 * 只发不收，没有 ACK / 重传 / 序号 / 校验——协议的可靠性机制就是"一直重发"。
 *
 * 手柄发的是**物理按键**，不是动作。整车根据自己当前处在行驶模式还是菜单模式
 * 决定一个键是什么意思，所以手柄不保存任何模式状态，也就不可能和整车失步。
 *
 * 按键 -> 协议 id 的对应表（按 v1 遥控器的语义搬过来的）：
 *
 *      左边 上 PB9  -> 'U'   加速 / 菜单上一项
 *      左边 左 PB8  -> 'L'   左转 / 值减一步
 *      左边 右 PB7  -> 'R'   右转 / 值加一步
 *      左边 下 PB6  -> 'D'   减速 / 菜单下一项
 *      右边 上 PA11 -> 'M'   模式切换（v1 这个键就是自动/手动切换）
 *      右边 左 PA15 -> 'S'   切换上位机 / 遥控控制源（每次按下只发一次）
 *      右边 右 PB3  ->  -    空闲
 *      右边 下 PA12 -> 'K'   停车回中 / 执行菜单项（v1 这个键是"中值"，语义一致）
 *
 * 左边是 U/D/L/R 方向键；右手是 M（模式）、S（控制源）、K（OK/停车）。
 * 控制源状态由主控维护，手柄只发送切换请求；右右暂时空闲。
 *
 * 时序（REMOTE_PROTOCOL.md 第 4 节）：
 *   - 按下瞬间立刻发一帧；
 *   - S 只在消抖后的按下沿发送 (S,)，长按不连发，松开后再按可再次切换；
 *   - 按住超过 400 ms 之后，每 100 ms 重发同一个 id（整车侧"一帧 = 一步"，
 *     没有长按概念）。点按一下只发一帧；
 *   - 无论有没有按键，每 150 ms 发一个 "(H,)" 心跳；
 *   - 整车 500 ms 收不到任何帧就切断动力（平衡环不动），所以心跳不能停。
 */

#ifndef __REMOTE_H
#define __REMOTE_H

#include <stdint.h>
#include "key.h"

/* 开始连发之前必须按住多久。
 *
 * 没有这个延迟时，一次正常点按（人手大约 60~250 ms）只要超过 100 ms 就会在
 * DOWN 之外再补一帧连发，整车上就变成一次点按走了两步。实测：松开消抖 20 ms 时
 * 按 10 下发 13 帧，把松开消抖加到 50 ms 后变成 20 帧——每次都多一帧，因为消抖
 * 把每次点按的"按住时长"都撑过了 100 ms。
 *
 * 协议只要求"按下发一帧、按住约 10 Hz 重发"，没规定连发从第几毫秒开始，所以
 * 加首次延迟是合规的。400 ms 比正常点按长得多，又不至于让有意长按觉得迟钝。 */
#define REMOTE_REPEAT_DELAY_MS  400u

/* 首帧连发之后的重发周期。整车 UI_SERVO_STEP = 5、M1_Ctl 步进 0.1，10 Hz 下
 * 大约是转向 1.6 s 打满、速度 1.0/s。 */
#define REMOTE_REPEAT_MS    100u

/* 心跳周期。协议要求 100~200 ms，取中间值留余量。 */
#define REMOTE_HEARTBEAT_MS 150u

void remote_init(void);

/* 主循环里把 key_poll() 取到的每个事件转给它，用来抓"按下"的边沿。 */
void remote_on_key_event(const key_event_t *ev);

/* 主循环里无条件反复调用：负责按住重发和心跳。 */
void remote_task(void);

/* 给 OLED 显示用。 */
char     remote_id_for_key(key_id_t id);   /* 该键发什么 id，0 = 未分配 */
uint32_t remote_frames_sent(void);

/* 只数按键帧，不含心跳。按 10 次就该正好加 10——用来验证"一次按下发一帧"。 */
uint32_t remote_btn_frames(void);
uint32_t remote_heartbeats(void);
char     remote_last_id(void);

#endif /* __REMOTE_H */
