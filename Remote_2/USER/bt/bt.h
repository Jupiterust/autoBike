/*
 * bt.h - 蓝牙串口的字节管道：USART1 (PA9 = TX, PA10 = RX)，115200 8N1。
 *
 * 只负责"把字节送出去"，不认识任何协议——协议在 USER/remote 里。
 *
 * 发送走环形缓冲 + DMA1_Channel4（.ioc 里已经给 USART1_TX 配好了），所以
 * bt_write() 不会阻塞主循环。整屏刷 OLED 要十几毫秒，如果发送是阻塞的，
 * 心跳就会被 OLED 顶到抖动。
 *
 * 按 REMOTE_PROTOCOL.md 第 1.2 节，手柄是**只发不收**的，整车不会回任何东西，
 * 所以这里没有接收通路。蓝牙模块自己吐的状态文本丢掉即可。
 */

#ifndef __BT_H
#define __BT_H

#include <stdint.h>

/* 必须是 2 的幂。一帧 4~5 字节，256 字节足够缓冲几十帧。 */
#define BT_TX_BUF_SIZE  256u

void     bt_init(void);

/* 把 len 个字节排进发送队列。返回 1 = 已排入，0 = 缓冲满、整帧丢弃。
 * 整帧丢弃而不是写半帧：半帧会让整车的解析器收到一个残缺的 "(" 开头，
 * 虽然下一个 "(" 会重新同步，但白白多毁一帧。 */
uint8_t  bt_write(const uint8_t *data, uint16_t len);
uint8_t  bt_puts(const char *s);

uint16_t bt_pending(void);      /* 队列里还没发出去的字节数 */
uint32_t bt_tx_bytes(void);     /* 累计发出的字节数 */
uint32_t bt_dropped(void);      /* 因缓冲满被丢掉的帧数 */

#endif /* __BT_H */
