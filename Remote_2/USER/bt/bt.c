/*
 * bt.c - 见 bt.h。
 */

#include "bt.h"
#include "main.h"
#include "usart.h"

static uint8_t           tx_ring[BT_TX_BUF_SIZE];
static volatile uint16_t tx_head;        /* 写入位置，只由 bt_write() 动 */
static volatile uint16_t tx_tail;        /* 发送位置，只由发送完成动 */
static volatile uint8_t  tx_busy;
static volatile uint16_t tx_chunk;       /* 本次 DMA 交出去的长度 */

static volatile uint32_t stat_tx_bytes;
static volatile uint32_t stat_dropped;

static void bt_kick(void);

void bt_init(void)
{
    /* USART1 本身由 CubeMX 的 MX_USART1_UART_Init() 初始化（115200 8N1，
     * TX 挂 DMA1_Channel4），这里只清软件状态。 */
    tx_head = 0;
    tx_tail = 0;
    tx_busy = 0;
    tx_chunk = 0;
    stat_tx_bytes = 0;
    stat_dropped  = 0;
}

static uint16_t tx_free(void)
{
    uint16_t used = (uint16_t)((tx_head - tx_tail) & (BT_TX_BUF_SIZE - 1u));

    /* 环形缓冲留一个空位区分"满"和"空"。 */
    return (uint16_t)(BT_TX_BUF_SIZE - 1u - used);
}

uint8_t bt_write(const uint8_t *data, uint16_t len)
{
    if (len == 0) return 1;

    if (len > tx_free())
    {
        stat_dropped++;
        return 0;                       /* 整帧丢弃，绝不写半帧 */
    }

    for (uint16_t i = 0; i < len; i++)
    {
        tx_ring[tx_head] = data[i];
        tx_head = (uint16_t)((tx_head + 1u) & (BT_TX_BUF_SIZE - 1u));
    }

    bt_kick();
    return 1;
}

uint8_t bt_puts(const char *s)
{
    uint16_t n = 0;

    while (s[n] != '\0') n++;

    return bt_write((const uint8_t *)s, n);
}

/* 启动下一段 DMA。可能从主循环调用（bt_write 之后），也可能从发送完成
 * 中断里调用，所以对 tx_busy 的检查和置位必须是原子的。 */
static void bt_kick(void)
{
    uint32_t primask;
    uint16_t tail, len;

    primask = __get_PRIMASK();
    __disable_irq();

    if (tx_busy || tx_head == tx_tail)
    {
        __set_PRIMASK(primask);
        return;
    }

    tail = tx_tail;

    /* DMA 发不了回绕的两段，所以一次只发到缓冲末尾为止，剩下的下一轮再发。 */
    len = (tx_head > tail) ? (uint16_t)(tx_head - tail)
                           : (uint16_t)(BT_TX_BUF_SIZE - tail);

    tx_busy  = 1;
    tx_chunk = len;

    __set_PRIMASK(primask);

    if (HAL_UART_Transmit_DMA(&huart1, &tx_ring[tail], len) != HAL_OK)
    {
        /* 串口还没空出来（上一次的 TC 还没到）。清掉 busy，等下一次
         * bt_write() 或者发送完成回调再试；不清的话会永久卡死。 */
        tx_busy  = 0;
        tx_chunk = 0;
    }
}

/* USART1 发送完成。HAL 是弱定义，这里覆盖它。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    tx_tail = (uint16_t)((tx_tail + tx_chunk) & (BT_TX_BUF_SIZE - 1u));
    stat_tx_bytes += tx_chunk;
    tx_chunk = 0;
    tx_busy  = 0;

    bt_kick();
}

uint16_t bt_pending(void)
{
    return (uint16_t)((tx_head - tx_tail) & (BT_TX_BUF_SIZE - 1u));
}

uint32_t bt_tx_bytes(void)
{
    return stat_tx_bytes;
}

uint32_t bt_dropped(void)
{
    return stat_dropped;
}
