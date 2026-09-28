#ifndef __UART_H
#define __UART_H 


#include "A_include.h" 


#define USART_RX_LEN1  			1  	//定义最大接收字节数 
#define USART_RX_LEN2  			1  	//定义最大接收字节数

extern uint8_t USART_RX_BUF1[USART_RX_LEN1],
USART_RX_BUF2[USART_RX_LEN2];     //接收缓冲




void uart_printf(const char *format,...);

/* Links whose RX is recovered by HAL_UART_ErrorCallback(); see uart.c. */
#define UART_ERR_IMU      0   /* huart8, CH100 - also drives balance()  */
#define UART_ERR_REMOTE   1   /* huart7, handset / VOFA                 */
#define UART_ERR_UPPER    2   /* huart2                                 */
#define UART_ERR_BLUE     3   /* huart6, bluetooth ValuePack            */
#define UART_ERR_LINKS    4

extern volatile uint32_t uart_err_cnt[UART_ERR_LINKS];

void uart_rx_watchdog(void);



#endif	   
















