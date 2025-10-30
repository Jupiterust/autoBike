#ifndef __UART_H
#define __UART_H 


#include "A_include.h" 


#define USART_RX_LEN1  			1  	//定义最大接收字节数 
#define USART_RX_LEN2  			1  	//定义最大接收字节数

extern uint8_t USART_RX_BUF1[USART_RX_LEN1],
USART_RX_BUF2[USART_RX_LEN2];     //接收缓冲




void uart_printf(const char *format,...);



#endif	   
















