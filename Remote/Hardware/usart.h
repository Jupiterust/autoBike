#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "sys.h"


#define TX_LEN 1
#define RX_LEN 1
extern uint8_t Buf_tx[TX_LEN];
extern uint8_t Buf_rx[RX_LEN];

void usart_init(uint32_t pclk2, uint32_t bound);/* 串口初始化函数 */

void Serial_SendByte(USART_TypeDef * UART,uint8_t Byte);
uint32_t Serial_Pow(uint32_t X, uint32_t Y);
void Serial_SendNumber(USART_TypeDef * UART,uint32_t Number, uint8_t Length);
void Serial_SendString(USART_TypeDef * UART,char *String);
void Serial1_SendArray(USART_TypeDef * UART,uint8_t *Array, uint16_t Length);



#endif  
















