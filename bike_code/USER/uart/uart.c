 #include "uart.h"	  
 
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)                              
struct __FILE 
{ 
	int handle; 
}; 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
	USART6->DR = (uint8_t) ch;      
	return ch;
}
#endif 



uint8_t USART_RX_BUF1[USART_RX_LEN1],
USART_RX_BUF2[USART_RX_LEN2];     //接收缓冲


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == (&huart8))
    {
        CH100_Rec(); 
				balance();			
    }
    else if (huart == (&huart2))
    {
				if(upper_Flag == 1)Upper(USART_RX_BUF1[0]);
        HAL_UART_Receive_IT(&huart2, (uint8_t *)USART_RX_BUF1, USART_RX_LEN1);
    }
   else if (huart == (&huart7))
    {
        //HAL_UART_Transmit_DMA(&huart7,USART_RX_BUF1,USART_RX_LEN1);

			
		#ifdef Button_Only
        //vofa
		vofa_get(USART_RX_BUF2[0]);
        #else
		//原串口处理
		//data_analysis(USART_RX_BUF2[0]);
        #endif
		
		HAL_UART_Receive_IT(&huart7, (uint8_t *)USART_RX_BUF2, USART_RX_LEN2);
    }


}


/* Per-link error counts. Sticky, never reset; watch them on the display's
   diagnostic row to tell "the radio dropped" from "the radio is fine". */
volatile uint32_t uart_err_cnt[UART_ERR_LINKS] = {0, 0, 0, 0};

/* Without this, one overrun kills a link for good.
 *
 * HAL_UART_IRQHandler() treats ORE - and, in DMA mode, any error at all - as
 * blocking: it calls UART_EndRxTransfer(), which clears RXNEIE and puts
 * RxState back to READY. Reception is aborted. This project only ever re-arms
 * reception inside HAL_UART_RxCpltCallback(), which by then can never fire
 * again, so the link stays dead until the board is reset. That is the "have
 * to power-cycle before the remote works again" symptom, and the radio is not
 * at fault.
 *
 * Overruns are easy to provoke here. UART7 sits at NVIC preemption priority 2
 * while UART8 - the IMU, which runs the entire 2.5 ms control loop inside its
 * ISR - sits at 0 and preempts it, so a long control tick can push the UART7
 * ISR past one byte time (86.8 us at 115200). A Bluetooth module dropping or
 * re-associating also puts noise and framing errors on the line.
 *
 * The same trap applies to UART8 itself, where it is far worse: losing that
 * stream stops balance() being called at all.
 *
 * Only re-arm when the transfer really was aborted. A non-blocking error
 * (FE/NE/PE without ORE, in interrupt mode) leaves reception running, and
 * re-arming on top of it would corrupt the handle's state. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Reading SR then DR clears PE/FE/NE/ORE together on F4. */
    __HAL_UART_CLEAR_PEFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    if (huart->RxState != HAL_UART_STATE_READY)
        return;                 /* transfer survived; nothing to restart */

    if (huart == &huart8)
    {
        uart_err_cnt[UART_ERR_IMU]++;
        HAL_UART_Receive_DMA(&huart8, (uint8_t *)CH100_buf, CH100_DATA_LEN);
    }
    else if (huart == &huart7)
    {
        uart_err_cnt[UART_ERR_REMOTE]++;
        HAL_UART_Receive_IT(&huart7, (uint8_t *)USART_RX_BUF2, USART_RX_LEN2);
    }
    else if (huart == &huart2)
    {
        uart_err_cnt[UART_ERR_UPPER]++;
        HAL_UART_Receive_IT(&huart2, (uint8_t *)USART_RX_BUF1, USART_RX_LEN1);
    }
    else if (huart == &huart6)
    {
        uart_err_cnt[UART_ERR_BLUE]++;
        HAL_UART_Receive_DMA(&huart6, (uint8_t *)vp_rxbuff, VALUEPACK_BUFFER_SIZE);
    }
}

/* Belt and braces for whatever the error callback does not catch: if a link
 * is sitting idle with no reception armed, arm it. Called from the main loop,
 * so it also recovers from a state entered before ui_task() started. */
void uart_rx_watchdog(void)
{
    if (huart8.RxState == HAL_UART_STATE_READY)
        HAL_UART_Receive_DMA(&huart8, (uint8_t *)CH100_buf, CH100_DATA_LEN);
    if (huart7.RxState == HAL_UART_STATE_READY)
        HAL_UART_Receive_IT(&huart7, (uint8_t *)USART_RX_BUF2, USART_RX_LEN2);
    if (huart2.RxState == HAL_UART_STATE_READY)
        HAL_UART_Receive_IT(&huart2, (uint8_t *)USART_RX_BUF1, USART_RX_LEN1);
    if (huart6.RxState == HAL_UART_STATE_READY)
        HAL_UART_Receive_DMA(&huart6, (uint8_t *)vp_rxbuff, VALUEPACK_BUFFER_SIZE);
}




void uart_printf(const char *format,...)
{
	uint32_t length;
  static uint8_t _dbg_TXBuff[255];
	va_list args;
	va_start(args, format);
	length = vsnprintf((char*)_dbg_TXBuff, sizeof(_dbg_TXBuff)+1, (char*)format, args);
	va_end(args);
    
	HAL_UART_Transmit_DMA(&huart7,_dbg_TXBuff,length);
}


