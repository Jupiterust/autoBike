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


