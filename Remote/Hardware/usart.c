#include "usart.h"

/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1

#if (__ARMCC_VERSION >= 6010050)            /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");  /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}


/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* MDK下需要重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);     /* 等待上一个字符发送完成 */

    USART1->DR = (uint8_t)ch;             /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}
#endif
/******************************************************************************************/


uint8_t Buf_tx[TX_LEN];
uint8_t Buf_rx[RX_LEN];

/**
 * @brief       串口X初始化函数
 * @param       sclk: 串口X的时钟源频率(单位: MHz)
 *              串口1 的时钟源来自: PCLK2 = 72Mhz
 *              串口2 - 5 的时钟源来自: PCLK1 = 36Mhz
 * @note        注意: 必须设置正确的sclk, 否则串口波特率就会设置异常.
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @retval      无
 */
void usart_init(uint32_t sclk, uint32_t baudrate)
{
    uint32_t temp;
    /* IO 及 时钟配置 */
    RCC->APB2ENR |= 1 << 2; /* 使能串口TX脚时钟 */    /* 使能串口RX脚时钟 */
    RCC->APB2ENR |= RCC->APB2ENR |= 1 << 14;     /* 使能串口时钟 */

    sys_gpio_set(GPIOA, SYS_GPIO_PIN9,
                 SYS_GPIO_MODE_AF, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_HIGH, SYS_GPIO_PUPD_PU);   /* 串口TX脚 模式设置 */

    sys_gpio_set(GPIOA, SYS_GPIO_PIN10,
                 SYS_GPIO_MODE_IN, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_HIGH, SYS_GPIO_PUPD_PU);   /* 串口RX脚 必须设置成输入模式 */

    temp = (sclk * 1000000 + baudrate / 2) / baudrate;  /* 得到BRR, 采用四舍五入计算 */
    /* 波特率设置 */
    USART1->BRR = temp;       /* 波特率设置 */
    USART1->CR1 = 0;          /* 清零CR1寄存器 */
    USART1->CR1 |= 0 << 12;   /* M = 0, 1个起始位, 8个数据位, n个停止位(由USART_CR2 STOP[1:0]指定, 默认是0, 表示1个停止位) */
    USART1->CR1 |= 1 << 3;    /* TE = 1, 串口发送使能 */

    /* 使能接收中断 */
    USART1->CR1 |= 1 << 2;    /* RE = 1, 串口接收使能 */
    USART1->CR1 |= 1 << 5;    /* RXNEIE = 1, 接收缓冲区非空中断使能 */
    sys_nvic_init(3, 3, USART1_IRQn, 2); /* 组2，最低优先级 */
    
    //USART1->CR3|=1<<6;         /* 使能串口1的DMA发送 */
    USART1->CR1 |= 1 << 13;   /* UE = 1, 串口使能 */
}



void Serial_SendByte(USART_TypeDef * UART,uint8_t Byte)
{
	while((UART->SR&0X40)==0);//循环发送,直到发送完毕   
	UART->DR =Byte;  
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(USART_TypeDef * UART,uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(UART,Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

void Serial_SendString(USART_TypeDef * UART,char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(UART,String[i]);
	}
}

void Serial1_SendArray(USART_TypeDef * UART,uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(UART,Array[i]);
	}
}



/**
 * @brief       串口X中断服务函数
 * @param       无
 * @retval      无
 */
void USART1_IRQHandler(void)
{
    uint8_t rxdata;
    if (USART1->SR & (1 << 5))                /* 接收到数据 */
    {
        rxdata = USART1->DR;
			  data_analysis(rxdata);
    }


}

