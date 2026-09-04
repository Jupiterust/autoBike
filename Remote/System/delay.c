#include "sys.h"
#include "delay.h"


static uint32_t g_fac_us = 0;       /* us延时倍乘数 */



/**
 * @brief     初始化延迟函数
 * @param     sysclk: 系统时钟频率, 即CPU频率(HCLK), 216Mhz
 * @retval    无
 */
void delay_init(uint16_t sysclk)
{
    SysTick->CTRL |= (1 << 2);              /* SYSTICK使用内部时钟源,频率为HCLK*/
    g_fac_us = sysclk;                      /* 不论是否使用OS,g_fac_us都需要使用 */
    SysTick->CTRL |= 1 << 0;                /* 使能Systick */
    SysTick->LOAD = 0X0FFFFFFF;             /* 注意systick计数器24位，所以这里设置最大重装载值 */
}


/**
 * @brief     延时nus
 * @note      无论是否使用OS, 都是用时钟摘取法来做us延时
 * @param     nus: 要延时的us数
 * @note      nus取值范围: 0 ~ (2^32 / fac_us) (fac_us一般等于系统主频, 自行套入计算)
 * @retval    无
 */
void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;        /* LOAD的值 */
    ticks = nus * g_fac_us;                 /* 需要的节拍数 */

    told = SysTick->VAL;                    /* 刚进入时的计数器值 */
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;        /* 这里注意一下SYSTICK是一个递减的计数器就可以了 */
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) 
            {
                break;                      /* 时间超过/等于要延迟的时间,则退出 */
            }
        }
    }


}

/**
 * @brief     延时nms
 * @param     nms: 要延时的ms数 (0< nms <= (2^32 / fac_us / 1000))(fac_us一般等于系统主频, 自行套入计算)
 * @retval    无
 */
void delay_ms(uint16_t nms)
{
    delay_us((uint32_t)(nms * 1000));                   /* 普通方式延时 */
}









