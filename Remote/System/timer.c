#include "timer.h"



/**
 * @brief       通用定时器TIMX定时中断初始化函数
 * @note
 *              通用定时器的时钟来自APB1,当PPRE1 ≥ 2分频的时候
 *              通用定时器的时钟为APB1时钟的2倍, 而APB1为36M, 所以定时器时钟 = 72Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值。
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void Time3_init(uint16_t arr, uint16_t psc)
{
    RCC->APB1ENR |= 1 << 1;
    TIM3->ARR = arr;           /* 设定计数器自动重装值 */
    TIM3->PSC = psc;           /* 设置预分频器  */
    TIM3->DIER |= 1 << 0;      /* 允许更新中断 */
    TIM3->CR1 |= 1 << 0;       /* 使能定时器TIMX */
    sys_nvic_init(1, 3, TIM3_IRQn, 2); /* 抢占1，子优先级3，组2 */
}



////TIM3中断函数
//void TIM3_IRQHandler(void)
//{
//    if(TIM3->SR & 0X0001)   /* 溢出中断 */
//    {
//       
//    }
//    TIM3->SR &= ~(1 << 0); /* 清除中断标志位 */
//}







