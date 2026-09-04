#include "beep.h"

/**
 * @brief       初始化BEEP相关IO口, 并使能时钟
 * @param       无
 * @retval      无
 */
void beep_init(void)
{
    RCC->APB2ENR |= 1 << 4; /* BEEP时钟使能 */

    sys_gpio_set(GPIOC, SYS_GPIO_PIN13,
                 SYS_GPIO_MODE_IN, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);   /* BEEP引脚模式设置 */

    BEEP(1);    /* 开启蜂鸣器 */
    delay_ms(20);
    BEEP(0);    /* 关闭蜂鸣器 */
}






