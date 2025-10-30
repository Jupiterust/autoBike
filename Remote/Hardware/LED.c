#include "LED.h"


/**
 * @brief       LED初始化函数
 * @param       无
 * @retval      无
 */
void LED_init(void)
{
    RCC->APB2ENR |= 1 << 3; 
    sys_gpio_set(GPIOB, SYS_GPIO_PIN4 |SYS_GPIO_PIN5,
                 SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);
    LED0(0);
    LED1(0);
}


