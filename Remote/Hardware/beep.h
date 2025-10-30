#ifndef __BEEP_H
#define __BEEP_H

#include "include.h"



/******************************************************************************************/

/* 蜂鸣器控制 */
#define BEEP(x)         sys_gpio_pin_set(GPIOC, SYS_GPIO_PIN13, x)

/* BEEP取反定义 */
#define BEEP_TOGGLE   do{ GPIOC->ODR ^= SYS_GPIO_PIN13; }while(0)     /* BEEP = !BEEP */

void beep_init(void);   /* 初始化蜂鸣器 */

#endif

















