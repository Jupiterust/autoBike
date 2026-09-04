#ifndef __LED_H
#define __LED_H

#include "include.h"



#define LED0(x)        sys_gpio_pin_set(GPIOB, SYS_GPIO_PIN4, x)
#define LED1(x)        sys_gpio_pin_set(GPIOB, SYS_GPIO_PIN5, x)
#define LED0_TOGGLE   do{ GPIOB->ODR ^= SYS_GPIO_PIN4; }while(0)     
#define LED1_TOGGLE   do{ GPIOB->ODR ^= SYS_GPIO_PIN5; }while(0)    
void LED_init(void);       

#endif


















