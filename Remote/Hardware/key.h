#ifndef __KEY_H
#define __KEY_H

#include "include.h"

#define KEY0        sys_gpio_pin_get(GPIOB, SYS_GPIO_PIN9)   //вСио
#define KEY1        sys_gpio_pin_get(GPIOA, SYS_GPIO_PIN11)  //срио
#define KEY2        sys_gpio_pin_get(GPIOB, SYS_GPIO_PIN8)   //вСвС
#define KEY3        sys_gpio_pin_get(GPIOB, SYS_GPIO_PIN7)   //вСср
#define KEY4        sys_gpio_pin_get(GPIOA, SYS_GPIO_PIN15)  //срвС  
#define KEY5        sys_gpio_pin_get(GPIOB, SYS_GPIO_PIN3)   //срср 
#define KEY6        sys_gpio_pin_get(GPIOB, SYS_GPIO_PIN6)   //вСоб 
#define KEY7        sys_gpio_pin_get(GPIOC, SYS_GPIO_PIN14)  //жпвС  
#define KEY8       sys_gpio_pin_get(GPIOA, SYS_GPIO_PIN0)    //жпср
#define KEY9       sys_gpio_pin_get(GPIOA, SYS_GPIO_PIN12)   //сроб

extern uint8_t Tx_Buf[14];
typedef enum{
    key_release,
    key_press,
    key_wait,
}key_state;

extern uint8_t Key_Flag[8];

uint8_t KEY_Read_All(void);

void Key_Scan(void);
void key_init(void);        /* ????????????? */


#endif


















