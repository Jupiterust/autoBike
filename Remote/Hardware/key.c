#include "key.h"

 
void key_init(void)
{
    RCC->APB2ENR |= 1 << 2; /* KEY时钟使能 */
    RCC->APB2ENR |= 1 << 3; /* KEY时钟使能 */
    RCC->APB2ENR |= 1 << 4; /* KEY时钟使能 */

    sys_gpio_set(GPIOA, SYS_GPIO_PIN0,
                 SYS_GPIO_MODE_IN, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PD);    /* KEY0引脚模式设置,下拉输入 */
     
    sys_gpio_set(GPIOA, SYS_GPIO_PIN11 |SYS_GPIO_PIN12 |SYS_GPIO_PIN15,
                 SYS_GPIO_MODE_IN, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);    /* KEY0引脚模式设置,上拉输入 */

    sys_gpio_set(GPIOB, SYS_GPIO_PIN3 |SYS_GPIO_PIN6 |SYS_GPIO_PIN7 | SYS_GPIO_PIN8 |SYS_GPIO_PIN9,
             SYS_GPIO_MODE_IN, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);    /* KEY0引脚模式设置,上拉输入 */

    sys_gpio_set(GPIOC, SYS_GPIO_PIN14,
         SYS_GPIO_MODE_IN, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);    /* KEY0引脚模式设置,上拉输入 */

}

key_state key_s = key_release;

uint8_t Key_Flag[8]={0,0,0,0,0,0,0,0};//按键标志 随用随清


uint8_t KEY_Read_All(void)
{
   uint8_t tm=0;
   tm = ((KEY0)|(KEY1<<1)|(KEY2<<2)|(KEY3<<3)
    |(KEY4<<4)|(KEY5<<5)|(KEY6<<6)|(KEY9<<7)
    );//读取各个按键状态并编码
   if(tm==0xFF)
    {
       return 0;
    }
//   while(tm ==  ((KEY0)|(KEY1<<1)|(KEY2<<2)));//等待按键释放
   return  (~tm)&0xFF;
}



void Key_Scan(void)         //长短按按键扫描函数，需要加在10-20ms中断里执行
{
    static uint8_t  Key_flag,//长短按
        Key_value;//按谁;
    static uint16_t  Key_time;
    
    switch(key_s)
    {
        case key_release:   if(KEY_Read_All()>0)key_s = key_press;  break;
        case key_press:     if(!KEY0)      Key_value = 1;
                            else if(!KEY1) Key_value = 2;
                            else if(!KEY2) Key_value = 3;
                            else if(!KEY3) Key_value = 4;
                            else if(!KEY4) Key_value = 5;
                            else if(!KEY5) Key_value = 6;
                            else if(!KEY6) Key_value = 7;
                            else if(!KEY9) Key_value = 8;
                            else
                            {
                                key_s = key_press;
                                break;
                            }
                            key_s = key_wait;
                            break;
        case key_wait:      if(KEY_Read_All()>0)
                            {
                                Key_time += 20;
                                break;
                            }

                            if(Key_time > 300) Key_flag = 2;   //长按  //调节长按的响应时间
                            else               Key_flag = 1;
                            key_s = key_release;
                            Key_time = 40;
                            break;
    }
   if((Key_flag == 1)&&(Key_value == 1)){Key_Flag[0] = 1;Key_flag =Key_value =0;}
    //else if((Key_flag == 2)&&(Key_value == 1)){Key_Flag[0] = 2;Key_flag =Key_value =0;}
    
    if((Key_flag == 1)&&(Key_value == 2)){Key_Flag[1] = 1;Key_flag =Key_value =0;}
    //else if((Key_flag == 2)&&(Key_value == 2)){Key_Flag[1] = 2;Key_flag =Key_value =0;}
    
    if((Key_flag == 1)&&(Key_value == 3)){Key_Flag[2] = 1;Key_flag =Key_value =0;}
    //else if((Key_flag == 2)&&(Key_value == 3)){Key_Flag[2] = 2;Key_flag =Key_value =0;}
    
    if((Key_flag == 1)&&(Key_value == 4)){Key_Flag[3] = 1;Key_flag =Key_value =0;}
    //else if((Key_flag == 2)&&(Key_value == 4)){Key_Flag[3] = 2;Key_flag =Key_value =0;}
    
    if((Key_flag == 1)&&(Key_value == 5)){Key_Flag[4] = 1;Key_flag =Key_value =0;}
    //else if((Key_flag == 2)&&(Key_value == 5)){Key_Flag[4] = 2;Key_flag =Key_value =0;}
    
    if((Key_flag == 1)&&(Key_value == 6)){Key_Flag[5] = 1;Key_flag =Key_value =0;}
    //else if((Key_flag == 2)&&(Key_value == 6)){Key_Flag[5] = 2;Key_flag =Key_value =0;}
    
    if((Key_flag == 1)&&(Key_value == 7)){Key_Flag[6] = 1;Key_flag =Key_value =0;}
    //else if((Key_flag == 2)&&(Key_value == 7)){Key_Flag[6] = 2;Key_flag =Key_value =0;}
    
    if((Key_flag == 1)&&(Key_value == 8)){Key_Flag[7] = 1;Key_flag =Key_value =0;}
   // else if((Key_flag == 2)&&(Key_value == 8)){Key_Flag[7] = 2;Key_flag =Key_value =0;}
}






