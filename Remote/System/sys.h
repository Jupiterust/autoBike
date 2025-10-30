#ifndef __SYS_H
#define __SYS_H


#include "include.h"

//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入



/* sys_nvic_ex_config专用宏定义 */
#define SYS_GPIO_FTIR           1       /* 下降沿触发 */
#define SYS_GPIO_RTIR           2       /* 上升沿触发 */
#define SYS_GPIO_BTIR           3       /* 任意边沿触发 */

/* GPIO设置专用宏定义 */
#define SYS_GPIO_MODE_IN        0       /* 普通输入模式 */
#define SYS_GPIO_MODE_OUT       1       /* 普通输出模式 */
#define SYS_GPIO_MODE_AF        2       /* AF功能模式 */
#define SYS_GPIO_MODE_AIN       3       /* 模拟输入模式 */

#define SYS_GPIO_SPEED_LOW      2       /* GPIO速度(低速,2M) */
#define SYS_GPIO_SPEED_MID      1       /* GPIO速度(中速,10M) */
#define SYS_GPIO_SPEED_HIGH     3       /* GPIO速度(高速,50M) */

#define SYS_GPIO_PUPD_NONE      0       /* 不带上下拉 */
#define SYS_GPIO_PUPD_PU        1       /* 上拉 */
#define SYS_GPIO_PUPD_PD        2       /* 下拉 */

#define SYS_GPIO_OTYPE_PP       0       /* 推挽输出 */
#define SYS_GPIO_OTYPE_OD       1       /* 开漏输出 */

/* GPIO引脚位置宏定义  */
#define SYS_GPIO_PIN0           1<<0
#define SYS_GPIO_PIN1           1<<1
#define SYS_GPIO_PIN2           1<<2
#define SYS_GPIO_PIN3           1<<3
#define SYS_GPIO_PIN4           1<<4
#define SYS_GPIO_PIN5           1<<5
#define SYS_GPIO_PIN6           1<<6
#define SYS_GPIO_PIN7           1<<7
#define SYS_GPIO_PIN8           1<<8
#define SYS_GPIO_PIN9           1<<9
#define SYS_GPIO_PIN10          1<<10
#define SYS_GPIO_PIN11          1<<11
#define SYS_GPIO_PIN12          1<<12
#define SYS_GPIO_PIN13          1<<13
#define SYS_GPIO_PIN14          1<<14
#define SYS_GPIO_PIN15          1<<15

//JTAG模式设置定义
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00    
void JTAG_Set(uint8_t mode);


/*函数申明*******************************************************************************************/
/* 静态函数(仅在sys.c里面用到) */
static void sys_nvic_priority_group_config(uint8_t group);                      /* 设置NVIC分组 */


/* 普通函数 */
void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset);             /* 设置中断偏移量 */
void sys_nvic_init(uint8_t pprio, uint8_t sprio, uint8_t ch, uint8_t group);    /* 设置NVIC */
void sys_nvic_ex_config(GPIO_TypeDef *p_gpiox, uint16_t pinx, uint8_t tmode);   /* 外部中断配置函数,只针对GPIOA~GPIOK */
void sys_gpio_remap_set(uint8_t pos, uint8_t bit, uint8_t val);                 /* GPIO REMAP 设置 */
void sys_gpio_set(GPIO_TypeDef *p_gpiox, uint16_t pinx, uint32_t mode, 
                  uint32_t otype, uint32_t ospeed, uint32_t pupd);              /*  GPIO通用设置 */
void sys_gpio_pin_set(GPIO_TypeDef *p_gpiox, uint16_t pinx, uint8_t status);    /* 设置GPIO某个引脚的输出状态 */
uint8_t sys_gpio_pin_get(GPIO_TypeDef *p_gpiox, uint16_t pinx);                 /* 读取GPIO某个引脚的状态 */
void sys_standby(void);                     /* 进入待机模式 */
void sys_soft_reset(void);                  /* 系统软复位 */
uint8_t sys_clock_set(uint32_t plln);       /* 时钟设置函数 */
void sys_stm32_clock_init(uint32_t plln);   /* 系统时钟初始化函数 */


/* 以下为汇编函数 */
void sys_wfi_set(void);             /* 执行WFI指令 */
void sys_intx_disable(void);        /* 关闭所有中断 */
void sys_intx_enable(void);         /* 开启所有中断 */
void sys_msr_msp(uint32_t addr);    /* 设置栈顶地址 */

#endif











