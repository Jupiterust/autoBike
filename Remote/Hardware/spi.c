#include "spi.h"

/**
 * @brief       SPI初始化代码
 *   @note      主机模式,8位数据,禁止硬件片选
 * @param       无
 * @retval      无
 */
void spi2_init(void)
{
    uint32_t tempreg = 0;
    
    RCC->APB1ENR |= 1 << 14;          /* SPI2时钟使能 */
    RCC->APB2ENR |= 1 << 3;     /* SPI2_SCK脚时钟使能 */
    RCC->APB2ENR |= 1 << 3;     /* SPI2_MISO脚时钟使能 */
    RCC->APB2ENR |= 1 << 3;     /* SPI2_MOSI脚时钟使能 */

    sys_gpio_set(GPIOB, SYS_GPIO_PIN13,
                 SYS_GPIO_MODE_AF, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_HIGH, SYS_GPIO_PUPD_PU);   /* SCK引脚模式设置(复用输出) */

    sys_gpio_set(GPIOB, SYS_GPIO_PIN14,
                 SYS_GPIO_MODE_AF, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_HIGH, SYS_GPIO_PUPD_PU);   /* MISO引脚模式设置(复用输出) */

    sys_gpio_set(GPIOB, SYS_GPIO_PIN15,
                 SYS_GPIO_MODE_AF, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_HIGH, SYS_GPIO_PUPD_PU);   /* MOSI引脚模式设置(复用输出) */

    tempreg |= 0 << 11;             /* DFF = 0      , 使用8位数据格式 */
    tempreg |= 0 << 10;             /* RXONLY = 0   , 全双工模式 */
    tempreg |= 1 << 9;              /* SSM = 1      , 软件片选(NSS)控制 */
    tempreg |= 1 << 8;              /* SSI = 1      , 禁止软件从设备, 即做主机 */
    tempreg |= 0 << 7;              /* LSBFRST = 0  , MSB先传输 */
    tempreg |= 7 << 3;              /* BR[2:0] = 7  , 默认使用256分频, 速度最低 */
    tempreg |= 1 << 2;              /* MSTR = 1     , 主机模式 */
    tempreg |= 1 << 1;              /* CPOL = 0     , 空闲状态下, SCK保持高电平 */
    tempreg |= 1 << 0;              /* CPHA = 1     , 数据采集从第二个时钟边沿开始 */

    SPI2->CR1 = tempreg;        /* 设置CR1寄存器 */
    SPI2->CR1 |= 1 << 6;        /* SPE = 1      , 使能SPI */

    spi2_read_write_byte(0xff);     /* 启动传输, 实际上就是产生8个时钟脉冲, 达到清空DR的作用, 非必需 */
}

/**
 * @brief       SPI2速度设置函数
 *   @note      SPI2时钟选择来自APB1, 即PCLK1, 为36Mhz
 *              SPI速度 = PCLK1 / 2^(speed + 1)
 * @param       speed   : SPI2时钟分频系数
 * @retval      无
 */
void spi2_set_speed(uint8_t speed)
{
    speed &= 0X07;                  /* 限制范围 */
    SPI2->CR1 &= ~(1 << 6);     /* SPE = 0          , SPI设备失能 */
    SPI2->CR1 &= ~(7 << 3);     /* BR[2:0] = 0      , 先清零 */
    SPI2->CR1 |= speed << 3;    /* BR[2:0] = speed  , 设置分频系数 */
    SPI2->CR1 |= 1 << 6;        /* SPE = 1          , SPI使能 */
}

/**
 * @brief       SPI2读写一个字节数据
 * @param       txdata  : 要发送的数据(1字节)
 * @retval      接收到的数据(1字节)
 */
uint8_t spi2_read_write_byte(uint8_t txdata)
{
    while ((SPI2->SR & 1 << 1) == 0);   /* 等待发送区空 */

    SPI2->DR = txdata;                  /* 发送一个byte */

    while ((SPI2->SR & 1 << 0) == 0);   /* 等待接收完一个byte */

    return SPI2->DR;                    /* 返回收到的数据 */
}






