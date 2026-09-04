#include "stmflash.h"

/**
 * @brief       解锁STM32的FLASH
 * @param       无
 * @retval      无
 */
static void stmflash_unlock(void)
{
    FLASH->KEYR = STM32_FLASH_KEY1;     /* FLASH 写入解锁序列. */
    FLASH->KEYR = STM32_FLASH_KEY2;
}

/**
 * @brief       flash上锁
 * @param       无
 * @retval      无
 */
static void stmflash_lock(void)
{
    FLASH->CR |= 1 << 7;    /* FLASH 上锁 */
}

/**
 * @brief       得到FLASH的错误状态
 * @param       无
 * @retval      执行结果
 *   @arg       0   : 已完成
 *   @arg       1   : 忙
 *   @arg       2   : 编程错误
 *   @arg       3   : 写保护错误
 */
static uint8_t stmflash_get_error_status(void)
{
    uint32_t res;
    res = FLASH->SR;

    if (res & (1 << 0))return 1;    /* BSY = 1      , 忙 */
    if (res & (1 << 2))return 2;    /* PGERR = 1    , 编程错误*/
    if (res & (1 << 4))return 3;    /* WRPRTERR = 1 , 写保护错误 */
    
    return 0;   /* 没有任何错误 操作完成. */
}

/**
 * @brief       等待操作完成
 * @param       time : 要延时的长短
 * @retval      执行结果
 *   @arg       0   : 已完成
 *   @arg       2   : 编程错误
 *   @arg       3   : 写保护错误
 *   @arg       0XFF: 超时
 */
static uint8_t stmflash_wait_done(uint32_t time)
{
    uint8_t res;

    do
    {
        res = stmflash_get_error_status();

        if (res != 1)
        {
            break;      /* 非忙, 无需等待了, 直接退出 */
        }
        
        time--;
    } while (time);

    if (time == 0)res = 0XFF;   /* 超时 */

    return res;
}

/**
 * @brief       擦除扇区
 * @param       saddr   : 扇区地址 0 ~ 256
 * @retval      执行结果
 *   @arg       0   : 已完成
 *   @arg       2   : 编程错误
 *   @arg       3   : 写保护错误
 *   @arg       0XFF: 超时
 */
static uint8_t stmflash_erase_sector(uint32_t saddr)
{
    uint8_t res = 0;
    saddr = (saddr * STM32_SECTOR_SIZE) + STM32_FLASH_BASE;   /* STM32F103擦除的时候是指定半字地址 */
    res = stmflash_wait_done(0X5FFFFF);     /* 等待上次操作结束, >20ms */

    if (res == 0)
    {
        FLASH->CR |= 1 << 1;    /* 页擦除 */
        FLASH->AR = saddr;      /* 设置页地址(实际是半字地址) */
        FLASH->CR |= 1 << 6;    /* 开始擦除 */
        res = stmflash_wait_done(0X5FFFFF); /* 等待操作结束, >20ms */

        if (res != 1)   /* 非忙 */
        {
            FLASH->CR &= ~(1 << 1); /* 清除页擦除标志 */
        }
    }

    return res;
}

/**
 * @brief       在FLASH指定地址写入半字 (16位数据)
 *   @note      这了写入半字, 是指2个字节
 * @param       faddr   : 写入地址 (此地址必须为2的倍数!!)
 * @param       data    : 要写入的数据(16位)
 * @retval      执行结果
 *   @arg       0   : 已完成
 *   @arg       2   : 编程错误
 *   @arg       3   : 写保护错误
 *   @arg       0XFF: 超时
 */
static uint8_t stmflash_write_halfword(uint32_t faddr, uint16_t data)
{
    uint8_t res;
    res = stmflash_wait_done(0X5FFFFF);

    if (res == 0)       /* OK */
    {
        FLASH->CR |= 1 << 0;                /* 编程使能 */
        *(volatile uint16_t *)faddr = data; /* 写入数据 */
        res = stmflash_wait_done(0X5FFFFF); /* 等待操作完成 */

        if (res != 1)   /* 操作成功 */
        {
            FLASH->CR &= ~(1 << 0);         /* 清除PG位 */
        }
    }

    return res;
}

/**
 * @brief       从指定地址读取一个半字 (16位数据)
 * @param       faddr   : 读取地址 (此地址必须为2的倍数!!)
 * @retval      读取到的数据 (16位)
 */
uint16_t stmflash_read_halfword(uint32_t faddr)
{
    return *(volatile uint16_t *)faddr;
}

/**
 * @brief       在FLASH 指定位置, 写入指定长度的数据(自动擦除)
 *   @note      该函数往 STM32 内部 FLASH 指定位置写入指定长度的数据
 *              该函数会先检测要写入的扇区是否是空(全0XFFFF)的?, 如果
 *              不是, 则先擦除, 如果是, 则直接往扇区里面写入数据.
 *              擦除过了, 否则可能写入失败.
 * @param       waddr   : 起始地址 (此地址必须为2的倍数!!,否则写入出错!)
 * @param       pbuf    : 数据指针
 * @param       length  : 要写入的 半字(16位)数
 * @retval      无
 */

uint16_t g_flashbuf[STM32_SECTOR_SIZE / 2]; /* 最多是2K字节 buffer */

void stmflash_write(uint32_t waddr, uint16_t *pbuf, uint16_t length)
{

    uint8_t status = 0;
    uint16_t secnum = 0;
    uint32_t addrx = 0;
    uint32_t endaddr = 0;

    if (waddr < STM32_FLASH_BASE || waddr % 2 ||        /* 写入地址小于 STM32_FLASH_BASE, 或不是2的整数倍, 非法. */
        waddr > (STM32_FLASH_BASE + STM32_FLASH_SIZE))  /* 写入地址大于 STM32_FLASH_BASE + STM32_FLASH_SIZE, 非法. */
    {
        return;
    }
    
    stmflash_unlock();              /* FLASH解锁 */
    addrx = waddr;                  /* 写入的起始地址 */
    endaddr = waddr + length * 2;   /* 写入的结束地址 */

    while (addrx < endaddr)         /* 扫清一切障碍 (对非0XFFFF的地方, 先擦除) */
    {
        secnum = (addrx - STM32_FLASH_BASE) / STM32_SECTOR_SIZE;    /* 得到扇区编号 */

        if (stmflash_read_halfword(addrx) != 0XFFFF)    /* 有非0XFFFF的地方,要擦除这个扇区 */
        {
            status = stmflash_erase_sector(secnum);     /* 擦除这个扇区 */

            if (status) /* 发生错误了 */
            {
                break;  /* 退出 */
            }
        }
        else
        {
            addrx += 4;
        }
    }

    if (status == 0)    /* 前面操作正常 */
    {
        while (waddr < endaddr) /* 写数据 */
        {
            if (stmflash_write_halfword(waddr, *pbuf))  /* 写入数据(每次2个字节) */
            {
                break;  /* 写入异常 */
            }

            waddr += 2; /* 地址加2 */
            pbuf++;     /* pbuf自增, 因为pbuf是16位的, 所以++就可以了 */
        }
    }
    
    stmflash_lock();    /* 上锁 */
}

/**
 * @brief       从指定地址开始读出指定长度的数据
 * @param       raddr : 起始地址
 * @param       pbuf  : 数据指针
 * @param       length: 要读取的半字(16位)数,即2个字节的整数倍
 * @retval      无
 */
void stmflash_read(uint32_t raddr, uint16_t *pbuf, uint16_t length)
{
    uint16_t i;

    for (i = 0; i < length; i++)
    {
        pbuf[i] = stmflash_read_halfword(raddr);    /* 读取2个字节 */
        raddr += 2; /* 偏移2个字节 */
    }
}











