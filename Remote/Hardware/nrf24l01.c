#include "nrf24l01.h"

//配对密码
const uint8_t TX_ADDRESS[]= {0x11,0x22,0x33,0x44,0x55};    //本地地址
const uint8_t RX_ADDRESS[]= {0x11,0x22,0x33,0x44,0x55};    //接收地址RX_ADDR_P0 == RX_ADDR



//初始化24L01的IO口
void NRF24L01_Configuration(void)
{ 
    RCC->APB2ENR |= 1 << 3;  /* 时钟使能 */
    
    sys_gpio_set(CE_Port, CE_Pin,
             SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_HIGH, SYS_GPIO_OTYPE_PP);  /* CE引脚模式设置(推挽输出) */
    sys_gpio_set(CSN_Port, CSN_Pin,
             SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_HIGH, SYS_GPIO_OTYPE_PP);  /* CSN引脚模式设置 推挽输出 */
    sys_gpio_set(IRQ_Port, IRQ_Pin,
             SYS_GPIO_MODE_IN, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_HIGH, SYS_GPIO_PUPD_PU);   /* IRQ引脚模式设置(上拉输入) */
    sys_gpio_set(SCK_Port, SCK_Pin,
             SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_HIGH, SYS_GPIO_OTYPE_PP);   /* SCK引脚模式设置(推挽输出) */
    sys_gpio_set(MISO_Port, MISO_Pin,
             SYS_GPIO_MODE_IN, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_HIGH, SYS_GPIO_PUPD_PU);   /* MOSI引脚模式设置(推挽输出) */
    sys_gpio_set(MOSI_Port, MOSI_Pin,
                 SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_HIGH, SYS_GPIO_OTYPE_PP);   /* MISO引脚模式设置(上拉输入) */

  

#ifdef   NFR_SPI 
    spi2_init();                /* 初始化SPI2 */

    /* 针对NRF的特点修改SPI的设置 */
    SPI2->CR1 &= ~(1 << 6);     /* SPE=0,SPI设备失能 */
    SPI2->CR1 &= ~(1 << 1);     /* CPOL=0,空闲状态下,SCK为低电平 */
    SPI2->CR1 &= ~(1 << 0);     /* CPHA=0,数据采样从第1个时间边沿开始 */
    SPI2->CR1 |= 1 << 6;        /* SPE=1,SPI设备使能 */

    NRF24L01_CE(0);             /* 使能24L01 */
    NRF24L01_CSN(1);            /* SPI片选取消 */
    spi2_set_speed(SPI_SPEED_32);                           /* spi速度为7.5Mhz（24L01的最大SPI时钟为10Mhz） */
#endif
}

// 模拟SPI 写一个字节
uint8_t SPI_SwapByte(uint8_t Byte)  
{
    uint8_t i;
    for(i = 0; i < 8; i ++) 
    {
        if((uint8_t)(Byte & 0x80) == 0x80)
        {
            W_MOSI(1);
        }         
        else 
        {
            W_MOSI(0);
        }            
        Byte = (Byte << 1);            
        W_SCK(1);                    
        Byte |= R_MISO;            
        W_SCK(0);                    
    }
    return Byte;
}
//上电检测NRF24L01是否在位
//写5个数据然后再读回来进行比较，
//相同时返回值:0，表示在位;否则返回1，表示不在位    
uint8_t NRF24L01_Check(void)
{

    uint8_t buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
    uint8_t i;
    nrf24l01_write_buf(SPI_WRITE_REG + TX_ADDR, buf, 5);    /* 写入5个字节的地址. */
    nrf24l01_read_buf(TX_ADDR, buf, 5);                     /* 读出写入的地址 */

    for (i = 0; i < 5; i++)
    {
        if (buf[i] != 0XA5) break;
    }
    
    if (i != 5) return 1;   /* 检测24L01错误 */

    return 0;               /* 检测到24L01 */
}          
//通过SPI写寄存器
static uint8_t nrf24l01_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t status;    
    W_SS(0);                    //使能SPI传输
#ifdef   NFR_SPI 
    status =spi2_read_write_byte(reg); //发送寄存器号 
      spi2_read_write_byte(value);            //写入寄存器的值
#else
      status =SPI_SwapByte(reg); //发送寄存器号 
      SPI_SwapByte(value);            //写入寄存器的值
#endif    
      W_SS(1);                    //禁止SPI传输       
      return(status);                        //返回状态值
}


//读取SPI寄存器值 ，regaddr:要读的寄存器
static uint8_t nrf24l01_read_reg(uint8_t reg)
{
    uint8_t reg_val;        
    W_SS(0);                //使能SPI传输  

#ifdef   NFR_SPI 
      spi2_read_write_byte(reg);     //发送寄存器号
      reg_val=spi2_read_write_byte(0XFF);//读取寄存器内容
#else
      SPI_SwapByte(reg);     //发送寄存器号
      reg_val=SPI_SwapByte(0XFF);//读取寄存器内容
#endif        

      W_SS(1);                //禁止SPI传输            
      return(reg_val);                 //返回状态值
}    
//在指定位置读出指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值 
static uint8_t nrf24l01_read_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
    uint8_t status,i;           
      W_SS(0);                     //使能SPI传输
#ifdef   NFR_SPI 
      status=spi2_read_write_byte(reg);   //发送寄存器值(位置),并读取状态值          
      for(i=0;i<len;i++)pbuf[i]=spi2_read_write_byte(0XFF);//读出数据
#else
      status=SPI_SwapByte(reg);   //发送寄存器值(位置),并读取状态值          
      for(i=0;i<len;i++)pbuf[i]=SPI_SwapByte(0XFF);//读出数据

    
#endif  
      W_SS(1);                     //关闭SPI传输
      return status;                        //返回读到的状态值
}
//在指定位置写指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值
static uint8_t nrf24l01_write_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
    uint8_t status,i;        
    W_SS(0);                                    //使能SPI传输
#ifdef   NFR_SPI 
      status = spi2_read_write_byte(reg);                //发送寄存器值(位置),并读取状态值
      for(i=0; i<len; i++)spi2_read_write_byte(*pbuf++); //写入数据 
#else
      status = SPI_SwapByte(reg);                //发送寄存器值(位置),并读取状态值
      for(i=0; i<len; i++)SPI_SwapByte(*pbuf++); //写入数据 

    
#endif  
      W_SS(1);                                    //关闭SPI传输
      return status;                                       //返回读到的状态值
}                   
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
uint8_t nrf24l01_tx_packet(uint8_t *txbuf)
{
    uint8_t state;   
    W_CE(0);
    nrf24l01_write_buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
    W_CE(1);                                     //启动发送       
    //while(NRF24L01_IRQ!=0);                         //等待发送完成
    state=nrf24l01_read_reg(STATUS);                     //读取状态寄存器的值       
    nrf24l01_write_reg(SPI_WRITE_REG+STATUS,state);      //清除TX_DS或MAX_RT中断标志
    if(state&MAX_TX)                                     //达到最大重发次数
    {
        nrf24l01_write_reg(FLUSH_TX,0xff);               //清除TX FIFO寄存器 
        return MAX_TX; 
    }
    if(state&TX_OK)                                      //发送完成
    {
        return TX_OK;
    }
    return 0xff;                                         //其他原因发送失败
}

//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
uint8_t nrf24l01_rx_packet(uint8_t *rxbuf)
{
    uint8_t state;                                              
    state=nrf24l01_read_reg(STATUS);                //读取状态寄存器的值         
    nrf24l01_write_reg(SPI_WRITE_REG+STATUS,state); //清除TX_DS或MAX_RT中断标志
    if(state&RX_OK)                                 //接收到数据
    {
        nrf24l01_read_buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
        nrf24l01_write_reg(FLUSH_RX,0xff);          //清除RX FIFO寄存器 
        return 0; 
    }       
    return 1;                                      //没收到任何数据
}

//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了           
void nrf24l01_rx_mode(void)
{
      W_CE(0);      
    //写RX节点地址
      nrf24l01_write_buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);

    //使能通道0的自动应答    
      nrf24l01_write_reg(SPI_WRITE_REG+EN_AA,0x01);    
    //使能通道0的接收地址       
      nrf24l01_write_reg(SPI_WRITE_REG+EN_RXADDR,0x01);
    //设置RF通信频率          
      nrf24l01_write_reg(SPI_WRITE_REG+RF_CH,45);       //(要改单机控制，就把45改成跟遥控器单独一样的。就可以单机控制了)
    //选择通道0的有效数据宽度         
      nrf24l01_write_reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
    //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
      nrf24l01_write_reg(SPI_WRITE_REG+RF_SETUP,0x0f);
    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX接收模式 
      nrf24l01_write_reg(SPI_WRITE_REG+NCONFIG, 0x0f); 
    //CE为高,进入接收模式 
      W_CE(1);                                
}            

//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,
//选择RF频道,波特率和LNA HCURR PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了           
//CE为高大于10us,则启动发送.     
void nrf24l01_tx_mode(void)
{                                                         
     W_CE(0);        
    //写TX节点地址 
      nrf24l01_write_buf(SPI_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);    
    //设置TX节点地址,主要为了使能ACK      
      nrf24l01_write_buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); 

    //使能通道0的自动应答    
      nrf24l01_write_reg(SPI_WRITE_REG+EN_AA,0x01);     
    //使能通道0的接收地址  
      nrf24l01_write_reg(SPI_WRITE_REG+EN_RXADDR,0x01); 
    //设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
      nrf24l01_write_reg(SPI_WRITE_REG+SETUP_RETR,0x1a);
    //设置RF通道为40
      nrf24l01_write_reg(SPI_WRITE_REG+RF_CH,45);     //(要改单机控制，就把45改成跟遥控器单独一样的。就可以单机控制了)
    //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
      nrf24l01_write_reg(SPI_WRITE_REG+RF_SETUP,0x0f);  //0x27  250K   0x07 1M
    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX发送模式,开启所有中断
      nrf24l01_write_reg(SPI_WRITE_REG+NCONFIG,0x0e);    
    // CE为高,10us后启动发送
     W_CE(1);                                  
}          

void NRF24L01_init(void)
{
    NRF24L01_Configuration();
    while(NRF24L01_Check()); 
    nrf24l01_tx_mode();
    
}



/*********************END OF FILE******************************************************/
















