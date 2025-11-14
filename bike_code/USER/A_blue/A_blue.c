#include "A_blue.h"


TxPack txpack;
RxPack rxpack;

const unsigned short  TXPACK_BYTE_SIZE = ((TX_BOOL_NUM+7)>>3)+TX_BYTE_NUM+(TX_SHORT_NUM<<1)+(TX_INT_NUM<<2)+(TX_FLOAT_NUM<<2);// 发送数据包的字节长度
const unsigned short  RXPACK_BYTE_SIZE = ((RX_BOOL_NUM+7)>>3)+RX_BYTE_NUM+(RX_SHORT_NUM<<1)+(RX_INT_NUM<<2)+(RX_FLOAT_NUM<<2);// 接收数据包的字节长度

long rdIndex=0;//// 读取计数每次在读取数据包后增加 +(数据包长度)  读取计数-记录当前的数据包读取进度，读取计数会一直落后于接收计数，当读取计数与接收计数之间距离超过一个接收数据包的长度时，会启动一次数据包的读取。
long rxIndex=0;// 接收计数-记录当前的数据接收进度 每次随串口的接收中断后 +1
uint8_t vp_rxbuff[VALUEPACK_BUFFER_SIZE];// 用于环形缓冲区的数组，环形缓冲区的大小可以在.h文件中定义VALUEPACK_BUFFER_SIZE
uint16_t vp_circle_rx_index=0;// 数据包环形缓冲区计数

#ifdef debug 
bool txet_bool=0;
char text_char=1;
short txet_short=2;
int txet_int=3;
float txet_float=4;
#endif

void Rx_pack_allocation(void)
{
#ifdef debug 
    txet_bool = rxpack.bools[0];
    text_char = rxpack.bytes[0];
    txet_short = rxpack.shorts[0];
    txet_int  = rxpack.integers[0];
    txet_float  = rxpack.floats[0];
#endif
    if(rxpack.bools[0] == 1)
    {
        if(param.M0_Flag == 0)param.M0_Flag = 1;
        else param.M0_Flag = 0;
        rxpack.bools[0] =0;
    }
    if(rxpack.bools[1] == 1)
    {
        if(param.M1_Flag == 0)param.M1_Flag = 1;
        else param.M1_Flag = 0;
        rxpack.bools[1] =0;
    }

		
	
    Servo_Ctl =  -(float)rxpack.integers[0];

    param.angular_kp = rxpack.floats[0];
    param.angular_ki = rxpack.floats[1];
    param.angular_kd = rxpack.floats[2];
    param.angular_v_kp = rxpack.floats[3];
    param.angular_v_ki = rxpack.floats[4]; 
    param.angular_v_kd = rxpack.floats[5];
    param.fly_wheel_speed_kp = rxpack.floats[6];
    param.fly_wheel_speed_ki = rxpack.floats[7];
    param.fly_wheel_speed_kd = rxpack.floats[8];
    param.angular_zero = rxpack.floats[9]; 
    param.Steer_Kp = rxpack.floats[10];
    param.Steer_Ki = rxpack.floats[11];
    param.Steer_Kd = rxpack.floats[12];
    
   //     txpack.floats[13] = imu.rol;
     M1_Ctl = rxpack.floats[14]/50.0f;

}

void Tx_pack_allocation(void)
{
#ifdef debug 
    txpack.bools[0] = txet_bool;
    txpack.bytes[0] = text_char;
    txpack.shorts[0] = txet_short;
    txpack.integers[0] = txet_int;
    txpack.floats[0] = txet_float ;
#endif
    txpack.bools[0]=param.M0_Flag;
    txpack.bools[1]=param.M1_Flag;

    //txpack.bytes[0] = odrive.now_speed0;
	
		txpack.shorts[0] = odrive.now_speed0;
	
    txpack.integers[0] = Servo_Ctl;
	
    txpack.floats[0] = param.angular_kp;
    txpack.floats[1] = param.angular_ki;
    txpack.floats[2] = param.angular_kd;
    txpack.floats[3] = param.angular_v_kp;
    txpack.floats[4] = param.angular_v_ki; 
    txpack.floats[5] = param.angular_v_kd;
    txpack.floats[6] = param.fly_wheel_speed_kp;
    txpack.floats[7] = param.fly_wheel_speed_ki;
    txpack.floats[8] = param.fly_wheel_speed_kd;
    txpack.floats[9] = param.angular_zero; 
    txpack.floats[10] = param.Steer_Kp;
    txpack.floats[11] = param.Steer_Ki;
    txpack.floats[12] = param.Steer_Kd;
    
    txpack.floats[13] = imu.rol;
    
    txpack.floats[14] = odrive.set_speed1;

    sendValuePack(&txpack);  // 在此对数据包赋值并将数据发送到手机

}


void uart_data_analysis(uint8_t data)//放中断里  注意CPU处理能力
{
    vp_rxbuff[vp_circle_rx_index++] = data;   // 读取数据到缓冲区中  将环形缓冲接收计数加一
    if(vp_circle_rx_index>=VALUEPACK_BUFFER_SIZE)vp_circle_rx_index=0;   // 数据到达缓冲区尾部后，转移到头部
    rxIndex++;  // 将全局接收计数加一
}


void uart_data_treating(void)//放while
{
    if(readValuePack(&rxpack))
    {
        Rx_pack_allocation();
       
    }
    Tx_pack_allocation();

}

#define UART_DMA 1


// unsigned char readValuePack(RxPack *rx_pack_ptr)
// 尝试从缓冲区中读取数据包
// 参数   - RxPack *rx_pack_ptr： 传入接收数据结构体的指针，从环形缓冲区中读取出数据包，并将各类数据存储到rx_pack_ptr指向的结构体中
// 返回值 - 如果成功读取到数据包，则返回1，否则返回0
unsigned char readValuePack(RxPack *rx_pack_ptr)
{
    unsigned short i;
    unsigned char sum=0;// 用于和校验
    unsigned char isok=0;// 存放数据包读取的结果
    unsigned short rx_pack_length = RXPACK_BYTE_SIZE+3;// 接收数据包的原数据加上包头、校验和包尾 之后的字节长度
    unsigned int err=0;// 记录读取的错误字节的次数
    long rdi,rdii,idl,bool_bit;// 数据读取涉及到的变量
    uint32_t  idc;// 变量地址
    
  
#ifdef UART_DMA
    
    unsigned short this_index=0;
    static unsigned short last_index=0;
    this_index =VALUEPACK_BUFFER_SIZE-__HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
	if(this_index<last_index)
        rxIndex+=VALUEPACK_BUFFER_SIZE+this_index-last_index;
	else
		rxIndex+=this_index-last_index;
    last_index = this_index;	

#endif
    
    
    while(rdIndex<(rxIndex-((rx_pack_length)*2))) rdIndex+=rx_pack_length;  // 确保读取计数和接收计数之间的距离小于2个数据包的长度
    while(rdIndex<=(rxIndex-rx_pack_length)) // 如果读取计数落后于接收计数超过 1个 数据包的长度，则尝试读取
    {
        rdi = rdIndex % VALUEPACK_BUFFER_SIZE;
        rdii=rdi+1;

        if(vp_rxbuff[rdi]==PACK_HEAD) // 比较包头
        {
            if(vp_rxbuff[(rdi+RXPACK_BYTE_SIZE+2)%VALUEPACK_BUFFER_SIZE]==PACK_TAIL) // 比较包尾 确定包尾后，再计算校验和
            {

                 for(i=0;i<RXPACK_BYTE_SIZE;i++) //  计算校验和
                {
                    rdi++;
                    if(rdi>=VALUEPACK_BUFFER_SIZE) rdi -= VALUEPACK_BUFFER_SIZE;
                    sum += vp_rxbuff[rdi];
                }
               rdi++;
               if(rdi>=VALUEPACK_BUFFER_SIZE) rdi -= VALUEPACK_BUFFER_SIZE;
               if(sum==vp_rxbuff[rdi]) // 校验和正确，则开始将缓冲区中的数据读取出来 //  提取数据包数据 一共有五步， bool byte short int float
                {
// 1. bool
#if  RX_BOOL_NUM>0
                      idc = (uint32_t)rx_pack_ptr->bools;
                      idl = (RX_BOOL_NUM+7)>>3;
                      bool_bit = 0;
                      for(i=0;i<RX_BOOL_NUM;i++)
                      {
                        *((unsigned char *)(idc+i)) = (vp_rxbuff[rdii]&(0x01<<bool_bit))?1:0;
                        bool_bit++;
                        if(bool_bit>=8)
                        {
                          bool_bit = 0;
                          rdii ++;
                        }
                      }
                    if(bool_bit) rdii ++;
                        #endif
// 2.byte
#if RX_BYTE_NUM>0
                      idc = (uint32_t)(rx_pack_ptr->bytes);
                      idl = RX_BYTE_NUM;
                      for(i=0;i<idl;i++)
                      {
                        if(rdii>=VALUEPACK_BUFFER_SIZE)rdii -= VALUEPACK_BUFFER_SIZE;
                        (*((unsigned char *)idc))= vp_rxbuff[rdii];
                            rdii++;
                            idc++;
                      }
                        #endif
// 3.short
#if RX_SHORT_NUM>0
                      idc = (uint32_t)(rx_pack_ptr->shorts);
                      idl = RX_SHORT_NUM<<1;
                      for(i=0;i<idl;i++)
                      {
                        if(rdii>=VALUEPACK_BUFFER_SIZE)rdii -= VALUEPACK_BUFFER_SIZE;
                        (*((unsigned char *)idc))= vp_rxbuff[rdii];
                            rdii++;
                            idc++;
                      }
                        #endif
// 4.int
#if RX_INT_NUM>0
                      idc = (uint32_t)(&(rx_pack_ptr->integers[0]));
                      idl = RX_INT_NUM<<2;
                      for(i=0;i<idl;i++)
                      {
                        if(rdii>=VALUEPACK_BUFFER_SIZE)rdii -= VALUEPACK_BUFFER_SIZE;
                        (*((unsigned char *)idc))= vp_rxbuff[rdii];
                            rdii++;
                            idc++;
                      }
                        #endif
// 5.float
#if RX_FLOAT_NUM>0
                      idc = (uint32_t)(&(rx_pack_ptr->floats[0]));
                      idl = RX_FLOAT_NUM<<2;
                      for(i=0;i<idl;i++)
                      {
                        if(rdii>=VALUEPACK_BUFFER_SIZE)rdii -= VALUEPACK_BUFFER_SIZE;
                        (*((unsigned char *)idc))= vp_rxbuff[rdii];
                            rdii++;
                            idc++;
                      }
                        #endif
                    rdIndex+=rx_pack_length; // 更新读取计数
                    isok = 1;
                }
               else
                {
                  rdIndex++; // 校验值错误 则 err+1 且 更新读取计数
                  err++;
                }
            }
            else
            {
                rdIndex++;
                err++; // 包尾错误 则 err+1 且 更新读取计数
            }
        }
        else
        {
            rdIndex++; // 包头错误 则 err+1 且 更新读取计数
            err++;
        }
    }
    return isok;
}



//  void sendValuePack(TxPack *tx_pack_ptr)
//  将发送数据结构体中的变量打包，并发送出去
//  传入参数- TxPack *tx_pack_ptr 待发送数据包的指针
//  先将待发送数据包结构体的变量转移到“发送数据缓冲区”中，然后将发送数据缓冲区中的数据发送
void sendValuePack(TxPack *tx_pack_ptr)
{
    // 数据包发送涉及的变量
  int i;
  unsigned short loop;
  unsigned char valuepack_tx_bit_index=0;
  unsigned char valuepack_tx_index=1;//  由于结构体中不同类型的变量在内存空间的排布不是严格对齐的，中间嵌有无效字节，因此需要特殊处理
  unsigned char  sum=0;// 存放数据包读取的结果
  static unsigned char vp_txbuff[TXPACK_BYTE_SIZE+3];//用于暂存发送数据的数组
  vp_txbuff[0]=0xa5;//包头

#if TX_BOOL_NUM>0
      for(loop=0;loop<TX_BOOL_NUM;loop++)
      {
          if(tx_pack_ptr->bools[loop]) vp_txbuff[valuepack_tx_index] |= 0x01<<valuepack_tx_bit_index;
          else   vp_txbuff[valuepack_tx_index] &= ~(0x01<<valuepack_tx_bit_index);
          valuepack_tx_bit_index++;
          if(valuepack_tx_bit_index>=8)
          {
              valuepack_tx_bit_index = 0;
              valuepack_tx_index++;
          }
      }
      if(valuepack_tx_bit_index!=0)  valuepack_tx_index++;
    #endif

#if TX_BYTE_NUM>0
      for(loop=0;loop<TX_BYTE_NUM;loop++)
      {
          vp_txbuff[valuepack_tx_index] = tx_pack_ptr->bytes[loop];
          valuepack_tx_index++;
      }
    #endif

#if TX_SHORT_NUM>0
      for(loop=0;loop<TX_SHORT_NUM;loop++)
      {
          vp_txbuff[valuepack_tx_index] = tx_pack_ptr->shorts[loop]&0xff;
          vp_txbuff[valuepack_tx_index+1] = tx_pack_ptr->shorts[loop]>>8;
          valuepack_tx_index+=2;
      }
    #endif

#if TX_INT_NUM>0
      for(loop=0;loop<TX_INT_NUM;loop++)
      {
          i = tx_pack_ptr->integers[loop];
          vp_txbuff[valuepack_tx_index] = i&0xff;
          vp_txbuff[valuepack_tx_index+1] = (i>>8)&0xff;
          vp_txbuff[valuepack_tx_index+2] = (i>>16)&0xff;
          vp_txbuff[valuepack_tx_index+3] = (i>>24)&0xff;
          valuepack_tx_index+=4;
        }
    #endif

#if TX_FLOAT_NUM>0
      for(loop=0;loop<TX_FLOAT_NUM;loop++)
      {
          i = *(int *)(&(tx_pack_ptr->floats[loop]));
          vp_txbuff[valuepack_tx_index] = i&0xff;
          vp_txbuff[valuepack_tx_index+1] = (i>>8)&0xff;
          vp_txbuff[valuepack_tx_index+2] =(i>>16)&0xff;
          vp_txbuff[valuepack_tx_index+3] = (i>>24)&0xff;
          valuepack_tx_index+=4;
      }
#endif
    for(i=1;i<=TXPACK_BYTE_SIZE;i++)sum+=vp_txbuff[i];
    vp_txbuff[TXPACK_BYTE_SIZE+1] = sum;
    vp_txbuff[TXPACK_BYTE_SIZE+2] = 0x5a;
      
    
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)vp_txbuff, TXPACK_BYTE_SIZE+3);
		//HAL_Delay(2);
     
}

