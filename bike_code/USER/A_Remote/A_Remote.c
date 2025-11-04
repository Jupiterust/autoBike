#include "A_Remote.h"

Tx_Pack tx_pack;
Rx_Pack rx_pack;

const unsigned short  TX_PACK_BYTE_SIZE = ((TX_BOOL+7)>>3)+TX_BYTE+(TX_SHORT<<1)+(TX_INT<<2)+(TX_FLOAT<<2);// 发送数据包的字节长度
const unsigned short  RX_PACK_BYTE_SIZE = ((RX_BOOL+7)>>3)+RX_BYTE+(RX_SHORT<<1)+(RX_INT<<2)+(RX_FLOAT<<2);// 接收数据包的字节长度

long rd_Index=0;//// 读取计数每次在读取数据包后增加 +(数据包长度)  读取计数-记录当前的数据包读取进度，读取计数会一直落后于接收计数，当读取计数与接收计数之间距离超过一个接收数据包的长度时，会启动一次数据包的读取。
long rx_Index=0;// 接收计数-记录当前的数据接收进度 每次随串口的接收中断后 +1
unsigned char v_rxbuff[BUFFER_SIZE];// 用于环形缓冲区的数组，环形缓冲区的大小可以在.h文件中定义BUFFER_SIZE
unsigned int v_circle_rx_index=0;// 数据包环形缓冲区计数




void Rx_pack(void)
{
	    if(rx_pack.bools[0] == 1)
    {
        if(param.M0_Flag == 0)param.M0_Flag = 1;
        else param.M0_Flag = 0;
        rx_pack.bools[0] =0;
    }
    if(rx_pack.bools[6] == 1)
    {
        if(param.M1_Flag == 0)
				{
					param.M1_Flag = 1;
					M1_Ctl = 1.8f;
				}
				else
				{
					 param.M1_Flag = 0;
						M1_Ctl = 0;
				}
        rx_pack.bools[6] =0;
    }
		
		if(rx_pack.bools[2] == 1)
    {
				upper_Flag =1;
        rx_pack.bools[2] =0;
    }
	 if(rx_pack.bools[3] == 1)
    {
       	upper_Flag=0;
        rx_pack.bools[3] =0;
    }
		
	 if(rx_pack.bools[4] == 1)
	 {
			param.angular_zero = param.angular_zero-0.02f;
			rx_pack.bools[4] =0;
	 }
	 if(rx_pack.bools[5] == 1)
	 {
		  param.angular_zero = param.angular_zero+0.02f;
			rx_pack.bools[5] =0;
	 } 

	 


//	txet_bool = rx_pack.bools[0];
//	txet_bool = rx_pack.bools[1];
//	txet_bool = rx_pack.bools[2];
//	txet_bool = rx_pack.bools[3];
//	txet_bool = rx_pack.bools[4];
//	txet_bool = rx_pack.bools[5];
//	txet_bool = rx_pack.bools[6];
//	txet_bool = rx_pack.bools[7];    


//Servo_Ctl = rx_pack.shorts[0];
//M1_Ctl = rx_pack.integers[0];
//text_char = rx_pack.floats[0];

	if(rx_pack.bytes[0]>137)  Servo_Ctl = 80;
	else if(rx_pack.bytes[0]<117)  Servo_Ctl = -80;
	else if((rx_pack.bytes[0]<=137)&&(rx_pack.bytes[0]>117))Servo_Ctl = 0;
	
	 
	static uint8_t Flag = 0;
	
	if(rx_pack.bools[7] == 1)
	 {
        if(Flag == 0)Flag = 1;
        else Flag = 0;
		    rx_pack.bools[7] =0;
	 } 
	if(Flag == 1)
	{
		if (rx_pack.shorts[0] >= 117 && rx_pack.shorts[0] <= 137)M1_Ctl = 0.0;
		else if (rx_pack.shorts[0] < 117) M1_Ctl = (rx_pack.shorts[0] - 127) / 80.f;
		else if (rx_pack.shorts[0] > 137) M1_Ctl = (rx_pack.shorts[0] - 127) / 80.f;
	}
}

void Tx_pack(void)
{

    tx_pack.bools[0] = param.M0_Flag;
	  tx_pack.bools[1] = param.M1_Flag;
	
    tx_pack.bytes[0] = Servo_Ctl;
    tx_pack.shorts[0] = (short)M1_Ctl;
    tx_pack.integers[0] = (int)odrive.now_speed0;
    tx_pack.floats[0] = param.angular_zero ;
	
    send_ValuePack(&tx_pack);  // 在此对数据包赋值并将数据发送到手机
}


void data_analysis(uint8_t data)//放中断里  注意CPU处理能力
{
    v_rxbuff[v_circle_rx_index++] = data;   // 读取数据到缓冲区中  将环形缓冲接收计数加一
    if(v_circle_rx_index>=BUFFER_SIZE)v_circle_rx_index=0;   // 数据到达缓冲区尾部后，转移到头部
    rx_Index++;  // 将全局接收计数加一
}


void data_treating(void)//放while
{
    if(read_ValuePack(&rx_pack))
    {
        Rx_pack();
			  //Tx_pack();
    }



}




// unsigned char read_ValuePack(RxPack *rx_pack_ptr)
// 尝试从缓冲区中读取数据包
// 参数   - RxPack *rx_pack_ptr： 传入接收数据结构体的指针，从环形缓冲区中读取出数据包，并将各类数据存储到rx_pack_ptr指向的结构体中
// 返回值 - 如果成功读取到数据包，则返回1，否则返回0
unsigned char read_ValuePack(Rx_Pack *rx_pack_ptr)
{
    unsigned short i;
    unsigned char sum=0;// 用于和校验
    unsigned char isok=0;// 存放数据包读取的结果
    unsigned short r_pack_length = RX_PACK_BYTE_SIZE+3;// 接收数据包的原数据加上包头、校验和包尾 之后的字节长度
    unsigned int err=0;// 记录读取的错误字节的次数
    long rdi,rdii,idl,bool_bit;// 数据读取涉及到的变量
    uint32_t  idc;// 变量地址

    while(rd_Index<(rx_Index-((r_pack_length)*2))) rd_Index+=r_pack_length;  // 确保读取计数和接收计数之间的距离小于2个数据包的长度
    while(rd_Index<=(rx_Index-r_pack_length)) // 如果读取计数落后于接收计数超过 1个 数据包的长度，则尝试读取
    {
        rdi = rd_Index % BUFFER_SIZE;
        rdii=rdi+1;

        if(v_rxbuff[rdi]==P_HEAD ) // 比较包头
        {
            if(v_rxbuff[(rdi+RX_PACK_BYTE_SIZE+2)%BUFFER_SIZE]==P_TAIL) // 比较包尾 确定包尾后，再计算校验和
            {

                  for(i=0;i<RX_PACK_BYTE_SIZE;i++) //  计算校验和
                {
                    rdi++;
                    if(rdi>=BUFFER_SIZE) rdi -= BUFFER_SIZE;
                    sum += v_rxbuff[rdi];
                }
               rdi++;
               if(rdi>=BUFFER_SIZE) rdi -= BUFFER_SIZE;
               if(sum==v_rxbuff[rdi]) // 校验和正确，则开始将缓冲区中的数据读取出来 //  提取数据包数据 一共有五步， bool byte short int float
                {
// 1. bool
#if  RX_BOOL>0
                      idc = (uint32_t)rx_pack_ptr->bools;
                      idl = (RX_BOOL+7)>>3;
                      bool_bit = 0;
                      for(i=0;i<RX_BOOL;i++)
                      {
                        *((unsigned char *)(idc+i)) = (v_rxbuff[rdii]&(0x01<<bool_bit))?1:0;
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
#if RX_BYTE>0
                      idc = (uint32_t)(rx_pack_ptr->bytes);
                      idl = RX_BYTE;
                      for(i=0;i<idl;i++)
                      {
                        if(rdii>=BUFFER_SIZE)rdii -= BUFFER_SIZE;
                        (*((unsigned char *)idc))= v_rxbuff[rdii];
                            rdii++;
                            idc++;
                      }
                        #endif
// 3.short
#if RX_SHORT>0
                      idc = (uint32_t)(rx_pack_ptr->shorts);
                      idl = RX_SHORT<<1;
                      for(i=0;i<idl;i++)
                      {
                        if(rdii>=BUFFER_SIZE)rdii -= BUFFER_SIZE;
                        (*((unsigned char *)idc))= v_rxbuff[rdii];
                            rdii++;
                            idc++;
                      }
                        #endif
// 4.int
#if RX_INT>0
                      idc = (uint32_t)(&(rx_pack_ptr->integers[0]));
                      idl = RX_INT<<2;
                      for(i=0;i<idl;i++)
                      {
                        if(rdii>=BUFFER_SIZE)rdii -= BUFFER_SIZE;
                        (*((unsigned char *)idc))= v_rxbuff[rdii];
                            rdii++;
                            idc++;
                      }
                        #endif
// 5.float
#if RX_FLOAT>0
                      idc = (uint32_t)(&(rx_pack_ptr->floats[0]));
                      idl = RX_FLOAT<<2;
                      for(i=0;i<idl;i++)
                      {
                        if(rdii>=BUFFER_SIZE)rdii -= BUFFER_SIZE;
                        (*((unsigned char *)idc))= v_rxbuff[rdii];
                            rdii++;
                            idc++;
                      }
                        #endif
                    rd_Index+=r_pack_length; // 更新读取计数
                    isok = 1;
                }
               else
                {
                  rd_Index++; // 校验值错误 则 err+1 且 更新读取计数
                  err++;
                }
            }
            else
            {
                rd_Index++;
                err++; // 包尾错误 则 err+1 且 更新读取计数
            }
        }
        else
        {
            rd_Index++; // 包头错误 则 err+1 且 更新读取计数
            err++;
        }
    }
    return isok;
}



//  void send_ValuePack(TxPack *tx_pack_ptr)
//  将发送数据结构体中的变量打包，并发送出去
//  传入参数- TxPack *tx_pack_ptr 待发送数据包的指针
//  先将待发送数据包结构体的变量转移到“发送数据缓冲区”中，然后将发送数据缓冲区中的数据发送
void send_ValuePack(Tx_Pack *tx_pack_ptr)
{
    // 数据包发送涉及的变量
  int i;
  unsigned short loop;
  unsigned char valuepack_tx_bit_index=0;
  unsigned char valuepack_tx_index=1;//  由于结构体中不同类型的变量在内存空间的排布不是严格对齐的，中间嵌有无效字节，因此需要特殊处理
  unsigned char sum=0;// 存放数据包读取的结果
  static unsigned char vp_txbuff[TX_PACK_BYTE_SIZE+3];//用于暂存发送数据的数组
  vp_txbuff[0]=0xa5;//包头

#if TX_BOOL>0
      for(loop=0;loop<TX_BOOL;loop++)
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

#if TX_BYTE>0
      for(loop=0;loop<TX_BYTE;loop++)
      {
          vp_txbuff[valuepack_tx_index] = tx_pack_ptr->bytes[loop];
          valuepack_tx_index++;
      }
    #endif

#if TX_SHORT>0
      for(loop=0;loop<TX_SHORT;loop++)
      {
          vp_txbuff[valuepack_tx_index] = tx_pack_ptr->shorts[loop]&0xff;
          vp_txbuff[valuepack_tx_index+1] = tx_pack_ptr->shorts[loop]>>8;
          valuepack_tx_index+=2;
      }
    #endif

#if TX_INT>0
      for(loop=0;loop<TX_INT;loop++)
      {
          i = tx_pack_ptr->integers[loop];
          vp_txbuff[valuepack_tx_index] = i&0xff;
          vp_txbuff[valuepack_tx_index+1] = (i>>8)&0xff;
          vp_txbuff[valuepack_tx_index+2] = (i>>16)&0xff;
          vp_txbuff[valuepack_tx_index+3] = (i>>24)&0xff;
          valuepack_tx_index+=4;
        }
    #endif

#if TX_FLOAT>0
      for(loop=0;loop<TX_FLOAT;loop++)
      {
          i = *(int *)(&(tx_pack_ptr->floats[loop]));
          vp_txbuff[valuepack_tx_index] = i&0xff;
          vp_txbuff[valuepack_tx_index+1] = (i>>8)&0xff;
          vp_txbuff[valuepack_tx_index+2] =(i>>16)&0xff;
          vp_txbuff[valuepack_tx_index+3] = (i>>24)&0xff;
          valuepack_tx_index+=4;
      }
#endif
    for(i=1;i<=TX_PACK_BYTE_SIZE;i++)sum+=vp_txbuff[i];
    vp_txbuff[TX_PACK_BYTE_SIZE+1] = sum;
    vp_txbuff[TX_PACK_BYTE_SIZE+2] = 0x5a;

		HAL_UART_Transmit_DMA(&huart7,(uint8_t *)vp_txbuff,TX_PACK_BYTE_SIZE+3);	
		HAL_Delay(1);
}




