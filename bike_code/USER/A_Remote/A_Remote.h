#ifndef _A_Remote_H_
#define _A_Remote_H_

#include "A_include.h"


// 本程序通过USART 配合接收中断 进行数据包的接收和发送
// 接收的数据在接收中断中写入到buffer中，通过定时调用read_ValuePack()函数来解析，定时间隔建议在10ms以内。

/// 1.指定接收缓冲区的大小 ----------------------------------------------------------------------------------
//    一般需要512字节以上，需要根据实际接收数据的速度和proc函数的频率考虑。
#define BUFFER_SIZE 1024

/// 2.指定发送数据包的结构--------------------------在发送时会自动额外在前后加上包头，包尾和校验和数据，因此会多出3个字节
//    根据实际需要的变量，定义数据包中 bool byte short int float 五种类型的数目

#define TX_BOOL  8
#define TX_BYTE  1
#define TX_SHORT 1
#define TX_INT   1
#define TX_FLOAT 1

/// 3.指定接收数据包的结构-----------------------------------------------------------------------------------
//    根据实际需要的变量，定义数据包中 bool byte short int float 五种类型的数目

#define RX_BOOL  8
#define RX_BYTE  1
#define RX_SHORT 1
#define RX_INT   1
#define RX_FLOAT 1

#define P_HEAD  0xa5
#define P_TAIL 0x5a


typedef struct
{
    #if TX_BOOL > 0
    unsigned char bools[TX_BOOL];
    #endif

    #if TX_BYTE > 0
    char bytes[TX_BYTE];
    #endif

    #if TX_SHORT> 0
    short shorts[TX_SHORT];
    #endif

    #if TX_INT > 0
    int  integers[TX_INT];
    #endif

    #if TX_FLOAT > 0
    float floats[TX_FLOAT];
    #endif
    char space; //只是为了占一个空，当所有变量数目都为0时确保编译成功
}
Tx_Pack;

typedef struct
{
    #if RX_BOOL > 0
    unsigned char bools[RX_BOOL];
    #endif

    #if RX_BYTE > 0
    char bytes[RX_BYTE];
    #endif

    #if RX_SHORT > 0
    short shorts[RX_SHORT];
    #endif

    #if RX_INT > 0
    int  integers[RX_INT];
    #endif

    #if RX_FLOAT > 0
    float floats[RX_FLOAT];
    #endif
    char space; //只是为了占一个空，当所有变量数目都为0时确保编译成功
}Rx_Pack;

void Tx_pack(void);
void Rx_pack(void);

// 需要保证至少每秒执行10次该函数
// 该函数的主要过程是先解析接收的缓冲区，如果接收到完整的RX数据包，则解析RX数据包中的数据，然后开始串口发送TX数据包 。
// 接收到数据包时 返回 1 ，否则返回 0
unsigned char read_ValuePack(Rx_Pack *rx_pack_ptr);
void send_ValuePack(Tx_Pack *tx_pack_ptr);// 发送数据包
void data_analysis(uint8_t data);//放串口中断里
void data_treating(void);//放while使用数据



#endif


