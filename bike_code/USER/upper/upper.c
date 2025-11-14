 #include "upper.h"	  
 

uint8_t upper_Flag = 0;

 
 
 
//通讯协议   
//帧头 数据帧 数据帧 校验和(低八位) 帧尾
//a5   01			01     02             5a

// 计算数据帧的校验和（低八位）
uint8_t calculateChecksum(uint8_t data1, uint8_t data2) {
    return (data1 + data2) & 0xFF; // 计算两个数据字节之和的低八位
}

// 处理接收到的字节并发送控制帧的函数
void Upper(uint8_t byte)
{
    #define STX 0xA5
    #define ETX 0x5A
    // 全局变量（或应该作为类的成员变量，根据上下文决定），用于处理UART接收缓冲区
    static uint8_t rxBuffer[5]; // 缓存接收到的帧
    static uint8_t idx = 0;     // 跟踪缓冲区中的位置
    static uint8_t receivedChecksum = 0; // 接收到的校验和

	
		static uint8_t Line_Flag = 0;//当前角度 读取标志位
		static uint8_t Left_Right_Flag = 0;//左右转向标志
		static int Servo_line = 0;////读取一次当前角度 判断当前位置
	
	
	
    // 使用 switch 语句处理不同的状态
    switch (idx) {
    case 0:
        // 如果接收到的字节是帧头
        if (byte == STX) {
            // 开始字符检测到，准备接收数据
            idx = 1;
        } else {
            // 如果不是帧头，则重置索引
            idx = 0;
        }
        break;
    case 1:
        // 存储第一个数据字节（速度控制信息）
        rxBuffer[idx] = byte;
        idx = 2;
        break;
    case 2:
        // 存储第二个数据字节（舵机角度控制信息）
        rxBuffer[idx] = byte;
        idx = 3;
        break;
    case 3:
        // 存储接收到的校验和
        receivedChecksum = byte;
        idx = 4;
        // 在这里不立即验证校验和，因为我们还没有接收到帧尾
        break;
    case 4:
        // 如果接收到的字节是帧尾
        if (byte == ETX) {
            // 计算期望的校验和
            uint8_t expectedChecksum = calculateChecksum(rxBuffer[1], rxBuffer[2]);
            // 验证校验和
            if (receivedChecksum == expectedChecksum) {
                // 校验和正确，处理数据
							
							
							
							if(rxBuffer[1] == 0x00)
							{
								
								if(Line_Flag == 0)
								{
									Line_Flag = 1;
									Servo_line = Servo_Ctl;
									if(Servo_line>=0)Left_Right_Flag = 0;
									else Left_Right_Flag = 1;
								}
								
								if(Left_Right_Flag == 1)
								{
									Servo_Ctl++;
									if(Servo_Ctl == Servo_Delta)Left_Right_Flag = 0;
								}
								else 
								{
									Servo_Ctl--;
									if(Servo_Ctl == -Servo_Delta)Left_Right_Flag = 1;
								}
								
							}
							else if(rxBuffer[1] == 0x01)Servo_Ctl = Servo_Ctl-6;
							else if(rxBuffer[1] == 0x02)Servo_Ctl = Servo_Ctl-2;
							else if(rxBuffer[1] == 0x03)Servo_Ctl = 0;
							else if(rxBuffer[1] == 0x04)Servo_Ctl = Servo_Ctl+2;
							else if(rxBuffer[1] == 0x05)Servo_Ctl = Servo_Ctl+6;							
							if(rxBuffer[1] != 0x00)Line_Flag = 0;

							
							
							
							
//							param.M1_Flag = 1;	
//							if(rxBuffer[2] == 0x00){M1_Ctl = 0;param.M1_Flag = 0;}
//							else if(rxBuffer[2] == 0x01)M1_Ctl = 0.6f;
//							else if(rxBuffer[2] == 0x02)M1_Ctl = 0.9f;
//							else if(rxBuffer[2] == 0x03)M1_Ctl = 1.2f;
//							else if(rxBuffer[2] == 0x04)M1_Ctl = 1.5f;
//							else if(rxBuffer[2] == 0x05)M1_Ctl = 1.8f;	
//							else if(rxBuffer[2] == 0x06)M1_Ctl = 2.1f;
//							else if(rxBuffer[2] == 0x07)M1_Ctl = 2.4f;
//							else if(rxBuffer[2] == 0x08)M1_Ctl = 2.7f;
//							else if(rxBuffer[2] == 0x09)M1_Ctl = 3.0f;

	
							
            }
            // 无论校验和是否正确，都重置索引以准备接收下一个帧
        } else {
            // 如果不是帧尾，则重置索引（虽然在这种情况下不太可能，但我们还是加上这个判断）
            idx = 0;
        }
        break;
    default:
        idx = 0;
        break;
    }
}

