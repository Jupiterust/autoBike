#include "include.h"

#define Button_Only


int servo_value;
float BAT;//电池电压
uint8_t Tx_Buf[14];
uint8_t TX_Flag = 0;

float speed = 1.0f;
char dir = 'M';

int upper_Flag = 1;
void simple_con(void)
{
    if(Key_Flag[0] == 1)
    {
        Serial_SendString(USART1,"(0,1)");
        Key_Flag[0] = 0;
    }

    if(Key_Flag[1] == 1)
    {
        Serial_SendString(USART1,"(1,1)");
        if(upper_Flag)
				{
					upper_Flag = 0;
				}
				else if(!upper_Flag)
				{
					upper_Flag = 1;
				}
        Key_Flag[1] = 0;
    }
    if(Key_Flag[2] == 1)
    {
        Serial_SendString(USART1,"(2,1)");
        speed+=0.1;												//后轮速度加
        Key_Flag[2] = 0;
    }
    if(Key_Flag[3] == 1)
    {
        Serial_SendString(USART1,"(3,1)");
        speed-=0.1;											//后轮速度减
        Key_Flag[3] = 0;
    }
    if(Key_Flag[6] == 1)
    {
        Serial_SendString(USART1,"(6,1)");
        Key_Flag[6] = 0;
    }
		if(!upper_Flag)
		{
			if(Key_Flag[4] == 1)
			{
					Serial_SendString(USART1,"(4,1)");//左转
					dir = 'L';
					Key_Flag[4] = 0;
			}
			if(Key_Flag[5] == 1)
			{
					Serial_SendString(USART1,"(5,1)");//右转
					
					dir = 'R';
					Key_Flag[5] = 0;
			}
			if(Key_Flag[7] == 1)
			{
					Serial_SendString(USART1,"(7,1)");//中值
				
					dir = 'M';
					Key_Flag[7] = 0;
			}
		}
		if(speed >= 2.7f)
		{
			speed = 2.7f;
		}
		if(speed <= 0.6f)
		{
			speed = 0.6f;
		}
		for(uint8_t i =0;i<8;i++)Key_Flag[i] = 0;
}
void Button_processing(void)
{
    for(uint8_t i =0;i<7;i++)
    {
        Tx_Buf[i]=Key_Flag[i];
    }
    Tx_Buf[9]=Key_Flag[7];
    if(KEY7 == 0){while(KEY7 == 0);Tx_Buf[7] ^=1;}
    if(KEY8 == 1){while(KEY8 == 1);Tx_Buf[8] ^=1;}
    for(uint8_t i =0;i<8;i++)Key_Flag[i] = 0;
}

#define NUM_SAMPLES 8  // 读取样本的数量
int8_t AD_Difference[4];
void AD_Compensate(void)
{
//    AD_GetValue();
//    AD_Difference[0] = 128 -  AD_Value[0]; 
//    AD_Difference[1] = 128 -  AD_Value[1]; 
//    AD_Difference[2] = 128 -  AD_Value[2]; 
//    AD_Difference[3] = 128 -  AD_Value[3]; 

    uint16_t sum[4] = {0};
    for (int i = 0; i < NUM_SAMPLES; i++) 
     {
        AD_GetValue();
        for (int j = 0; j < 4; j++) 
        {
            sum[j] += 128 - AD_Value[j];
        }
    }
    for (int j = 0; j < 4; j++) 
    {
        AD_Difference[j] = sum[j] / NUM_SAMPLES;
    }
}


extern bool txet_bool,txet_bool1;
extern char text_char;
extern short txet_short;
extern int txet_int;
extern float txet_float;

int main(void)
{
    sys_stm32_clock_init(9);                /* 设置时钟, 72Mhz */
    delay_init(72);                         /* 延时初始化 */
    JTAG_Set(JTAG_SWD_DISABLE);             /* 关闭JTAG的PB3 PB4 PA15的功能，作为普通I/O口使用*/
    JTAG_Set(SWD_ENABLE);                   /* 开启SWD的下载调试 */
    usart_init(72,115200);                  /* 串口初始化 */ 
    beep_init();                            /* 蜂鸣器初始化 */    
    LED_init();                             /* LED初始化 */ 
    key_init();                             /* 按键初始化 */
    OLED_Init();                            /* OLED初始化 */
    OLED_ShowString(0, 0, "2.4G ERROR Check!!!",OLED_6X8);  //初始化失败 
    OLED_Update(); 
   // NRF24L01_init();                       /* NFR24模块初始化 */ 
    OLED_Clear();                           /* 清除全屏 */
    #ifdef Button_Only
		LED0(1);
		LED1(1);
    OLED_ShowString(0, 2*8, "DIR  :",OLED_6X8); //电池显示
    OLED_ShowString(0, 3*8, "SPEED:     ",OLED_6X8); //电池显示
    OLED_ShowString(0, 4*8, "UPPER:     ",OLED_6X8); //电池显示
    #else
    AD_Init();                              /*adc初始化 */
    AD_Compensate();
    OLED_ShowString(6*6, 0, "BAT:",OLED_6X8); //电池显示
    OLED_ShowString(0, 3*8, "LR:     ",OLED_6X8); //电池显示
    OLED_ShowString(0, 4*8, "FB:     ",OLED_6X8); //电池显示
    #endif
    Time3_init(20000 - 1, 72 - 1);          /* 10ms定时器 */  
    
//        for(uint8_t i = 0;i<4;i++)
//        Tx_Buf[10+i]= 127;
    while(1)
    {
		data_treating();

			
  

        #ifdef Button_Only
//            OLED_ShowChar(6*6, 2*8, dir,OLED_6X8); //方向
//            OLED_ShowFloatNum(6*6, 3*8,speed,2,2,OLED_6X8); //显示速度
//						OLED_ShowNum(6*6,4*8,upper_Flag,1,OLED_6X8);//自动与手动切换
        #else
               if((BAT < 3.9)&&(BAT > 3.8))BEEP(1); 
                else   BEEP(0);

                OLED_ShowFloatNum(10*6, 0*8, BAT, 1, 2, OLED_6X8);     //显示电压

                OLED_ShowNum(3*6,0*8,Tx_Buf[0],1,OLED_6X8);  //显示按键按下
                OLED_ShowNum(16*6,0*8,Tx_Buf[1],1,OLED_6X8);  //显示按键按下

                OLED_ShowNum(1*6,  1*8,Tx_Buf[2],1,OLED_6X8);   //显示按键按下
                OLED_ShowNum(5*6, 1*8,Tx_Buf[3],1,OLED_6X8);  //显示按键按下
                OLED_ShowNum(14*6, 1*8,Tx_Buf[4],1,OLED_6X8);  //显示按键按下
                OLED_ShowNum(18*6,1*8,Tx_Buf[5],1,OLED_6X8); //显示按键按下

                OLED_ShowNum(3*6,2*8,Tx_Buf[6],1,OLED_6X8);  //显示按键按下
                
                OLED_ShowNum(8*6,2*8,Tx_Buf[7],1,OLED_6X8);  //显示按键按下
                OLED_ShowNum(11*6,2*8,Tx_Buf[8],1,OLED_6X8);  //显示按键按下
            
                OLED_ShowNum(16*6,2*8,Tx_Buf[9],1,OLED_6X8);  //显示按键按下

                OLED_ShowNum(3*6,3*8,Tx_Buf[10],3,OLED_6X8); //显示左摇杆档位
                OLED_ShowNum(3*6,4*8,Tx_Buf[11],3,OLED_6X8);//显示右摇杆档位  
                OLED_ShowNum(16*6,3*8,Tx_Buf[12],3,OLED_6X8); //显示左摇杆档位
                OLED_ShowNum(16*6,4*8,Tx_Buf[13],3,OLED_6X8);//显示右摇杆档位 
        #endif
//        OLED_ShowNum(0*6,5*8,txet_bool,1,OLED_6X8); 
//        OLED_ShowNum(3*6,5*8,txet_bool1,1,OLED_6X8);
//        OLED_ShowNum(12*6,5*8,text_char,3,OLED_6X8); 
//        OLED_ShowNum(0*6,6*8,txet_short,3,OLED_6X8);
//	      OLED_ShowNum(12*6,6*8,txet_int,3,OLED_6X8); 
//        OLED_ShowFloatNum(0*6,7*8,txet_float,2,2,OLED_6X8);
//					


        OLED_Update();  //更新显示   
   
    }  
}

//TIM3中断函数
void TIM3_IRQHandler(void)
{
    int16_t AD_Middle[4];
    
    if(TIM3->SR & 0X0001)   /* 溢出中断 */
    {
        Key_Scan();
        #ifdef Button_Only
            simple_con();
        #else
            AD_GetValue();                                  //获得ADC值
            BAT = (float)AD_Value[4] / 255 * 9.85;          //电池电压计算
            Button_processing();
            for(uint8_t i = 0;i<4;i++)
            {
            AD_Middle[i] = AD_Value[i]+AD_Difference[i];
            AD_Middle[i]=AD_Middle[i]>255?255:(AD_Middle[i]<1?(1):AD_Middle[i]); 
            }
            #ifdef rocker_con
            for(uint8_t i = 0;i<4;i++)
            Tx_Buf[10+i]= AD_Middle[i];
            #endif
            if(((Tx_Buf[10]>133) ||(Tx_Buf[10]<123)) || ((Tx_Buf[11]>133) ||(Tx_Buf[11]<123)) || 
                ((Tx_Buf[12]>133) ||(Tx_Buf[12]<123)) || ((Tx_Buf[13]>133) ||(Tx_Buf[13]<123))||
                        (Tx_Buf[0]!=0) ||(Tx_Buf[1]!=0)||(Tx_Buf[2]!=0) ||(Tx_Buf[3]!=0)|| 
            (Tx_Buf[4]!=0)||(Tx_Buf[5]!=0)||(Tx_Buf[6]!=0)||(Tx_Buf[9]!=0)
                    
            )TX_Flag = 1;
            else TX_Flag = 0;
            

            if((Tx_Buf[8] == 1)&&(TX_Flag == 1))
            {
                nrf24l01_tx_packet(Tx_Buf);
                LED0_TOGGLE;
                        //BEEP_TOGGLE;
            
            }
            if((TX_Flag == 1)&&(Tx_Buf[7] == 1))
            {
                LED1_TOGGLE;
                //BEEP_TOGGLE;
                
                static uint8_t count = 0;
                if((Tx_Buf[0]!=0) ||(Tx_Buf[1]!=0)||(Tx_Buf[2]!=0) ||(Tx_Buf[3]!=0)|| 
            (Tx_Buf[4]!=0)||(Tx_Buf[5]!=0)||(Tx_Buf[6]!=0)||(Tx_Buf[9]!=0))
                {
                    Tx_pack();
                }
                else count++;
                if(count++>5)
                {
                        Tx_pack();
                        count = 0;
                }

            }
        #endif
        
        
    }
    TIM3->SR &= ~(1 << 0); /* 清除中断标志位 */
}

