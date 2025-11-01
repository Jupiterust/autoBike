#include "Init.h" 


float xx = 0;
int yy = 0;

void Sys_All_Init(void)
{
    odrive_init();
	  param_init();
    
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL); //编码器 I5 I6
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);       //蜂鸣器	 H6
    Servo_Init();                                   //舵机	 A0
    //odrive_canFilter_init();                      //can 未使用
    //HAL_UART_Receive_DMA(&huart3, (uint8_t *)USART_RX_BUF2,USART_RX_BUF2);  //串口控制电机  不需要接受
    HAL_UART_Receive_DMA(&huart6, (uint8_t *)vp_rxbuff,VALUEPACK_BUFFER_SIZE);//蓝牙G9 G14
	  HAL_UART_Receive_IT(&huart2, (uint8_t *)USART_RX_BUF1, USART_RX_LEN1);    //上位机 D5 D6
    HAL_UART_Receive_IT(&huart7,(uint8_t *)USART_RX_BUF2, USART_RX_LEN2);     //E7 E8
    CH100_USART_Init();                                                       //串口8 陀螺仪2.5ms
		//HAL_TIM_Base_Start_IT(&htim3);                                          //使用陀螺仪中断即可    2ms定时中断
	
		LED0 = 1;
		LED1 = 1;
	
		//保持直立三十秒
//		while(KEY_0==0);
		if(param.M0_Flag == 0)param.M0_Flag = 1;
		else param.M0_Flag = 0;
//		for(uint8_t i =0;i<31;i++)
//		{
//			if(i%10 == 0)BEEP_ON;
//			HAL_Delay(1000);BEEP_OFF;
//		}
		
		
		LED0 = 0;
		LED1 = 0;
		upper_Flag = 1;//使能上位机控制
    while(1)
    {
			
			  if(KEY_0)
        {
            BEEP_ON;
            HAL_Delay(20);
            while(KEY_0);
            BEEP_OFF;
            if(param.M0_Flag == 0)param.M0_Flag = 1;
            else param.M0_Flag = 0;
        }
        

//原串口处理
//				uart_data_treating();
//				data_treating();
				
				//vofa
				vofa_apply();
				
				
			
				
				
				
				
        //printf("hello\r\n");
        //uart_printf("%.2f,%.2f\n",odrive.now_speed0,odrive.set_speed0);
        //uart_printf("%.2f,%d\n",xx,yy);
        //uart_printf("%.2f,%.2f,%.2f,%.2f\n",imu.rol,param.angular_zero,odrive.now_speed0/10.0f,odrive.set_speed0);

    }
}

