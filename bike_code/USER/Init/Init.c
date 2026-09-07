#include "Init.h" 

bool restart_flag = 0;


float xx = 0;
int yy = 0;

void Sys_All_Init(void)
{
    odrive_init();
	param_init();
    HAL_UART_Transmit(&huart6, (uint8_t *)"System Init OK\r\n", 16, 1000);
	
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
	
		ui_init();     //OLED bring-up: SPI1 + panel + splash

		LED0 = 1;
		LED1 = 1;
	
		//保持直立三十秒
		while(KEY_0==0);
		if(param.M0_Flag == 0)param.M0_Flag = 1;
		else param.M0_Flag = 0;
		for(uint8_t i =0;i<31;i++)
		{
			if(i%10 == 0)BEEP_ON;
			HAL_Delay(1000);BEEP_OFF;
		}
		
		
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
						 
					
						//车倒后直立的缓冲时间
						upper_Flag = 0;//在这似乎没起到作用，故在Blance//else if(param.M0_Flag==0)进行处理
						restart_flag = 1;
						ServoCtrl(Servo_Center_Mid); 
						Servo_Ctl = 0;
            if(param.M0_Flag == 0)param.M0_Flag = 1;
            else param.M0_Flag = 0;
						for(uint8_t i =0;i<2;i++)
						{
							if(i%1 == 0)BEEP_ON;
							HAL_Delay(1000);BEEP_OFF;
						}
						upper_Flag = 1;
						restart_flag = 0;
        }
#ifdef Button_Only
          //vofa
    	vofa_apply();
    	vofa_con();
          //原串口处理
#else
          uart_data_treating();
          data_treating();
#endif

    	ui_task();      //OLED refresh, main loop only - never from balance()
				
			
				
				
				
				
        //printf("hello\r\n");
        //uart_printf("%.2f,%.2f\n",odrive.now_speed0,odrive.set_speed0);
        //uart_printf("%.2f,%d\n",xx,yy);
        //uart_printf("%.2f,%.2f,%.2f,%.2f\n",imu.rol,param.angular_zero,odrive.now_speed0/10.0f,odrive.set_speed0);

    }
}

