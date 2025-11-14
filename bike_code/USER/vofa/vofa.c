#include "vofa.h"



#define VOFA_BUFF_SIZE 		256
#define VOFA_COUNT	2
uint8_t vofa_buff[VOFA_COUNT][VOFA_BUFF_SIZE];
uint8_t vofa_flag;
uint8_t vofa_index[VOFA_COUNT];

char vofa_send_buff[VOFA_BUFF_SIZE];
void vofa_get(uint8_t byte)
{
	static uint8_t vofa_state;
	if(vofa_state == 0)
	{
			if(byte == '(')
			{
					vofa_state = 1;
			}
	}
	else if(vofa_state == 1)
	{
			if(byte == ',')
			{
					vofa_state = 2;
			}
			else
			{
					vofa_buff[0][vofa_index[0]++] = byte;
			}
	}
	else if(vofa_state == 2)
	{
			if(byte == ')')
			{
					vofa_state = 0;
					vofa_flag = 1;
			}
			else
			{
					vofa_buff[1][vofa_index[1]++] = byte;
			}
	}
}
void vofa_apply(void)
{
	static uint8_t servo_con = 0;
	if(!upper_Flag)//上位机处于失能
	{
		if(servo_con == 0)
		{
				if(Servo_Ctl == 0)
				{
					Servo_Ctl = 0;
				}
				else if(Servo_Ctl < 0)
				{
					Servo_Ctl++;
					HAL_Delay(10);
				}
				else if(Servo_Ctl > 0)
				{
					Servo_Ctl--;
					
					HAL_Delay(10);
				}
		}
		else if(servo_con == 1)
		{
//			Servo_Ctl = -45;
			Servo_Ctl ++;
			HAL_Delay(10);
			if(Servo_Ctl >= 50)
			{
				Servo_Ctl = 50;
			}
		}
		else if(servo_con == 2)
		{
//			Servo_Ctl = 45;
			Servo_Ctl --;
			HAL_Delay(10);
			if(Servo_Ctl <= -50)
			{
				Servo_Ctl = -50;
			}
		}
	}
	if(vofa_flag)
	{ 
		switch(vofa_buff[0][0])
		{
				case '0':
						if(param.M0_Flag == 0)param.M0_Flag = 1;
						else param.M0_Flag = 0;
//					param.angular_v_kp = atof((char *)vofa_buff[1]); 
						break;
				case '1'://切换自动控制与手动控制
						if(upper_Flag)
						{
							upper_Flag = 0;
						}
						else if(!upper_Flag)
						{
							upper_Flag = 1;
						}
//					param.angular_v_kd = atof((char *)vofa_buff[1]); 
						break;
				case '2':
						
						M1_Ctl+=0.1f;
						if(M1_Ctl >= 2.7f)
						{
							M1_Ctl = 2.7f;
						}
				case '3':
						M1_Ctl-=0.1f;
						if(M1_Ctl <= 0.6f)
						{
							M1_Ctl = 0.6f;
						}
//					param.angular_kd = atof((char *)vofa_buff[1]); 
						break;
				case '4':
					servo_con = 1;
//					param.fly_wheel_speed_kp = atof((char *)vofa_buff[1]); 
						break;
				case '5':
					servo_con = 2;
//					param.fly_wheel_speed_ki = atof((char *)vofa_buff[1]); 
						break;
				case '6':
					if(param.M1_Flag == 0)
					{
						param.M1_Flag = 1;
						M1_Ctl = 1.0f;
					}
					else
					{
						 param.M1_Flag = 0;
						M1_Ctl = 0;
					}
//					param.angular_kp = atof((char *)vofa_buff[1]); 
						break;
				case '7':
					servo_con = 0;
						break;
		}
		vofa_flag = 0;
		for(uint8_t i = 0;i<2;i++)
		{
			memset(vofa_buff[i],'\0',vofa_index[i]);
			vofa_index[i] = 0;
		}
//		sprintf(vofa_send_buff,"%.3f,%.3f\r\n",param.angular_v_kp,param.angular_v_kd);
//		HAL_UART_Transmit_DMA(&huart7,(uint8_t *)vofa_send_buff,strlen(vofa_send_buff));	
//		HAL_Delay(1);
	}
}
