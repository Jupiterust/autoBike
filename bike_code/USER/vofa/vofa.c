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
	if(vofa_flag)
	{ 
		switch(vofa_buff[0][0])
		{
				case '0':
					param.angular_v_kp = atof((char *)vofa_buff[1]); 
						break;
				case '1':
					param.angular_v_kd = atof((char *)vofa_buff[1]); 
						break;
				case '2':
					param.angular_kp = atof((char *)vofa_buff[1]); 
						break;
				case '3':
					param.angular_kd = atof((char *)vofa_buff[1]); 
						break;
				case '4':
					param.fly_wheel_speed_kp = atof((char *)vofa_buff[1]); 
						break;
				case '5':
					param.fly_wheel_speed_ki = atof((char *)vofa_buff[1]); 
						break;
				case '6':
									if(param.M0_Flag == 0)param.M0_Flag = 1;
									else param.M0_Flag = 0;
						break;
				case '7':
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
						break;
		}
		vofa_flag = 0;
		for(uint8_t i = 0;i<2;i++)
		{
			memset(vofa_buff[i],'\0',vofa_index[i]);
			vofa_index[i] = 0;
		}
		sprintf(vofa_send_buff,"%.3f,%.3f\r\n",param.angular_v_kp,param.angular_v_kd);
		HAL_UART_Transmit_DMA(&huart7,(uint8_t *)vofa_send_buff,strlen(vofa_send_buff));	
		HAL_Delay(1);
	}
}
