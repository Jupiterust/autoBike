#include "vofa.h"



#define VOFA_BUFF_SIZE 		256
#define VOFA_COUNT	2
uint8_t vofa_buff[VOFA_COUNT][VOFA_BUFF_SIZE];
volatile uint8_t vofa_flag;                 /* set in the UART7 RX ISR */
volatile uint8_t vofa_index[VOFA_COUNT];

char vofa_send_buff[VOFA_BUFF_SIZE];
/* Frame parser for "(id,value)", called once per byte from the UART7 RX ISR.
 *
 * Two defects fixed against the original:
 *  1. neither index was bounds checked, so a stream without a ')' - noise on
 *     the line, or a handset resetting mid-frame - ran straight off the end of
 *     the 256 byte buffers and corrupted whatever globals followed them;
 *  2. the indices were only reset in vofa_apply(), after the frame had been
 *     consumed.  A frame arriving while vofa_flag was still set appended to
 *     the previous one instead of replacing it, and a dropped ')' left the
 *     state machine stuck forever.  '(' now always starts a clean frame,
 *     wherever it turns up, and newest-frame-wins.
 */
void vofa_get(uint8_t byte)
{
	static uint8_t vofa_state;

	if (byte == '(')                    /* always restarts, in any state */
	{
		vofa_index[0] = 0;
		vofa_index[1] = 0;
		vofa_state = 1;
		return;
	}

	if (vofa_state == 1)
	{
		if (byte == ',')
			vofa_state = 2;
		else if (vofa_index[0] < VOFA_BUFF_SIZE - 1)
			vofa_buff[0][vofa_index[0]++] = byte;
	}
	else if (vofa_state == 2)
	{
		if (byte == ')')
		{
			vofa_buff[0][vofa_index[0]] = '\0';
			vofa_buff[1][vofa_index[1]] = '\0';
			vofa_state = 0;
			vofa_flag = 1;
		}
		else if (vofa_index[1] < VOFA_BUFF_SIZE - 1)
			vofa_buff[1][vofa_index[1]++] = byte;
	}
}

uint8_t servo_con = 0;

/* Automatic steering ramp: 1 = back to centre, 2 = toward +Servo_Delta,
 * 3 = toward -Servo_Delta.  Only 1 is reachable from the remote (OK/stop);
 * 2 and 3 are kept for the legacy VOFA+ keys '4' and '5'.
 *
 * The original stepped Servo_Ctl by one unit and then sat in HAL_Delay(10),
 * so a full sweep to +-80 blocked the main loop for 800 ms - during which the
 * OLED froze, buttons were ignored and the RC heartbeat went unprocessed.
 * balance() runs from the UART8 ISR so the bike stayed up, but every bit of
 * interaction died.  Same 10 ms per step, deadline check instead of a delay.
 */
#define SERVO_RAMP_MS   10u

void vofa_con(void)
{
	static uint32_t next_step_ms = 0;
	uint32_t now = HAL_GetTick();

	if (servo_con == 0)
		return;

	/* was: if(!upper_Flag).  Steering is a drive-mode action; in the menu the
	 * ramp is held so an edit cannot move the servo under the rider. */
	if (ui_get_mode() != UI_MODE_DRIVE)
		return;

	if ((uint32_t)(now - next_step_ms) < SERVO_RAMP_MS)
		return;
	next_step_ms = now;

	if (servo_con == 1)                 /* centre */
	{
		if (Servo_Ctl > 0)
			Servo_Ctl--;
		else if (Servo_Ctl < 0)
			Servo_Ctl++;

		if (Servo_Ctl == 0)
			servo_con = 0;
	}
	else if (servo_con == 2)            /* toward +Servo_Delta */
	{
		if (++Servo_Ctl >= Servo_Delta)
		{
			Servo_Ctl = Servo_Delta;
			servo_con = 0;
		}
	}
	else if (servo_con == 3)            /* toward -Servo_Delta */
	{
		if (--Servo_Ctl <= -Servo_Delta)
		{
			Servo_Ctl = -Servo_Delta;
			servo_con = 0;
		}
	}
}

//#define RM
void vofa_apply(void)
{
 
	if(vofa_flag)
	{ 
//		#ifndef RM
//		switch(vofa_buff[0][0])
//		{
//				case '0':
////						if(param.M0_Flag == 0)param.M0_Flag = 1;
////						else param.M0_Flag = 0;
//					param.angular_v_kp = atof((char *)vofa_buff[1]); 
//						break;
//				case '1'://切换自动控制与手动控制
////						if(upper_Flag)
////						{
////							upper_Flag = 0;
////							M1_Ctl = 1.0f;
////						}
////						else
////						{
////							upper_Flag = 1;
////						}					
////						servo_con = 0;
//					param.angular_v_kd = atof((char *)vofa_buff[1]); 
//						break;
//				case '2':
////						if(upper_Flag)
////						{
////							upper_Flag = 0;
////							M1_Ctl = 1.0f;
////						}
////						else
////						{
////							upper_Flag = 1;
////						}					
////						servo_con = 0;
//				
//					param.angular_kp = atof((char *)vofa_buff[1]); 
//						break;
//				case '3':
//					param.angular_kd = atof((char *)vofa_buff[1]); 
//						break;
//				case '4':
////					servo_con = 2;
//					param.fly_wheel_speed_kp = atof((char *)vofa_buff[1]); 
//						break;
//				case '5':
////					servo_con = 3;
//					param.fly_wheel_speed_ki = atof((char *)vofa_buff[1]); 
//						break;
//				case '6':
//					if(param.M1_Flag == 0)
//					{
//						param.M1_Flag = 1;
//						M1_Ctl = 1.0f;
//					}
//					else
//					{
//						 param.M1_Flag = 0;
//						M1_Ctl = 0;
//					}
////					param.angular_kp = atof((char *)vofa_buff[1]); 
//						break;
//				case '7':
//					servo_con = 1;
//						break;
//		}

//		#else
		switch(vofa_buff[0][0])
		{
				case '0':
								param.angular_zero = param.angular_zero-0.01f;
//						if(param.M0_Flag == 0)param.M0_Flag = 1;
//						else param.M0_Flag = 0; 
						break;
				case '1'://切换自动控制与手动控制
						if(upper_Flag)
						{
							upper_Flag = 0;
							param.M1_Flag = 1;
							M1_Ctl = 1.5f;
						}
						else
						{
							upper_Flag = 1;
						}					
						servo_con = 0; 
						break;
				case '2': //无风版
//						M1_Ctl+=0.1f;
//						if(M1_Ctl >= 2.7f)
//						{
//							M1_Ctl = 2.7f;
//						}
						    param.angular_v_kp = 1.4;
								param.angular_v_ki = 0; 
								param.angular_v_kd = 1.115;

								param.angular_kp = -7.3;
								param.angular_ki =  0;
								param.angular_kd = -1.2;

								param.fly_wheel_speed_kp = -0.16;
								param.fly_wheel_speed_ki = -0.061;
								param.fly_wheel_speed_kd = 0;
								
								param.angular_zero = -1.27;

								param.Steer_Kp = 1;
								param.Steer_Ki = 0;
								param.Steer_Kd = 0;
						break;
				case '3'://有风版
//						M1_Ctl-=0.1f;
//						if(M1_Ctl <= 0.6f)
//						{
//							M1_Ctl = 0.6f;
//						}
							//		param.angular_v_kp = 1.1;//1.2;
								param.angular_v_kp = 1.2;//1.2;
								param.angular_v_ki = 0;
								param.angular_v_kd = 1.16;//0.5;
							
								param.angular_kp = -7.9;//-6.915;
								param.angular_ki = 0;
								param.angular_kd = -1.6;//-0.82;
							
								param.fly_wheel_speed_kp = -0.17;//-0.179;
								param.fly_wheel_speed_ki = -0.065;//-0.06;
								param.fly_wheel_speed_kd = 0;
							
								param.angular_zero = -1.27;
								
								param.Steer_Kp = 1;
								param.Steer_Ki = 0;
								param.Steer_Kd = 0;
						break;
				case '4':
						servo_con = 2; 
						break;
				case '5':
						servo_con = 3; 
						break;
				case '6':
//						if(param.M1_Flag == 0)
//						{
//							param.M1_Flag = 1;
//							M1_Ctl = 1.3f;
//						}
//						else//39
//						{
//							param.M1_Flag = 0;
//							M1_Ctl = 0;
//						}
						param.angular_zero = param.angular_zero+0.01f;
						break;
				case '7':
						servo_con = 1;
						break;
				default:
						/* Everything that is not a legacy VOFA+ digit key is a
						 * remote button; ui.h holds the id table. */
						ui_command((char)vofa_buff[0][0], (const char *)vofa_buff[1]);
						break;
		} 
//		#endif
		vofa_flag = 0;
		/* The buffers are no longer cleared here: vofa_get() resets the
		 * indices when a frame starts, which also closes the race where a
		 * frame arriving between this point and the memset lost its first
		 * bytes. */
	}
}
