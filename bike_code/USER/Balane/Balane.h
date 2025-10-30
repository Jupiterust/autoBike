#ifndef __Balane_H
#define __Balane_H	


#include "A_include.h" 


//pid参数结构体
typedef struct 
{
	  unsigned char M0_Flag;
		unsigned char M1_Flag;
			
		//平衡飞轮角速度环
    float angular_v_kp;
    float angular_v_ki;
    float angular_v_kd;
		//平衡飞轮角度环
    float angular_kp;
    float angular_ki;
    float angular_kd;
		//平衡飞轮速度环
    float fly_wheel_speed_kp;
    float fly_wheel_speed_ki;
    float fly_wheel_speed_kd;

    float angular_zero;             //角度零点

    float Steer_Kp;                 //舵机kp
    float Steer_Ki;                 //舵机ki
    float Steer_Kd;        					//舵机kd
	
		
	
		    
}paramTypeDef;

extern paramTypeDef param;
extern int Servoduty;
extern int Servo_Ctl;
extern float Servo_zhongzhi_Gain;//中值改变量
extern float error_zero;
extern float M1_Ctl;



void param_init(void);
void balance(void);




#endif

















