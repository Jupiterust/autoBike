#ifndef __PID_H
#define __PID_H


#include "A_include.h" 




float Angle_Velocity(float Gyro,float Gyro_Target);//角速度环
float X_balance_Control(float Angle,float Angle_Zero,float gyro);//角度环
float Velocity_Control(int encoder,int target_encoder);//速度环
float low_pass_filter(float value);
int Steer_Engine_control(float image_bias);
float Servo_Gain(void);
extern float Servo_Gain_K;
int Steer_Speed_Limit(float now,float limit);
float lowPassFilter(float input) ;
float Fly_Spped_Zero_Gain(float Encoder, float range, float limit, float gain);
int calculateServoLim(float Encoder);
float calculateGain(int Encoder, float servo);
float M1_Speed_Limit(float now,float limit);
	
#endif























