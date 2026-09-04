#include "Servo.h" 


void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1); //使能PWM通道1
    ServoCtrl(Servo_Center_Mid);//舵机复位
}

void ServoCtrl (int duty)
{
    if (duty >= Servo_Left_Max) duty = Servo_Left_Max;                   //限制幅值
    else if (duty <= Servo_Right_Min) duty = Servo_Right_Min;            //限制幅值
    int res = 20000-((float)(duty*1.8)  + 500); //舵机理论范围为：0.5ms--2.5ms，大多舵机实际比这个范围小
    //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,abs(res));	
	  TIM2->CCR1=res;
}

