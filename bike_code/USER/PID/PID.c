#include "PID.h"




//角速度环pid
float Angle_Velocity(float Gyro,float Gyro_Target)
{
    float Angle_Velocity_Bias;
    float PWM_Out;
    static float Angle_Velocity_Last_Bias,Angle_Velocity_Integral;
    Angle_Velocity_Bias = Gyro - Gyro_Target;
    Angle_Velocity_Integral+=Angle_Velocity_Bias;
    if(Angle_Velocity_Integral > 10000)
			Angle_Velocity_Integral =10000;                   
    if(Angle_Velocity_Integral < -10000) 
			Angle_Velocity_Integral = -10000;        
		
    PWM_Out = param.angular_v_kp * Angle_Velocity_Bias + param.angular_v_ki * Angle_Velocity_Integral + param.angular_v_kd * (Angle_Velocity_Bias - Angle_Velocity_Last_Bias);
    Angle_Velocity_Last_Bias = Angle_Velocity_Bias;                             //保留上次误差
    return PWM_Out;
}
//角度环pid
float X_balance_Control(float Angle,float Angle_Zero,float gyro)
{
     float PWM,Bias;
     static float error;
     Bias=Angle-Angle_Zero;                                            //获取偏差
     error+=Bias;                                                      //偏差累积
     if(error>+30) error=+30;                                          //积分限幅
     if(error<-30) error=-30;                                          //积分限幅
     PWM=param.angular_kp*Bias + param.angular_ki*error + (gyro)*param.angular_kd;   //获取最终数值
     return PWM;
}
//速度环pid
float Velocity_Control(int encoder,int target_encoder)
{
    float encoder_bias,Velocity;
    static float encoder_bias_integral;
    encoder_bias = encoder - target_encoder;
    encoder_bias_integral += encoder_bias;
    if(encoder_bias_integral > +200) 
			encoder_bias_integral = +200;                    //积分限幅
    if(encoder_bias_integral < -200) 
			encoder_bias_integral = -200;                    //积分限幅
    Velocity = encoder_bias * param.fly_wheel_speed_kp/10 + encoder_bias_integral * param.fly_wheel_speed_ki/1000;
    return Velocity;
}




#define ALPHA 0.03f  // 定义滤波系数  
// 低通滤波器函数  
float low_pass_filter(float value)  
{  
    static float out_last = 0.0f; // 上一次的输出值，作为当前滤波的初始值  
    static int first_call = 1;    // 标记是否是函数的第一次调用  
  
    if (first_call) {  
        first_call = 0;           // 重置标记  
        out_last = value;         // 将第一次输入的值作为初始输出值  
    }  
  
    // 计算当前输出值  
    float out = out_last + ALPHA * (value - out_last);  
    out_last = out;               // 更新上一次的输出值  
  
    return out;  
}  


//舵机pid
int Steer_Engine_control(float image_bias)
{
    int steer_out;
    static float Last_image_bias;
    static float bias_intergral;
    bias_intergral += image_bias;
    if(bias_intergral>=50) bias_intergral = 50;
    if(bias_intergral<=-50) bias_intergral = -50;
    steer_out = param.Steer_Kp*image_bias+ param.Steer_Ki * bias_intergral + param.Steer_Kd*(image_bias-Last_image_bias);
    Last_image_bias=image_bias;
    return steer_out;
}

//舵机打角限速
int Steer_Speed_Limit(float now,float limit)
{
    static int last;
    int out;
    
    if ((now - last) >= limit) out =  last + limit; 
    else if ((now - last) <= -limit)out =  last - limit;
    else out = now;
   
    last = out; //记录上次打角值
    return out;
}



float M1_Speed_Limit(float now,float limit)
{
    static float last;
    int out;
    
    if ((now - last) >= limit) out =  last + limit; 
    else if ((now - last) <= -limit)out =  last - limit;
    else out = now;
   
    last = out; //记录上次打角值
    return out;
}






/////////////////////////////////舵机打脚控制零点  以及速度辅助平衡//////////////////////////////////



// 不同角度下的零点校准增益
 const float Servo_ZERO[20] = {
    -0.0010,-0.0052,-0.0058,  -0.0061,-0.0068,//4
    -0.0069,-0.0066,  -0.0061, -0.0062, -0.0063,//9    
    
    -0.0010,-0.0024,-0.0037, -0.0049, -0.0051,  
    -0.0053, -0.0055,-0.0057, -0.00585, -0.0060, //19
};


float Servo_Gain(void)
{
    uint8_t gain_index;//索引
    if (Servo_Ctl > 90) gain_index = 9;
    else if (Servo_Ctl > 80) gain_index = 8;
    else if (Servo_Ctl > 70) gain_index = 7;
    else if (Servo_Ctl > 60) gain_index = 6;
    else if (Servo_Ctl > 50) gain_index = 5;
    else if (Servo_Ctl > 40) gain_index = 4;
    else if (Servo_Ctl > 30) gain_index = 3;
    else if (Servo_Ctl > 20) gain_index = 2;
    else if (Servo_Ctl > 10) gain_index = 1;
    else if (Servo_Ctl > 0) gain_index = 0;

    else if (Servo_Ctl < -90) gain_index = 19;
    else if (Servo_Ctl < -80) gain_index = 18;
    else if (Servo_Ctl < -70) gain_index = 17;
    else if (Servo_Ctl < -60) gain_index = 16;
    else if (Servo_Ctl < -50) gain_index = 15;
    else if (Servo_Ctl < -40) gain_index = 14;
    else if (Servo_Ctl < -30) gain_index = 13;
    else if (Servo_Ctl < -20) gain_index = 12;
    else if (Servo_Ctl < -10) gain_index = 11;
    else if (Servo_Ctl < 0) gain_index = 10;

		float output = Servo_Ctl * 0.0015f + 
						 Servo_ZERO[gain_index] * Servo_Ctl;
		return output;
}



// 滤波器参数
#define FILTER_CONSTANT 0.001f // 滤波系数，根据截止频率和采样频率计算得到
/**
 * @brief 一阶低通滤波器函数
 * @param input 新的输入值
 * @return 滤波后的值
 */
float lowPassFilter(float input) 
{
    static float filteredValue = 0.0f; // 滤波后的输出值
    // 应用一阶低通滤波器的递推公式
    filteredValue = (FILTER_CONSTANT * input) + ((1 - FILTER_CONSTANT) * filteredValue);
    return filteredValue;
}

/**
 * @brief 计算飞控速度零增益
 * @param Encoder 编码器值
 * @param range 增益变化范围
 * @param limit 增益限制
 * @param gain 增益系数
 * @return 计算后的增益
 */
float Fly_Spped_Zero_Gain(float Encoder, float range, float limit, float gain) 
{
    static float last_gain = 0; // 记录上次的增益

    float Out_Gain;

    // 对编码器进行滤波
    //Encoder = lowPassFilter(Encoder);

    // 对编码器进行增益计算
    Out_Gain = Encoder * gain;

    // 对增益变化进行限制
    float delta_gain = Out_Gain - last_gain;
    if (delta_gain > range) {Out_Gain = last_gain + range;} 
		else if (delta_gain < -range) {Out_Gain = last_gain - range;}

    // 对增益进行限幅
    if (Out_Gain > limit) {Out_Gain = limit;} 
		else if (Out_Gain < -limit) {Out_Gain = -limit;}

    last_gain = Out_Gain; // 记录本次的增益
    return Out_Gain; // 返回计算后的增益
}





// 定义常数
#define MAX_SERVO_LIM 5.0
#define A 4.99
#define B 0.005
// 由于分母1-e^(-2)是常数，我们可以预先计算它以避免在每次函数调用时都计算它
#define DENOMINATOR (1 - exp(-2))

int calculateServoLim(float Encoder)
{
	int out = MAX_SERVO_LIM - A * (1 - exp(-B * Encoder)) / DENOMINATOR;
	if(out<1)out=1;
	return out;
}










// 定义增益输出的范围  
#define MIN_GAIN -0.50f  
#define MAX_GAIN 0.50f  
  
// 定义编码器速度和舵机的范围  
#define ENCODER_RANGE 200  
#define SERVO_DEADBAND 15  
  
// 函数定义  
float calculateGain(int Encoder, float servo) {  
    float gain = 0.0;  
    // 检查舵机是否在死区范围内  
    if (abs(servo) <= SERVO_DEADBAND) {  
        return 0.0;  
    }  
  
    // 计算编码器的绝对值，用于增益计算  
    int absEncoder = abs(Encoder);  
  
    // 根据逻辑决定增益的符号和大小  
    if ((servo > 0 && Encoder < 0) || (servo < 0 && Encoder > 0)) {  
        // 需要减速  
        gain = -fabs((float)absEncoder / ENCODER_RANGE * MAX_GAIN);  
    } else if ((servo > 0 && Encoder > 0) || (servo < 0 && Encoder < 0)) {  
        // 需要加速  
        gain = fabs((float)absEncoder / ENCODER_RANGE * MAX_GAIN);  
    }  
    // 确保增益在范围内  
    if (gain > MAX_GAIN) {  
        gain = MAX_GAIN;  
    } else if (gain < MIN_GAIN) {  
        gain = MIN_GAIN;  
    }  
  
    return gain;  
}



