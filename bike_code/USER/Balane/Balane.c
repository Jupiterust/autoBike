#include "Balane.h" 


#define fly_wheel_rate_limit 18  //动量轮速度限幅

extern bool restart_flag;


paramTypeDef param;
int Servoduty = Servo_Center_Mid; // 全局中间量   
float Servo_zhongzhi_Gain = 0;	  //舵机中值偏移补偿
float Fly_Gain = 0;//动量轮零点补偿
float M1_gain = 0;//平衡速度增益
float error_zero = 0;//实际误差
float Servo_Lim = 1.0;//舵机限制
int Servo_Ctl=0;//舵机控制量
float M1_Ctl = 0.0;//后轮速度控制

//float M1_wheel_rate_limit = 3.0;//动量轮速度限幅
#define M1_wheel_rate_limit 3;//动量轮速度限幅



void param_init(void)
{
	
		param.M0_Flag =0;
		param.M1_Flag =0;

//有风版	

//		param.angular_v_kp = 1.2;//1.2;
//		param.angular_v_ki = 0;
//		param.angular_v_kd = 1.16;//0.5;
//	
//		param.angular_kp = -7.9;//-6.915;
//		param.angular_ki = 0;
//		param.angular_kd = -1.6;//-0.82;
//	
//		param.fly_wheel_speed_kp = -0.17;//-0.179;
//		param.fly_wheel_speed_ki = -0.065;//-0.06;
//		param.fly_wheel_speed_kd = 0;
//	
//param.angular_zero = -1.165;
//		
//		param.Steer_Kp = 1;
//		param.Steer_Ki = 0;
//		param.Steer_Kd = 0;  



//无风版
    param.angular_v_kp = 1.4;
    param.angular_v_ki = 0; 
    param.angular_v_kd = 1.115;

    param.angular_kp = -7.3;
    param.angular_ki =  0;
    param.angular_kd = -1.2;

    param.fly_wheel_speed_kp = -0.16;
    /* -0.081, was -0.061.  iter2 (2026-09-08) kept it: at a given balance-point
       offset the residual flywheel idle speed dropped ~18% (d(idle)/d(offset)
       -117.9 -> -96.1) with no cost in roll std, saturation or the slow mode.
       This is the no-wind set only; the windy set below keeps its own value. */
    param.fly_wheel_speed_ki = -0.081;
    param.fly_wheel_speed_kd = 0;
    
//    param.angular_zero = -1.55 + 0.39;
//		param.angular_zero = -1.33;
		/* Mechanical zero, re-measured 2026-09-08.  -1.27 was integrated on the
		   previous mechanical configuration and is no longer valid: the bike now
		   balances at about -1.40.  Estimated by regressing d(now_speed0)/dt
		   against imu_rol (the roll at which the flywheel stops being forced to
		   accelerate); the settled window telem_20260908_210718 gave -1.4007 with
		   the best correlation of the session, and idle speed +0.37, roll std
		   0.028, steady-state angle error -0.003 - the cleanest state recorded so
		   far.  This matters mainly for the 31s upright countdown, which runs
		   before the menu can be reached.  Expect it to move again if the mount
		   changes: mount-to-mount scatter measured about +-0.013. */
		param.angular_zero = -1.40;
    param.Steer_Kp = 1;
    param.Steer_Ki = 0;
    param.Steer_Kd = 0;
	
	
	
}

const float slope = 0.00625f; // 斜率，计算为 (1.8 - 0.8) / (80 - (-80))
const float intercept = 1.8f; // 截距，当Servo_Ctl为0时的M1_wheel_rate_limit
 

/* ---- reproducible test disturbance, see Balane.h ---------------------- */
static volatile uint8_t  test_active   = 0;
static volatile uint32_t test_start_ms = 0;

uint8_t test_pulse_start(void)
{
    /* A held OK repeats at ~10Hz (see REMOTE_PROTOCOL.md), and re-arming would
       refresh test_start_ms and stretch the pulse.  Measured 2026-09-08: one
       run came out 500ms instead of 400ms, exactly one repeat interval longer,
       which breaks the "every run is the identical excitation" premise the
       whole comparison rests on.  Ignore requests while one is running. */
    if (test_active)
        return 0;

    if (param.M0_Flag != 1)
        return 0;           /* no point perturbing an open loop */

    if (fabs(error_zero) > TEST_ARM_MAX_DEG)
        return 0;           /* not settled: the run would not be comparable */

    /* Deadline first, then arm: balance() runs from the UART8 ISR and can
       land between these two writes.  In this order it would see
       test_active still 0 and do nothing, which is harmless; the other
       order would let it run one tick off a stale start time. */
    test_start_ms = HAL_GetTick();
    test_active   = 1;
    return 1;
}

void test_pulse_update(void)
{
    if (!test_active)
        return;

    if (param.M0_Flag != 1
        || fabs(error_zero) > TEST_ABORT_DEG
        || (uint32_t)(HAL_GetTick() - test_start_ms) >= TEST_STEP_MS)
        test_active = 0;
}

float test_pulse_offset(void)
{
    return test_active ? TEST_STEP_DEG : 0.0f;
}

uint8_t test_pulse_active(void)
{
    return test_active;
}

void balance(void)
{
    static uint16_t cnt = 0;//角度环计数
    static uint16_t cnt1=0;//速度环计数
    static uint16_t M1_cnt1=0;//速度环计数
    static float PWM_X,PWM_accel;// PWM中间量

    cnt++;	
    cnt1++;
    M1_cnt1++;
    Read_Encoder();//滤波读取速度
    //odrive.now_speed0 = Read_Speed();//直接读取速度
		error_zero = imu.rol-param.angular_zero;
		test_pulse_update();   //test step: timeout + safety abort, uses error_zero
	
/************舵机控制**************************************************/	
    Servo_Ctl=Servo_Ctl>Servo_Delta?Servo_Delta:(Servo_Ctl<-Servo_Delta?(-Servo_Delta):Servo_Ctl); //舵机限幅
    Servo_Lim = calculateServoLim(fabs(odrive.now_speed0));//平衡优先舵机
    Servo_Ctl = Steer_Speed_Limit(Servo_Ctl,Servo_Lim); // 舵机打角限速  防止突变
    if(param.M0_Flag==1 && !restart_flag)
    {
       Servoduty = Servo_Center_Mid + Servo_Ctl;
       ServoCtrl(Servoduty); 
    }
    else if(param.M0_Flag==0)
		{
			ServoCtrl(Servo_Center_Mid); 
			Servo_Ctl = 0;
			upper_Flag = 0;
		}
/************平衡控制**************************************************/	
		Servo_zhongzhi_Gain = Servo_Gain();//舵机零点偏移平衡增益
    if(cnt1>=60){cnt1=0;PWM_accel = Velocity_Control(odrive.now_speed0 , 0);}                              
    if(cnt>=6)
    {
        cnt=0; 
				Fly_Gain = Fly_Spped_Zero_Gain(odrive.now_speed0,0.01,0.3,0.0005);//编码器 变化限幅 最大值 增益P   动量轮转速 零点偏移
				PWM_X = X_balance_Control(imu.rol,param.angular_zero +PWM_accel+Servo_zhongzhi_Gain+Fly_Gain+test_pulse_offset(),imu.vx);
				
				//角度环调参
//				PWM_X = X_balance_Control(imu.rol,0,imu.vx);
    }	          
    odrive.set_speed0 = Angle_Velocity(imu.vx,PWM_X);   
//    odrive.set_speed0 = PWM_X;  
    odrive.set_speed0 = odrive.set_speed0>fly_wheel_rate_limit?fly_wheel_rate_limit:(odrive.set_speed0<-fly_wheel_rate_limit?(-fly_wheel_rate_limit):odrive.set_speed0); // 动量轮电机限幅																																																			 

//调参时先增大保护范围
//		if(fabs(error_zero)>10) { param.M0_Flag=0;param.M1_Flag =0;}   
//    if(param.M0_Flag==0)odrive.set_speed0= 0;
		
   if(fabs(error_zero)>5) { param.M0_Flag=0;param.M1_Flag =0;}   
    if(param.M0_Flag==0)odrive.set_speed0= 0;
		
    /* Skip axis 0 on the tick that sends axis 1.  Both share one USART3 DMA:
       a 12 byte frame takes 260us to clock out, the axis 1 call below lands a
       few us later, and HAL_UART_Transmit_DMA() then returns HAL_BUSY and
       sends nothing - so the rear wheel was never actually commanded.  The
       flywheel loses one update in 40 (2.5ms) and is re-sent on the next
       tick, which it cannot notice. */
    if(M1_cnt1 < 40)
    odrive_speed_ctl(0,odrive.set_speed0);    //odrive_speed_ctrl(0,odrive.set_speed0);//can
/************后轮控制**************************************************/	
    if(M1_cnt1 >= 40)
    {
        M1_cnt1=0;
        if(param.M1_Flag == 1)
        {
						M1_gain = calculateGain(odrive.now_speed0, Servo_Ctl);//速度辅助动量轮增益
						if(M1_Ctl <= 0.5f)M1_gain = 0;//太小了不使能速度增益

//						if(abs(Servo_Ctl)>75)M1_wheel_rate_limit =1.0;
//						else if(abs(Servo_Ctl)>60)M1_wheel_rate_limit =1.3;
//						else if(abs(Servo_Ctl)>45)M1_wheel_rate_limit =1.6;
//						else if(abs(Servo_Ctl)>30)M1_wheel_rate_limit =2.0;
//						else if(abs(Servo_Ctl)>15)M1_wheel_rate_limit =2.5;
//						else if(abs(Servo_Ctl)>0)M1_wheel_rate_limit =3.0;					
					
					
						//M1_Ctl = M1_Ctl>M1_wheel_rate_limit?M1_wheel_rate_limit:(M1_Ctl<-M1_wheel_rate_limit?(-M1_wheel_rate_limit):M1_Ctl);	
					
						//M1_Ctl = M1_Speed_Limit(M1_Ctl,0.2);
					
            odrive.set_speed1 = M1_Ctl + M1_gain;//基础控制
						if(param.M0_Flag==0)
						{
					  	odrive.set_speed1 = 0;
							param.M1_Flag = 0;
						}
        }
        else odrive.set_speed1 = 0;

				odrive_speed_ctl(1,odrive.set_speed1);        //odrive_speed_ctrl(1,odrive.set_speed1);//can
    }
/************END**************************************************/	
    /* telemetry: control output is computed, clamped and dispatched by here */
    telem_task();
    
}





