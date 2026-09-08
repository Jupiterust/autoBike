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

/* ---- reproducible test disturbance (TUNING_PLAYBOOK.md section 3) ------
 * A fixed offset added to the ANGLE LOOP SETPOINT for a fixed time, so
 * every run is the identical excitation and two parameter sets can be
 * compared frame for frame.  Nothing else in the control path changes;
 * when the pulse is idle the offset is exactly 0.0f, so the arithmetic is
 * bit-identical to a build without it.
 */
/* 0.3 deg.  History: 2.0 drove error_zero to 7.84 and tripped the 5 deg cutout;
   0.5 survived but still charged the flywheel to |now_speed0| 119..153 against
   a static range of only +-30, so the measured response was dominated by the
   momentum being unloaded afterwards rather than by the angle/rate loops -
   the peak roll excursion came 670..745ms AFTER the pulse ended and was 2-3x
   the step, in the opposite direction.  0.3 should put the momentum peak back
   near the static range so the step response reflects the loops under test.
   The excitation has to stay inside the linear range or it measures the fall,
   not the loop. */
#define TEST_STEP_DEG   0.3f    /* setpoint offset while the pulse runs */
#define TEST_STEP_MS    400u    /* how long it is held                  */
/* Abort if the bike is actually diverging.  A +2 deg setpoint step drives
   error_zero to about 2 deg legitimately, so 4 deg means ~100% overshoot,
   i.e. a runaway; balance()'s own cutout still kills the motors at 5. */
#define TEST_ABORT_DEG  4.0f

/* Refuse to arm unless the bike is already settled.  Two reasons: a step
   launched from a disturbed state is not the same excitation, which breaks
   the whole "repeat the identical disturbance" premise of the tuning loop;
   and it stops a second press from stacking onto an ongoing recovery.
   Settled |error_zero| measures 0.18 deg peak, so 1.0 is not a nuisance. */
#define TEST_ARM_MAX_DEG  1.0f

/* Arm the pulse. Refused unless param.M0_Flag == 1 and the bike is settled
   (see TEST_ARM_MAX_DEG). Returns 1 if armed, 0 if refused. Call from the
   main loop (the OLED menu action), never from an ISR. */
uint8_t test_pulse_start(void);
/* Once per balance() tick: handles the timeout and the safety abort. */
void    test_pulse_update(void);
/* TEST_STEP_DEG while running, else 0.0f. */
float   test_pulse_offset(void);
/* For the telemetry TEST_ACTIVE flag, which the PC uses as t0. */
uint8_t test_pulse_active(void);




#endif

















