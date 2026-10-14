#ifndef __Servo_H
#define __Servo_H	 

#include "A_include.h" 

 
#include "servo_limits.h"         /* RC actual shaft target +/-20 deg (estimate) */
#define Servo_Center_Mid 180      //舵机直行中值
#define Servo_Left_Max   (Servo_Center_Mid+Servo_Delta)      //舵机左转极限值
#define Servo_Right_Min  (Servo_Center_Mid-Servo_Delta)      //舵机右转极限值，此值跟舵机放置方式有关，立式


void Servo_Init(void);
void ServoCtrl (int duty);



#endif

















