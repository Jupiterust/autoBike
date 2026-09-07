#ifndef _vofa_H_
#define _vofa_H_

#include "A_include.h"

#define Button_Only  //只使用按键控制


#define VOFA_BUFF_SIZE 		256
#define VOFA_COUNT	2

extern uint8_t vofa_buff[VOFA_COUNT][VOFA_BUFF_SIZE];
extern volatile uint8_t vofa_flag;
extern volatile uint8_t vofa_index[VOFA_COUNT];

/* Automatic steering ramp state, driven by vofa_con():
 * 0 = idle, 1 = ramp back to centre, 2 = to +Servo_Delta, 3 = to -Servo_Delta. */
extern uint8_t servo_con;
 
void vofa_get(uint8_t byte);
void vofa_apply(void);

void vofa_con(void);
#endif


