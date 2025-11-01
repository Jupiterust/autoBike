#ifndef _vofa_H_
#define _vofa_H_

#include "A_include.h"


#define VOFA_BUFF_SIZE 		256
#define VOFA_COUNT	2

extern uint8_t vofa_buff[VOFA_COUNT][VOFA_BUFF_SIZE];
extern uint8_t vofa_flag;
extern uint8_t vofa_index[VOFA_COUNT];
 
void vofa_get(uint8_t byte);
void vofa_apply(void);
#endif


