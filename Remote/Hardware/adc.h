#ifndef __ADC_H
#define __ADC_H

#include "include.h"

extern uint16_t AD_Value[5];
extern uint16_t AV_ADC_Channel1_Sample;	// ADC_CH1平均值
extern uint16_t AV_ADC_Channel3_Sample;	// ADC_CH3平均值

void AD_Init(void);
void AD_GetValue( void);

#endif 















