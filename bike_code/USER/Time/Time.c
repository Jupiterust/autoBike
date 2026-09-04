 #include "Time.h"	

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if (htim == (&htim3))
  {
        balance();
  }
}


float Read_Speed(void)
{
    short int  Encoder;
    Encoder = TIM8 -> CNT;
    TIM8 -> CNT=0;  
    float now_speed=(float)Encoder;///10.0f;
		return  now_speed;
}


void Read_Encoder(void)
{
    short int  Encoder;
    Encoder = TIM8 -> CNT;
    TIM8 -> CNT=0;  
    odrive.speed0_i = (++odrive.speed0_i) % 3;
    odrive.fliter_speed0[odrive.speed0_i] = (float)Encoder;  
    odrive.now_speed0 = (odrive.fliter_speed0[0]+odrive.fliter_speed0[1]+odrive.fliter_speed0[2])/3;
}


#define CPU_FREQUENCY_MHZ    180		// STM32Ê±ÖÓÖ÷Æµ
void delay_us(__IO uint32_t delay)
{
    int last, curr, val;
    int temp;
 
    while (delay != 0)
    {
        temp = delay > 900 ? 900 : delay;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0)
        {
            do
            {
                val = SysTick->VAL;
            }
            while ((val < last) && (val >= curr));
        }
        else
        {
            curr += CPU_FREQUENCY_MHZ * 1000;
            do
            {
                val = SysTick->VAL;
            }
            while ((val <= last) || (val > curr));
        }
        delay -= temp;
    }
}

