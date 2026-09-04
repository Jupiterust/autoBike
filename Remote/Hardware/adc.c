#include "adc.h"
#include "delay.h"
/***********************************
ADC1_CH2---PA2 左摇杆左右
ADC1_CH1---PA1 左摇杆上下
ADC1_CH3---PA3 右摇杆左右
ADC1_CH6---PA6 右摇杆上下
ADC1_CH4---PA4 电源检测
************************************/

uint16_t AD_Value[5];//定义数组存放ADC值，依次存放AD2,AD1,AD3,AD6,AD4

    

    

//ADC扫描模式+DMA数据转运
void AD_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//开启ADC1的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//开启GPIOA的时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//开启DMA1时钟
    
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//6分频，ADCCLK=72MHz/6=12MHz,最高14MHz只能选6分频或8分频
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//模拟输入，在AIN模式下GPIO口是无效的，防止GPIO口的输入输出对模拟电压造成干扰
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);//初始化GPIO
    
    ADC_RegularChannelConfig(ADC1,ADC_Channel_2,1,ADC_SampleTime_55Cycles5);//ADC规则组通道设置，ADC1，通道3，序列1，采样时间
    ADC_RegularChannelConfig(ADC1,ADC_Channel_1,2,ADC_SampleTime_55Cycles5);//通过参数ADC_Channel改变通道，实现多个IO口的ADC获取
    ADC_RegularChannelConfig(ADC1,ADC_Channel_3,3,ADC_SampleTime_55Cycles5);//通过参数ADC_Channel改变通道，实现多个IO口的ADC获取
    ADC_RegularChannelConfig(ADC1,ADC_Channel_6,4,ADC_SampleTime_55Cycles5);//通过参数ADC_Channel改变通道，实现多个IO口的ADC获取
    ADC_RegularChannelConfig(ADC1,ADC_Channel_4,5,ADC_SampleTime_55Cycles5);//通过参数ADC_Channel改变通道，实现多个IO口的ADC获取
    
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//外部触发源选择，不使用外部触发，就是软件触发
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//单次转换
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;//扫描模式
    ADC_InitStructure.ADC_NbrOfChannel = 5;//通道数目
    ADC_Init(ADC1,&ADC_InitStructure);//初始化ADC
    
    
    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;//起始地址参数传值
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//数据宽度16位半字
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//地址不自增，始终转运同一个位置的数据
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value;//存储器起始地址
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//数据宽度半字
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//地址自增
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//外设地址作为源，传输方向由外设到存储器
    DMA_InitStructure.DMA_BufferSize = 5;//传输次数
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//正常模式
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//硬件触发，触发源为ADC1
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
    DMA_Init(DMA1_Channel1,&DMA_InitStructure);//DMA初始化
    
    DMA_Cmd(DMA1_Channel1,ENABLE);//使能
    ADC_DMACmd(ADC1,ENABLE);//ADC1开启DMA输出信号
    
    ADC_Cmd(ADC1,ENABLE);//开启ADC1
    
    ADC_ResetCalibration(ADC1);//复位校准
    while(ADC_GetResetCalibrationStatus(ADC1) == SET);//等待复位校准完成
    ADC_StartCalibration(ADC1);//开始校准
    while(ADC_GetCalibrationStatus(ADC1) == SET);//等待校准完成
}

void AD_GetValue( void)
{
    DMA_Cmd(DMA1_Channel1,DISABLE);//失能
    DMA_SetCurrDataCounter(DMA1_Channel1,5);//给传输计数器写数据
    DMA_Cmd(DMA1_Channel1,ENABLE);//使能
    
    ADC_SoftwareStartConvCmd(ADC1,ENABLE);//软件触发
    
    while(DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);//等待传输完成
    DMA_ClearFlag(DMA1_FLAG_TC1);//清除标志位
    
    for(uint8_t i=0;i<5;i++)
    {
      AD_Value[i]=AD_Value[i]/16 ;
    }  
}






