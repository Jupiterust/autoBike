
#ifndef __DMA_H
#define    __DMA_H

#include "include.h"

void MYDMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar,u16 cndtr);
void dma_usart1_tx_config(uint32_t mar);                        /* 串口1 TX DMA初始化 */
void dma_enable(DMA_Channel_TypeDef *dmax_chy, uint16_t cndtr); /* 使能一次DMA传输 */
void dma_basic_config(DMA_Channel_TypeDef *dmax_chy,  uint32_t par, uint32_t mar);  /* DMA基本配置 */
#endif






























