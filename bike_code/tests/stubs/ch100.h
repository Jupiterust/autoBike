#ifndef TEST_CH100_H
#define TEST_CH100_H
#include "A_include.h"
typedef test_imu_t imu_t;
#define CH100_DATA_PRE 0x5A
#define CH100_DATA_TYPE 0xA5
#define CH100_DATA_LEN 82
#define CH100_DATA_HEADLEN 6
extern uint8_t CH100_buf[CH100_DATA_LEN];
extern UART_HandleTypeDef huart8;
uint8_t CH100_ProcAll(uint8_t *data);
void CH100_USART_Init(void);
void CH100_Rec(void);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *);
void telem_note_imu_crc_err(void);
float low_pass_filter(float value);
#endif
