#ifndef __CH100_H
#define __CH100_H

#include "A_include.h"

// Statement
#define CH100_DATA_PRE		(0x5A)
#define CH100_DATA_TYPE		(0xA5)
#define CH100_DATA_LABEL1	(0x91)
#define CH100_DATA_ID		1
#define CH100_DATA_HEADLEN	6
#define CH100_DATA_LEN		82

typedef struct
{

	float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
	float wy;
	float wz;

	float vx;
	float vy;
	float vz;

	float rol;
	float pit;
	float yaw;
} imu_t;
extern imu_t	imu;
extern imu_t imu;

extern uint8_t CH100_buf[CH100_DATA_LEN];	// ÷°ª∫¥Ê

uint8_t CH100_ProcAll(uint8_t *data);
void CH100_USART_Init(void);
void CH100_Rec(void);//∑≈÷–∂œ

#endif























