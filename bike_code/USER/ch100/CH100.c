#include "ch100.h"

uint8_t CH100_buf[CH100_DATA_LEN] = {0};	// 帧缓存
//static int16_t	CU2(uint8_t *p) {int16_t	u; memcpy(&u, p, 2); return u;}
//static int32_t	CU4(uint8_t *p) {int32_t	u; memcpy(&u, p, 4); return u;}
static float	CR4(uint8_t *p) {float		r; memcpy(&r, p, 4); return r;}
imu_t	imu;

void CH100_USART_Init(void)
{
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);				// 使能串口接收中断
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);				// 使能串口空闲中断
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);					// 使能串口错误中断
   HAL_UART_Receive_DMA(&huart8, (uint8_t *)CH100_buf,CH100_DATA_LEN);//上位机E7 E8


}

/**
  * @brief			CRC校验
  */
static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
	uint32_t crc = *currectCrc;
	uint32_t j;
	for (j = 0; j < lengthInBytes; ++j) {
		uint32_t i;
		uint32_t byte = src[j];
		crc ^= byte << 8;
		for (i = 0; i < 8; ++i) {
			uint32_t temp = crc << 1;
			if (crc & 0x8000) {
				temp ^= 0x1021;
			}
			crc = temp;
		}
	}
	*currectCrc = crc;
}


/**
  * @brief			CH100串口通讯协议解析
  * @param[in]		data
  */
uint8_t CH100_ProcAll(uint8_t *data)
{

	uint16_t ch100_crc0 = 0;		// 数据帧中CRC
	uint16_t ch100_crc = 0;			// 收到数据计算CRC
	// 取出帧中携带CRC
	ch100_crc0 =  CH100_buf[4] | (CH100_buf[5] << 8);
	// 计算CRC
	crc16_update(&ch100_crc, CH100_buf, 4);
	crc16_update(&ch100_crc, CH100_buf + 6, CH100_DATA_LEN - CH100_DATA_HEADLEN);
	// CRC校验
	if (ch100_crc == ch100_crc0) 
    {
		// 解析串口通讯协议
		imu.wx =  CR4(CH100_buf + 18);	// 加速度XYZ
		imu.wy =  CR4(CH100_buf + 22);
		imu.wz =  CR4(CH100_buf + 26);
        
		imu.vx = CR4(CH100_buf + 30);	// 角速度XYZ
		imu.vy = CR4(CH100_buf + 34);
		imu.vz = CR4(CH100_buf + 38);
        
		imu.pit = CR4(CH100_buf + 54);		// 横滚角(Roll)
		imu.rol = CR4(CH100_buf + 58);		// 俯仰角(Pitch)
		imu.yaw = CR4(CH100_buf + 62);		// 航向角(Yaw) 
  
        //自行添加
		//if(imu.yaw<0) imu.yaw = imu.yaw+360;
		imu.vx = low_pass_filter(imu.vx); 
        
        
        
		return 1;
	} 
  else 
  {
		telem_note_imu_crc_err();
		memset(&CH100_buf, 0, sizeof(CH100_buf));
		return 0;
	}
}



void CH100_Rec(void)//放中断
{
    if (CH100_ProcAll(CH100_buf) == 1) 
    {
			//balance();
    }
    else 
    {
        HAL_UART_DMAStop(&huart8);
        HAL_UART_Receive_DMA((UART_HandleTypeDef *)&huart8, (uint8_t *)CH100_buf, (uint16_t)CH100_DATA_LEN);
    }
}

