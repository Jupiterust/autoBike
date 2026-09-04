#include "Odrive.h" 


OdirveTypeDef odrive;
// can2  250 kbps  250000
HAL_StatusTypeDef status1;
HAL_StatusTypeDef status2;
void odrive_init(void)
{
    odrive.set_speed0 = 0;
    odrive.set_speed1 = 0;
    odrive.now_speed0 = 0;
    odrive.now_speed1 = 0;
    odrive.fliter_speed0[0] = 0;
    odrive.fliter_speed0[1] = 0;
    odrive.fliter_speed0[2] = 0;
    odrive.fliter_speed1[0] = 0;
    odrive.fliter_speed1[1] = 0;
    odrive.fliter_speed1[2] = 0;
    odrive.speed0_i = 0;
    odrive.speed1_i = 0;

}


void odrive_canFilter_init(void)
{

	CAN_FilterTypeDef filter;
    filter.FilterActivation = ENABLE;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(&hcan2, &filter);

    status = HAL_CAN_Start(&hcan2);


    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void odrive_speed_ctrl(unsigned char num, float speed)
{
	CAN_TxHeaderTypeDef header;
	uint8_t data[8];
	header.RTR = CAN_RTR_DATA;
	header.IDE = CAN_ID_STD;
	header.DLC = 8;
	header.StdId = ((NODE_ID(num) << 5) | MSG_SET_INPUT_VEL);
	header.ExtId=0;
	uint8_t *ptrSpeed = (uint8_t *)&speed;
	data[0] = ptrSpeed[0];
	data[1] = ptrSpeed[1];
	data[2] = ptrSpeed[2];
	data[3] = ptrSpeed[3];
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	uint32_t ret;
	status1 = HAL_CAN_AddTxMessage(&hcan2, &header, data, &ret);
   

}


// 0为飞轮 1为后轮
void odrive_vel_callback(unsigned char num)
{
	CAN_TxHeaderTypeDef header;
	uint8_t data[8];
	header.RTR = CAN_RTR_REMOTE;
	header.IDE = CAN_ID_STD;
	header.DLC = 0;
	header.StdId = ((NODE_ID(num) << 5) | MSG_GET_ENCODER_ESTIMATES);
	header.ExtId = 0;
	uint32_t ret;
	status2 = HAL_CAN_AddTxMessage(&hcan2, &header, data, &ret);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef header;
	uint8_t buf[8];
	if(hcan==&hcan2)
	{
		HAL_CAN_GetRxMessage(&hcan2,  CAN_RX_FIFO0, &header, buf);
		switch (header.StdId & 0x1F)
		{
			case (MSG_GET_ENCODER_ESTIMATES):
				if((header.StdId >> 5) == AXIS0_CAN_NODE_ID)
				{
					odrive.speed0_i = (++odrive.speed0_i) % 3;
					odrive.fliter_speed0[odrive.speed0_i] = *(float *)(buf + 4);
					odrive.now_speed0 = (odrive.fliter_speed0[0]+odrive.fliter_speed0[1]+odrive.fliter_speed0[2])/3;
				}
				else if((header.StdId >> 5) == AXIS1_CAN_NODE_ID)
				{
					odrive.speed1_i = (++odrive.speed1_i) % 3;
					odrive.fliter_speed1[odrive.speed1_i] = *(float *)(buf + 4);
					odrive.now_speed1 = (odrive.fliter_speed1[0]+odrive.fliter_speed1[1]+odrive.fliter_speed1[2])/3;	
				}
				break;
			default:
				break;
		}
		
	}
}


// odrive 速度闭环 num 选择电机
void odrive_speed_ctl(unsigned char num, float speed)
{
    static uint8_t odeive_buf[12];
    odeive_buf[0] = 'v';
    odeive_buf[1] = ' ';
    if(num==0) odeive_buf[2] = '0';
    else odeive_buf[2] = '1';
    
    odeive_buf[3] = ' ';
    if(speed<0)
    {
        odeive_buf[4] = '-';
        odeive_buf[5] = (short)(-speed)%100/10+48;
        odeive_buf[6] = (short)(-speed)%10/1+48;
        odeive_buf[7] = '.';
        odeive_buf[8] = (short)(-speed*10)%10/1+48;
        odeive_buf[9] = (short)(-speed*100)%10/1+48;
        // odeive_buf[9] = (short)(-speed*1000)%10/1+48;
        // odeive_buf[10] = (short)(-speed*10000)%10/1+48;
        // odeive_buf[11] = (short)(-speed*100000)%10/1+48;
    }
    else
    {
        odeive_buf[4] = '+';
        odeive_buf[5] = (short)(speed)%100/10+48;
        odeive_buf[6] = (short)(speed)%10/1+48;
        odeive_buf[7] = '.';
        odeive_buf[8] = (short)(speed*10)%10/1+48;
        odeive_buf[9] = (short)(speed*100)%10/1+48;
        // odeive_buf[9] = (short)(speed*1000)%10/1+48;
        // odeive_buf[10] = (short)(speed*10000)%10/1+48;
        // odeive_buf[11] = (short)(speed*100000)%10/1+48;
    }
    odeive_buf[10] = 0x0D;
    odeive_buf[11] = 0x0A;
    
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)odeive_buf, 12); 
}



//获得odrive电机速度
void odrive_feedback(void)
{
    static char* msg = "f 0\r\n";
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)msg, strlen(msg)); 
}



void odrive_analyze_speed(char* msg,int len)
{

    if(len <= 15) return;    //检验
    // if(msg[len-1] != '\n') return;   
    int msg_i;
    
    for (msg_i = len-1; msg_i > 0; msg_i--) if(msg[msg_i] == '.')break;  //解析   
    if(msg_i == 0) return;
    float res = (msg[msg_i+1] - '0')*0.1 + (msg[msg_i+2] - '0')*0.01 + (msg[msg_i+3] - '0')*0.001;
    int i = 0;
    while(msg[--msg_i]!=' ')
    {
        if(msg[msg_i] == '-') res = -res;
        else res += (msg[msg_i] - '0')*pow(10,i);
        i++;
    }
    //滤波
      odrive.speed0_i = (++odrive.speed0_i) % 3;
      odrive.fliter_speed0[odrive.speed0_i] = res;
      odrive.now_speed0 = (odrive.fliter_speed0[0]+odrive.fliter_speed0[1]+odrive.fliter_speed0[2])/3;
}



//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{

//	if(huart->Instance==USART1)//如果是串口1
//	{
//		if((USART_RX_STA&0x8000)==0)//接收未完成
//		{
//			if(USART_RX_STA&0x4000)//接收到了0x0d
//			{
//				if(aRxBuffer1[0]!=0x0a)USART_RX_STA=0;//接收错误,重新开始
//				else USART_RX_STA|=0x8000;	//接收完成了 
//			}
//			else //还没收到0X0D
//			{	
//				if(aRxBuffer1[0]==0x0d)USART_RX_STA|=0x4000;
//				else
//				{
//					USART_RX_BUF1[USART_RX_STA&0X3FFF]=aRxBuffer1[0] ;
//					USART_RX_STA++;
//					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
//				}		 
//			}
//		}else{
//      int len;
//      len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
//      odrive_analyze_speed(USART_RX_BUF1, len);
//      // scope_show();
//      //  HAL_UART_Transmit(&huart2,(unsigned char*)USART_RX_BUF6,len,55);	//发送接收到的数据/
//        while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		//等待发送结束

//			USART_RX_STA=0;
//    }











