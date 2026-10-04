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
/* Number of speed commands that never reached the wire because the USART3 DMA
 * was still busy.  This used to be invisible: the HAL_UART_Transmit_DMA()
 * return value was discarded, so a refused transfer silently did nothing. */
volatile uint32_t odrive_tx_drops = 0;

void odrive_speed_ctl(unsigned char num, float speed)
{
    /* One buffer PER AXIS, not one shared buffer.  A 12 byte frame at 460800
     * takes 260 us to clock out and DMA reads straight from this memory, so a
     * single shared buffer let the next call rewrite a frame that was still
     * in flight. */
    static uint8_t  odeive_buf[2][12];
    /* Last value actually sent, and when.  balance() calls this on 39 of
     * every 40 IMU frames - about 360 commands a second - but set_speed0 is
     * only recomputed by the angular-rate loop every 6 frames, so four fifths
     * of that was the same number over and over.  ODrive parses every one of
     * them in a thread that shares its UART, and its sink drops the rest of a
     * response when its own TX is still busy: that is what truncated every
     * reply to 10 bytes.  Skipping unchanged commands changes nothing about
     * the control - ODrive holds the last velocity it was given. */
    static uint8_t  last_sent[2][6];
    static uint32_t last_ms[2] = {0u, 0u};
    static uint8_t  primed[2]  = {0u, 0u};

    uint8_t  ax = (num == 0) ? 0u : 1u;
    uint8_t *b  = odeive_buf[ax];
    uint8_t  f[6];
    uint32_t now = HAL_GetTick();

    /* Format into a scratch copy first: b may still be in flight, and
     * rewriting it before deciding to skip would corrupt that frame. */
    if (speed < 0)
    {
        f[0] = '-';
        f[1] = (short)(-speed)%100/10+48;
        f[2] = (short)(-speed)%10/1+48;
        f[3] = '.';
        f[4] = (short)(-speed*10)%10/1+48;
        f[5] = (short)(-speed*100)%10/1+48;
    }
    else
    {
        f[0] = '+';
        f[1] = (short)(speed)%100/10+48;
        f[2] = (short)(speed)%10/1+48;
        f[3] = '.';
        f[4] = (short)(speed*10)%10/1+48;
        f[5] = (short)(speed*100)%10/1+48;
    }

    /* Resend an unchanged command every ODRIVE_CMD_HEARTBEAT_MS anyway, so a
     * single dropped frame cannot leave ODrive on a stale setpoint. */
    if (primed[ax]
     && memcmp(last_sent[ax], f, sizeof f) == 0
     && (uint32_t)(now - last_ms[ax]) < ODRIVE_CMD_HEARTBEAT_MS)
        return;

    b[0] = 'v';
    b[1] = ' ';
    b[2] = (num == 0) ? '0' : '1';
    b[3] = ' ';
    memcpy(&b[4], f, sizeof f);
    b[10] = 0x0D;
    b[11] = 0x0A;

    if (HAL_UART_Transmit_DMA(&huart3, b, 12) != HAL_OK)
    {
        odrive_tx_drops++;      /* HAL_BUSY: the previous frame is still going */
        return;                 /* not sent, so do not record it as sent */
    }

    memcpy(last_sent[ax], f, sizeof f);
    last_ms[ax] = now;
    primed[ax]  = 1u;
}

/* ====================================================================== *
 * Axis 1 (rear wheel) feedback over the same UART3 ASCII link.
 *
 * Until now the rear wheel was open loop: the firmware knew what speed it had
 * COMMANDED and nothing else.  The competition needs better than that - the
 * red light has to be stopped for inside a 0.5 m zone and the finish line
 * needs the rear wheel to stop on it - so both speed and distance have to be
 * measured.  ODrive already has a 8192 cpr encoder on that axis; "f <axis>"
 * returns "<pos_estimate> <vel_estimate>", in turns and turns/s.
 *
 * Deliberately separate from the old odrive_feedback()/odrive_analyze_speed()
 * pair below, which are dead code: they are hard-wired to axis 0 and write
 * odrive.now_speed0 - but that variable holds the FLYWHEEL speed read from
 * TIM8 in encoder counts per IMU frame, which every tuned gain depends on.
 * Letting ODrive turns/s land in it would silently rescale the whole speed
 * loop.
 * ====================================================================== */

volatile float    odrive_pos1_turns = 0.0f;   /* absolute, motor turns   */
volatile float    odrive_vel1_turns = 0.0f;   /* motor turns/s           */
volatile uint32_t odrive_fb_ok      = 0;
volatile uint32_t odrive_fb_bad     = 0;      /* unparseable replies     */
volatile uint32_t odrive_fb_tx      = 0;      /* queries actually sent   */
volatile uint32_t odrive_fb_rxbytes = 0;      /* raw bytes seen on UART3 */
volatile uint32_t odrive_fb_ovf     = 0;      /* lines lost to overflow  */
volatile uint32_t odrive_fb_eol     = 0;      /* CR or LF bytes seen     */
volatile uint32_t odrive_fb_lead    = 0;      /* too much junk to skip   */

static char    fb_line[48];                   /* filled by the RX ISR    */
static uint8_t fb_len = 0;
static char    fb_last[48] = "";              /* last line, for the display */
/* The first line that would not parse, latched and never overwritten: a
   30 Hz display cannot be read, and one bad sample settles the format. */
static char    fb_err[48] = "";

/* Bring-up: one latched window of the raw stream, in hex.  Inferring the
   reply format from counters has cost two rounds already.  49 bytes spans
   more than two replies, which is what shows whether the odd byte seen at
   offset 10 recurs on a period - i.e. whether bytes are being dropped in
   bursts - rather than leaving it to be guessed at. */
#define FB_RAW_N  49u    /* 7 display rows x 7 bytes */
static volatile uint8_t fb_raw[FB_RAW_N];
static volatile uint8_t fb_raw_n    = 0;
static volatile uint8_t fb_raw_cap  = 0;
static volatile uint8_t fb_raw_done = 0;

/* UART3 receives into this ring by circular DMA; the main loop drains it.
   At 19 bytes a reply and 40 replies a second it holds ~340 ms. */
static uint8_t  rx_ring[ODRIVE_RX_RING];
static uint16_t rx_tail = 0;
static uint8_t  fb_first_drain_done = 0;
static volatile uint8_t fb_ready = 0;

static float odom_origin_turns = 0.0f;

/* Which value the outstanding query asked for: 0 = position, 1 = speed. */
static volatile uint8_t fb_expect = 1u;

/* "f <axis>" returns both values in one reply, but that reply is 19
   characters and only the first 10 survive, so the speed was always cut
   off.  Two single-value queries fit: "-30.000000" is exactly 10.  They
   alternate, giving 5 Hz each at the 10 Hz query rate. */
void odrive_request_feedback(uint8_t axis)
{
    static uint8_t q[32];
    static const char *tail[2] = { "pos_estimate\r\n", "vel_estimate\r\n" };

    uint8_t next = (uint8_t)(fb_expect ^ 1u);
    uint8_t n;

    memcpy(q, "r axis0.encoder.", 16);
    q[6] = axis ? '1' : '0';
    n = (uint8_t)strlen(tail[next]);
    memcpy(&q[16], tail[next], n);
    n = (uint8_t)(16u + n);

    if (HAL_UART_Transmit_DMA(&huart3, q, n) == HAL_OK)
    {
        fb_expect = next;
        odrive_fb_tx++;
    }
    else
    {
        odrive_tx_drops++;
    }
}

/* One byte, straight from the UART3 RX interrupt.  Keep it short. */

/* Hand-rolled rather than atof(): that drags in newlib strtod for ~5.6 KB,
   and this only ever has to read ODrive's own "%f %f" output. */
static const char *fb_parse_float(const char *s, float *out)
{
    float v = 0.0f, frac = 0.1f;
    uint8_t neg = 0;

    /* Replies arrive with a stray byte in front of the number - a 'c' in
       every sample captured so far.  ODrive writes the value into a
       char[10] in cmd_read_property() and then hands that buffer to
       respond() as a FORMAT string, while fibre_over_uart writes straight
       to the same uart_tx_stream that the ASCII protocol reaches through a
       multiplexer; something in there leaks a byte.  Skip it, but only a
       byte or two, so "invalid motor 1" is still rejected rather than read
       as the number 1. */
    {
        uint8_t skipped = 0;
        while (*s != '\0' && skipped <= FB_MAX_LEAD_JUNK
            && !((*s >= '0' && *s <= '9') || *s == '-' || *s == '+'))
        {
            s++;
            skipped++;
        }
        if (skipped > FB_MAX_LEAD_JUNK)
        {
            odrive_fb_lead++;
            return 0;
        }
    }

    if (*s == '-') { neg = 1; s++; }
    else if (*s == '+') s++;

    if (*s < '0' || *s > '9') return 0;

    while (*s >= '0' && *s <= '9')
        v = v * 10.0f + (float)(*s++ - '0');

    if (*s == '.')
    {
        s++;
        while (*s >= '0' && *s <= '9')
        {
            v += (float)(*s++ - '0') * frac;
            frac *= 0.1f;
        }
    }

    *out = neg ? -v : v;
    return s;
}

/* Called once a terminator arrives.  This runs in main-loop context now, so
   there is no ISR/main handover left to get wrong. */
static void fb_line_done(void)
{
    float v = 0.0f;

    if (fb_len == 0u)
        return;

    fb_line[fb_len] = '\0';
    memcpy(fb_last, fb_line, (size_t)fb_len + 1u);

    if (fb_parse_float(fb_line, &v) == 0)
    {
        if (fb_err[0] == '\0')
            memcpy(fb_err, fb_line, sizeof fb_err);
        odrive_fb_bad++;
        return;
    }

    if (fb_expect == 0u)
    {
        odrive_pos1_turns = v;
    }
    else
    {
        odrive_vel1_turns = v;
        odrive.now_speed1 = v;
    }
    odrive_fb_ok++;
}

static void fb_feed(uint8_t b)
{
    if (fb_raw_cap && fb_raw_n < FB_RAW_N)
    {
        fb_raw[fb_raw_n++] = b;
        if (fb_raw_n >= FB_RAW_N) { fb_raw_cap = 0; fb_raw_done = 1; }
    }
    if (b == '\n' || b == '\r')
        odrive_fb_eol++;

    odrive_fb_rxbytes++;

    /* ODrive truncates each reply at 10 characters, so the CR/LF never
       arrives - odrive_fb_eol stays 0 and is kept as the probe for that.
       0x21 does arrive on every reply, and ODrive's own ASCII parser
       treats '!' as end-of-line, so it is a protocol character here and
       not noise.  Split on it. */
    if (b == '\n' || b == '\r' || b == '!')
    {
        fb_line_done();
        fb_len = 0;
        return;
    }

    fb_line[fb_len++] = (char)b;

    /* Overflow means this line is lost - the UART came up mid-reply, or the
       ring was drained late and wrapped.  Throw it away and resynchronise on
       the next terminator. */
    if (fb_len >= (uint8_t)(sizeof fb_line - 1u))
    {
        fb_len = 0;
        odrive_fb_ovf++;
    }
}

/* Circular DMA: reception is armed once and never needs re-arming, and the
   DMA controller services RXNE within a few cycles, so the control loop
   hogging the CPU can no longer cost bytes. */
void odrive_feedback_start(void)
{
    rx_tail = 0;
    HAL_UART_Receive_DMA(&huart3, rx_ring, ODRIVE_RX_RING);
}

void odrive_feedback_task(void)
{
    uint16_t head;

    if (huart3.hdmarx == 0)
        return;

    head = (uint16_t)(ODRIVE_RX_RING - __HAL_DMA_GET_COUNTER(huart3.hdmarx));
    if (head >= ODRIVE_RX_RING)
        return;                 /* mid-reload; next pass will see it */

    /* The first pass skips whatever piled up while the boot countdown had the
       main loop blocked: those bytes are stale and the ring has wrapped over
       them.  It is also the right moment to arm the raw window. */
    if (!fb_first_drain_done)
    {
        fb_first_drain_done = 1;
        rx_tail = head;
        fb_raw_n = 0;
        fb_raw_cap = 1;
        return;
    }

    while (rx_tail != head)
    {
        fb_feed(rx_ring[rx_tail]);
        if (++rx_tail >= ODRIVE_RX_RING)
            rx_tail = 0;
    }
}

/* Bring-up only: hand the partially assembled line to the display, so the
   bytes that ARE arriving can be read directly instead of guessed at. */
/* One display row of the window, 7 bytes as hex.  Returns how many bytes
   have been captured in all, so the caller can show the window length. */
uint8_t odrive_fb_hex(char *dst, uint8_t n, uint8_t row)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t i, o = 0, from = (uint8_t)(row * 7u);

    for (i = from; i < (uint8_t)(from + 7u) && i < fb_raw_n
                && (uint8_t)(o + 3u) < n; i++)
    {
        dst[o++] = hex[(fb_raw[i] >> 4) & 0xfu];
        dst[o++] = hex[fb_raw[i] & 0xfu];
        dst[o++] = ' ';
    }
    dst[o] = '\0';
    return fb_raw_n;
}

void odrive_odom_reset(void)
{
    odom_origin_turns = odrive_pos1_turns;
}

float odrive_odom_m(void)
{
    return (odrive_pos1_turns - odom_origin_turns) * DRIVE_M_PER_MOTOR_TURN;
}

float odrive_speed_mps(void)
{
    return odrive_vel1_turns * DRIVE_M_PER_MOTOR_TURN;
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











