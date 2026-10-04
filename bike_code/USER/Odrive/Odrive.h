#ifndef __Odrive_H
#define __Odrive_H	 

#include "A_include.h" 



///////////////////////////////////////////////can////////////////////////////////////////

#define AXIS1_CAN_NODE_ID (0x010) // 飞轮
#define AXIS0_CAN_NODE_ID (0x018)
#define NODE_ID(num) ((num) == 0 ? AXIS0_CAN_NODE_ID : AXIS1_CAN_NODE_ID)

typedef enum
{
    MSG_CO_NMT_CTRL = 0x000, // CANOpen NMT Message REC
    MSG_ODRIVE_HEARTBEAT,
    MSG_ODRIVE_ESTOP,
    MSG_GET_MOTOR_ERROR, // Errors
    MSG_GET_ENCODER_ERROR,
    MSG_GET_SENSORLESS_ERROR,
    MSG_SET_AXIS_NODE_ID,
    MSG_SET_AXIS_REQUESTED_STATE,
    MSG_SET_AXIS_STARTUP_CONFIG,
    MSG_GET_ENCODER_ESTIMATES,
    MSG_GET_ENCODER_COUNT,
    MSG_SET_CONTROLLER_MODES,
    MSG_SET_INPUT_POS,
    MSG_SET_INPUT_VEL,
    MSG_SET_INPUT_CURRENT,
    MSG_SET_VEL_LIMIT,
    MSG_START_ANTICOGGING,
    MSG_SET_TRAJ_VEL_LIMIT,
    MSG_SET_TRAJ_ACCEL_LIMITS,
    MSG_SET_TRAJ_A_PER_CSS,
    MSG_GET_IQ,
    MSG_GET_SENSORLESS_ESTIMATES,
    MSG_RESET_ODRIVE,
    MSG_GET_VBUS_VOLTAGE,
    MSG_CLEAR_ERRORS,
    MSG_CO_HEARTBEAT_CMD = 0x700, // CANOpen NMT Heartbeat  SEND
} OdriveMsg_t;



typedef struct
{
    float set_speed0; // 飞轮目标速度
    float set_speed1; // 后驱目标速度
    float fliter_speed0[3];
    float fliter_speed1[3];
    float now_speed0;
    float now_speed1;
    int speed0_i; //平均滤波滤波索引
    int speed1_i;
}OdirveTypeDef;

extern OdirveTypeDef odrive;

void odrive_init(void);
void odrive_canFilter_init(void);
void odrive_vel_callback(unsigned char num);
void odrive_speed_ctrl(unsigned char num, float speed);






void odrive_speed_ctl(unsigned char num, float speed);

/* Speed commands refused by the USART3 DMA because it was still busy.
 * Must stay at 0 in normal running; anything else means a motor is being
 * commanded less often than the code thinks. */
extern volatile uint32_t odrive_tx_drops;

/* ---- axis 1 (rear wheel) feedback, see Odrive.c ---------------------- */

/* Ground travelled per motor turn = rolling circumference / gear ratio.
 *
 * NOT CALIBRATED YET.  Two measurements fill it in, both with the bike on a
 * stand, neither needing the gear ratio to be looked up:
 *   1. command M1_Ctl = 1.00 (one motor turn/s), mark a spoke, count rear
 *      wheel revolutions over 30 s  ->  gear ratio = 30 / revolutions
 *   2. put it down, mark the tyre, push it exactly one wheel revolution and
 *      measure the ground distance  ->  rolling circumference (this beats
 *      pi*diameter because it includes tyre squash)
 *   DRIVE_M_PER_MOTOR_TURN = circumference / gear ratio
 *
 * Left at 0 on purpose: odrive_odom_m() and odrive_speed_mps() then return
 * a plain 0, which is obviously wrong.  A guessed constant would return
 * plausible-looking metres and quietly mislead the navigation side. */
#define DRIVE_M_PER_MOTOR_TURN  0.0f

/* Flywheel ticks between rear-wheel feedback queries (cycle is 40). */
#define ODRIVE_FB_EVERY  20u  /* M1_cnt1 runs 1..39, so this is one
                                 "f 1" per 40 frames, i.e. ~10 Hz  */

extern volatile float    odrive_pos1_turns;  /* absolute, motor turns */
extern volatile float    odrive_vel1_turns;  /* motor turns/s         */
extern volatile uint32_t odrive_fb_ok;       /* replies parsed        */
extern volatile uint32_t odrive_fb_bad;      /* replies not parsed    */
extern volatile uint32_t odrive_fb_tx;       /* queries sent          */
extern volatile uint32_t odrive_fb_rxbytes;  /* raw bytes on UART3    */
extern volatile uint32_t odrive_fb_ovf;      /* lines lost to overflow*/
extern volatile uint32_t odrive_fb_eol;      /* CR/LF bytes seen      */
extern volatile uint32_t odrive_fb_lead;     /* junk-prefix rejects   */
#define FB_MAX_LEAD_JUNK  2u  /* stray bytes tolerated before a number */

#define ODRIVE_RX_RING  256u  /* UART3 circular DMA ring, in Odrive.c   */

void  odrive_feedback_start(void);            /* arm circular RX DMA   */
void  odrive_request_feedback(uint8_t axis);  /* sends "f <axis>"      */
void  odrive_feedback_task(void);             /* drain ring + parse    */
uint8_t odrive_fb_hex(char *dst, uint8_t n, uint8_t row); /* raw hex   */
void  odrive_odom_reset(void);
float odrive_odom_m(void);
float odrive_speed_mps(void);
void odrive_feedback(void);
void odrive_init(void);
void odrive_analyze_speed(char* msg,int len);

#endif

















