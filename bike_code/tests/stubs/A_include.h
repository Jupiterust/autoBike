#ifndef TEST_A_INCLUDE_H
#define TEST_A_INCLUDE_H
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

typedef struct {
    uint8_t M0_Flag, M1_Flag;
    float angular_zero, angular_v_kp, angular_v_ki, angular_v_kd;
    float angular_kp, angular_ki, angular_kd;
    float fly_wheel_speed_kp, fly_wheel_speed_ki, fly_wheel_speed_kd;
    float Steer_Kp, Steer_Ki, Steer_Kd;
} param_t;
typedef struct { float set_speed1, now_speed0; } odrive_t;
typedef struct { unsigned gState; } UART_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
#define HAL_UART_STATE_READY 0u
#define Servo_Delta 120
#define Servo_Center_Mid 180
#ifdef TEST_REAL_UI
#include "ui.h"
#else
#define UI_MODE_DRIVE 0
#endif
extern param_t param;
extern odrive_t odrive;
extern UART_HandleTypeDef huart2;
extern float M1_Ctl;
extern int Servo_Ctl, Servoduty;
extern bool restart_flag;
extern volatile float odrive_vel1_turns;
extern volatile uint32_t odrive_vel1_ms;
extern volatile uint8_t odrive_vel1_valid;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
int Steer_Speed_Limit(float value, float limit);
void ServoCtrl(int value);
#ifndef TEST_REAL_UI
int ui_get_mode(void);
#endif
void ui_command(char id, const char *value);
void ui_request_redraw(void);
#ifdef TEST_REAL_UI
typedef struct { uint32_t DEMCR; } test_core_t;
typedef struct { uint32_t CYCCNT, CTRL; } test_dwt_t;
extern test_core_t test_core;
extern test_dwt_t test_dwt;
#define CoreDebug (&test_core)
#define DWT (&test_dwt)
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define UART_ERR_REMOTE 0u
#define UART_ERR_ODRIVE 1u
#define UART_ERR_IMU 2u
#define TEST_ARM_MAX_DEG 1.0f
typedef struct { float rol; } test_imu_t;
extern test_imu_t imu;
extern float Servo_Gain_K, error_zero;
extern float odrive_pos1_turns;
extern uint32_t uart_err_cnt[3], odrive_tx_drops;
extern uint32_t odrive_fb_tx, odrive_fb_ok, odrive_fb_bad, odrive_fb_eol, odrive_fb_ovf;
float odrive_odom_m(void);
uint8_t test_pulse_start(void);
uint8_t test_pulse_active(void);
void led_task(void);
void uart_rx_watchdog(void);
#endif
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, const uint8_t *, uint16_t);
#include "upper.h"
#include "vofa.h"
#endif
