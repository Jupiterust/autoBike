/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "key.h"
#include "led.h"
#include "bt.h"
#include "remote.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t  oled_ack;       /* 1 = 屏在 I2C 总线上回了 ACK */
static uint32_t key_event_count; /* 收到过的按键事件总数 */
static char     last_key[4]  = "--";
static char     last_event[6] = "----";
static uint16_t last_hold_ms;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void draw_key_page(void);
static void copy_str(char *dst, const char *src, uint8_t size);
static uint32_t last_draw_ms;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 按键测试页在屏上的位置：两排各 4 个方框，左排一排、右排一排。 */
#define KP_BOX_W    16
#define KP_BOX_H    16
#define KP_X0       16          /* 第一个方框的 x */
#define KP_PITCH    22          /* 方框间距 */
#define KP_ROW_L    11          /* 左排 y */
#define KP_ROW_R    29          /* 右排 y */

static void copy_str(char *dst, const char *src, uint8_t size)
{
  uint8_t i = 0;

  while (src[i] != 0 && i < (uint8_t)(size - 1u))
  {
    dst[i] = src[i];
    i++;
  }
  dst[i] = 0;
}

/* 一个方框 = 一个按键。框里是方向字母，按下时整块反色。 */
static void draw_key_box(int16_t x, int16_t y, char label, uint8_t pressed)
{
  OLED_DrawRectangle(x, y, KP_BOX_W, KP_BOX_H, OLED_UNFILLED);
  OLED_ShowChar(x + 5, y + 4, label, OLED_6X8);

  if (pressed)
  {
    OLED_ReverseArea(x, y, KP_BOX_W, KP_BOX_H);
  }
}

/* 框里画的是这个键实际会发出去的协议 id，不是它在手柄上的方位——这样
 * 对着屏就能直接核对映射表，不用回去翻代码。未分配的键画 '-'。 */
static void draw_key_page(void)
{
  char id;

  OLED_Clear();

  OLED_ShowString(0, 0, "REMOTE", OLED_6X8);
  OLED_ShowString(60, 0, "LED", OLED_6X8);
  OLED_ShowNum(84, 0, led_get(LED0), 1, OLED_6X8);
  OLED_ShowNum(96, 0, led_get(LED1), 1, OLED_6X8);

  OLED_ShowString(2, KP_ROW_L + 4, "L", OLED_6X8);
  OLED_ShowString(2, KP_ROW_R + 4, "R", OLED_6X8);

  for (uint8_t i = 0; i < 4; i++)
  {
    int16_t x = (int16_t)(KP_X0 + i * KP_PITCH);

    id = remote_id_for_key((key_id_t)(KEY_L_UP + i));
    draw_key_box(x, KP_ROW_L, id ? id : '-', key_is_down((key_id_t)(KEY_L_UP + i)));

    id = remote_id_for_key((key_id_t)(KEY_R_UP + i));
    draw_key_box(x, KP_ROW_R, id ? id : '-', key_is_down((key_id_t)(KEY_R_UP + i)));
  }

  /* BTN = 只数按键帧、不含心跳，所以按一下就该只 +1。这是验证"一次按下发一帧"
   * 的那个数字；总帧数 TX 里混着心跳，看不出重复发送。
   * ID = 最后一帧的 id，N = 按键事件数，HB = 心跳数，
   * P = 发送队列积压（长期不为 0 说明 DMA 没排空），D = 因缓冲满丢掉的帧。 */
  /* N = 按下次数，B = 实际发出去的按键帧数。点按 10 下应该 N=10、B=10。
   *   B > N        -> 点按被当成了按住，加大 REMOTE_REPEAT_DELAY_MS
   *   N > 按的次数 -> 触点回弹，加大 KEY_DEBOUNCE_RELEASE_MS */
  OLED_ShowString(0,  47, "B:", OLED_6X8);
  OLED_ShowNum(12, 47, remote_btn_frames(), 4, OLED_6X8);
  OLED_ShowString(42, 47, "N:", OLED_6X8);
  OLED_ShowNum(54, 47, key_event_count, 4, OLED_6X8);
  OLED_ShowString(84, 47, "ID:", OLED_6X8);
  OLED_ShowChar(102, 47, remote_last_id() ? remote_last_id() : '-', OLED_6X8);

  OLED_ShowString(0,  56, "HB:", OLED_6X8);
  OLED_ShowNum(18, 56, remote_heartbeats(), 5, OLED_6X8);
  OLED_ShowString(54, 56, "P:", OLED_6X8);
  OLED_ShowNum(66, 56, bt_pending(), 3, OLED_6X8);
  OLED_ShowString(90, 56, "D:", OLED_6X8);
  OLED_ShowNum(102, 56, bt_dropped(), 3, OLED_6X8);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  /* MX_SPI1_Init(); */  /* PA5/PA7 are the OLED SCL/SDA now - see USER/oled/OLED.c */
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  
  /* USER CODE BEGIN 2 */
  /* --- OLED 上电自检 -------------------------------------------------------
   * 分三级，坏在哪一级就说明问题在哪：
   *   1. OLED_Probe()  - 地址 0x78 有没有 ACK。0 = 接线/供电/地址不对。
   *   2. 全屏点亮      - 整屏白，说明供电 + I2C + 初始化序列都通了。
   *   3. 显存          - 文字和方框。第 2 级正常而这级不正常，问题在显存/刷新，
   *                      不在硬件。
   * --------------------------------------------------------------------- */
  OLED_Init();
  oled_ack = OLED_Probe();

  OLED_Clear();                                   /* 第 2 级：整屏点亮 */
  OLED_DrawRectangle(0, 0, 128, 64, OLED_FILLED);
  OLED_Update();
  HAL_Delay(1000);

  OLED_Clear();                                   /* 第 3 级：显存绘制 */
  OLED_ShowString(0,  0, "Remote_2 OLED", OLED_6X8);
  OLED_ShowString(0,  8, "SCL PA5  SDA PA7", OLED_6X8);
  OLED_ShowString(0, 16, oled_ack ? "I2C ACK: yes" : "I2C ACK: NO!", OLED_6X8);
  OLED_Update();
  HAL_Delay(800);

  /* --- 按键 / LED ---------------------------------------------------------
   * 8 个按键的 GPIO 由 MX_GPIO_Init() 配好（上拉输入），key_init() 只清软件
   * 状态；LED 的 PB4/PB5 不在 .ioc 里，由 led_init() 自己配。
   * 扫描挂在 TIM1 更新中断上，1 ms 一次（PSC 71 / ARR 999）。
   * --------------------------------------------------------------------- */
  led_init();
  key_init();
  HAL_TIM_Base_Start_IT(&htim1);

  /* 开机把两颗灯各点一下，确认 LED_ACTIVE_HIGH 的极性配对了。 */
  led_set(LED0, 1); HAL_Delay(200); led_set(LED0, 0);
  led_set(LED1, 1); HAL_Delay(200); led_set(LED1, 0);

  /* --- 蓝牙遥控链路 --------------------------------------------------------
   * USART1 (PA9/PA10) 115200 8N1，透传到整车的 UART7。只发不收。
   * 上电就开始发心跳：整车 500 ms 收不到帧会切断动力，但在收到第一帧之前
   * 失效保护是不会触发的，所以早发晚发都安全。
   * --------------------------------------------------------------------- */
  bt_init();
  remote_init();

  OLED_Clear();
  OLED_Update();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* --- 1. 取按键事件 ---------------------------------------------------
     * 队列在中断里写、这里读，主循环被 OLED 刷新拖慢也不会漏按键。
     * ------------------------------------------------------------------ */
    key_event_t ev;

    while (key_poll(&ev))
    {
      /* 只数"按下"。之前把 UP/LONG/REPEAT 也算进去，10 次点按显示 N=20，
       * 还得心算才知道正不正常，白白误导过一次诊断。 */
      if (ev.type == KEY_EV_DOWN)
      {
        key_event_count++;
      }
      copy_str(last_key,   key_name((key_id_t)ev.id),              sizeof(last_key));
      copy_str(last_event, key_event_name((key_event_type_t)ev.type), sizeof(last_event));
      last_hold_ms = ev.hold_ms;

      /* LED1 每来一个"按下"闪 60 ms，按键有没有被识别一眼就看得出来。 */
      if (ev.type == KEY_EV_DOWN)
      {
        led_blink(LED1, 60);
      }

      /* 同一个事件也交给协议层，用来抓按下的边沿。 */
      remote_on_key_event(&ev);
    }

    /* 按住重发 + 心跳。必须每轮都调，心跳不能停。 */
    remote_task();

    /* LED0 = 当前有键按住。 */
    led_set(LED0, key_any_down());

    /* --- 2. 画面 ---------------------------------------------------------
     * 整屏刷一次要十几毫秒，按 50 ms 一帧（20 Hz）足够跟手又不占满主循环。
     * ------------------------------------------------------------------ */
    if (HAL_GetTick() - last_draw_ms >= 50u)
    {
      last_draw_ms = HAL_GetTick();
      draw_key_page();
      OLED_Update();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* TIM1 更新中断，1 ms。整个按键扫描和 LED 计时都在这里。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    key_tick();
    led_tick();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
