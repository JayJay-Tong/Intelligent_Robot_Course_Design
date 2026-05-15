/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 迷宫机器人主程序（HAL库，STM32F103C8T6）
  *
  * 硬件连接：
  *   左电机  : TIM4_CH1(PB6)=正转, TIM4_CH2(PB7)=反转
  *   右电机  : TIM4_CH3(PB8)=正转, TIM4_CH4(PB9)=反转
  *   左红外  : PA11 (上拉输入, 检测到墙=低电平)
  *   右红外  : PA12 (上拉输入, 检测到墙=低电平)
  *   启动按键: PA15 (上拉输入, 按下=低电平)
  *
  * 迷宫导航逻辑：
  *   两侧均有墙 (L=0,R=0) → 直行
  *   左侧路口   (L=1,R=0) → 左转
  *   右侧路口   (L=0,R=1) → 右转
  *   两侧均开放 (L=1,R=1) → 右转（T形/十字路口）
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define MOTOR_SPEED_FWD   70   /* 前进速度，范围 0~99 */
#define MOTOR_SPEED_TURN  70   /* 转弯速度，范围 0~99 */
#define TURN_TIME_MS      400  /* 转弯持续时间 (ms)  */
/* USER CODE END PD */

/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void Motor_SetPWM(uint32_t l1, uint32_t l2, uint32_t r1, uint32_t r2);
static void Motor_Forward(uint32_t speed);
static void Motor_TurnLeft(uint32_t speed, uint32_t ms);
static void Motor_TurnRight(uint32_t speed, uint32_t ms);
static void Motor_Stop(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/**
 * 直接设置四路PWM占空比（0~99，对应 0%~100%）
 * l1: 左电机正转  l2: 左电机反转
 * r1: 右电机正转  r2: 右电机反转
 */
static void Motor_SetPWM(uint32_t l1, uint32_t l2, uint32_t r1, uint32_t r2)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, l1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, l2);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, r1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, r2);
}

/* 直行 */
static void Motor_Forward(uint32_t speed)
{
    Motor_SetPWM(speed, 0, speed, 0);
}

/* 左转：仅右轮驱动，持续 ms 毫秒后停止 */
static void Motor_TurnLeft(uint32_t speed, uint32_t ms)
{
    Motor_SetPWM(0, 0, speed, 0);
    HAL_Delay(ms);
    Motor_Stop();
}

/* 右转：仅左轮驱动，持续 ms 毫秒后停止 */
static void Motor_TurnRight(uint32_t speed, uint32_t ms)
{
    Motor_SetPWM(speed, 0, 0, 0);
    HAL_Delay(ms);
    Motor_Stop();
}

/* 停止 */
static void Motor_Stop(void)
{
    Motor_SetPWM(0, 0, 0, 0);
}

/* USER CODE END 0 */

/**
  * @brief  应用程序入口
  */
int main(void)
{
    /* MCU 初始化 */
    HAL_Init();
    SystemClock_Config();

    /* 外设初始化 */
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    /* 启动 TIM4 四路 PWM 输出 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    /* 等待按键按下（PA15 上拉，按下为低电平）*/
    while (HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_SET);
    HAL_Delay(50);   /* 消抖 */
    /* USER CODE END 2 */

    /* 主循环 */
    while (1)
    {
        /* USER CODE BEGIN 3 */
        GPIO_PinState L = HAL_GPIO_ReadPin(IR_LEFT_GPIO_Port,  IR_LEFT_Pin);
        GPIO_PinState R = HAL_GPIO_ReadPin(IR_RIGHT_GPIO_Port, IR_RIGHT_Pin);

        /*
         * GPIO_PIN_RESET (0) = 低电平 = 检测到墙壁
         * GPIO_PIN_SET   (1) = 高电平 = 该侧无墙（出现路口）
         */
        if (L == GPIO_PIN_RESET && R == GPIO_PIN_RESET)
        {
            /* 两侧均有墙 → 直行 */
            Motor_Forward(MOTOR_SPEED_FWD);
        }
        else if (L == GPIO_PIN_SET && R == GPIO_PIN_RESET)
        {
            /* 左侧出现路口 → 左转 */
            Motor_TurnLeft(MOTOR_SPEED_TURN, TURN_TIME_MS);
        }
        else
        {
            /* 右侧路口 或 两侧均开放（T形/十字）→ 右转 */
            Motor_TurnRight(MOTOR_SPEED_TURN, TURN_TIME_MS);
        }
        /* USER CODE END 3 */
    }
}

/**
  * @brief 系统时钟配置：HSE 8MHz × PLL×9 = 72MHz
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* APB1=36MHz, TIM4=72MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
