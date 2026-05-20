/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 迷宫机器人主程序 (HAL 库, STM32F103C8T6)
  *
  * 硬件连接：
  *   左电机   : TIM4_CH1(PB6)=正转, TIM4_CH2(PB7)=反转
  *   右电机   : TIM4_CH3(PB8)=正转, TIM4_CH4(PB9)=反转
  *   左红外   : PA11 (上拉, 检测到墙=低电平)
  *   右红外   : PA12 (上拉, 检测到墙=低电平)
  *   按键     : PA15 (上拉, 按下=低电平)
  *   超声波   : TRIG=PB1, ECHO=PA0 (TIM2_CH1 输入捕获)
  *   电子罗盘 : SCL=PB10, SDA=PB11 (I2C2)
  *   USART1   : PA9(TX) / PA10(RX), 115200 8N1 (printf 调试)
  *
  * 控制策略：增强右手法则 + 罗盘闭环
  *   - 前方 < DIST_FRONT_WALL_MM (10 cm) 视为有墙
  *   - 90° / 180° 转弯均由罗盘角度反馈结束 (超时兜底, 防罗盘异常)
  *   - 直行段以起步航向为基准做比例航向校正, 减小累计偏移
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stm32f10x_HC-SR04.h"
#include "HMC5883L.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define MOTOR_SPEED_FWD       70    /* 前进基础 PWM 占空比 0~99 */
#define MOTOR_SPEED_TURN      70    /* 转弯 PWM 占空比 */
#define MAX_TURN_MS           1200  /* 90° 转弯超时兜底 (罗盘异常时退化为定时) */
#define MAX_SPIN_MS           2200  /* 180° 掉头超时兜底 */
#define BRAKE_MS              150   /* 转弯前后停稳时间 */

#define DIST_FRONT_WALL_MM    100   /* 前方 < 10 cm 视为有墙 */

#define TURN_TARGET_DEG       80.0f /* 90° 转弯目标 (略小以补惯性) */
#define SPIN_TARGET_DEG      170.0f /* 180° 掉头目标 */

#define HEADING_HYST_DEG       5.0f /* 直行时航向偏差死区 */
#define HEADING_KP             1.5f /* 直行航向校正比例增益 */
/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void  Motor_SetPWM(uint32_t l1, uint32_t l2, uint32_t r1, uint32_t r2);
static void  Motor_Stop(void);
static void  Motor_ForwardCorrected(float heading_ref);
static float Angle_Diff(float a, float ref);
static void  Motor_TurnByCompass(int8_t dir, float target_deg, uint32_t timeout_ms);
static void  Motor_SpinByCompass(float target_deg, uint32_t timeout_ms);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static uint8_t g_compass_ok = 0;     /* HMC5883L 初始化是否成功 */

/* 同时驱动 4 路 PWM */
static void Motor_SetPWM(uint32_t l1, uint32_t l2, uint32_t r1, uint32_t r2)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, l1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, l2);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, r1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, r2);
}

static void Motor_Stop(void)
{
    Motor_SetPWM(0, 0, 0, 0);
}

/* 计算 a - ref 的最短有符号角差, 结果范围 (-180, +180] */
static float Angle_Diff(float a, float ref)
{
    float d = a - ref;
    while (d >  180.0f) d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;
    return d;
}

/* 直行 + 航向比例校正
 * heading_ref < 0 或罗盘不可用 -> 走开环直行 */
static void Motor_ForwardCorrected(float heading_ref)
{
    if (!g_compass_ok || heading_ref < 0.0f) {
        Motor_SetPWM(MOTOR_SPEED_FWD, 0, MOTOR_SPEED_FWD, 0);
        return;
    }

    float h = HMC5883L_GetHeading();
    float err = Angle_Diff(h, heading_ref);  /* 正=偏右, 负=偏左 */

    int32_t left  = MOTOR_SPEED_FWD;
    int32_t right = MOTOR_SPEED_FWD;

    if (err > HEADING_HYST_DEG) {
        /* 偏右 -> 减右轮速, 车体回左 */
        right = MOTOR_SPEED_FWD - (int32_t)(err * HEADING_KP);
        if (right < 0) right = 0;
    } else if (err < -HEADING_HYST_DEG) {
        /* 偏左 -> 减左轮速, 车体回右 */
        left  = MOTOR_SPEED_FWD - (int32_t)(-err * HEADING_KP);
        if (left  < 0) left  = 0;
    }
    Motor_SetPWM((uint32_t)left, 0, (uint32_t)right, 0);
}

/* 罗盘闭环转弯
 *   dir = +1 : 右转 (左轮前进, 右轮停)
 *   dir = -1 : 左转 (左轮停,  右轮前进)
 *   target_deg : 目标累计转过角度 (绝对值, 度)
 *   timeout_ms : 兜底超时, 罗盘异常时退化为定时转弯
 *
 * 算法: 每次循环用 Angle_Diff 累加增量, 避免 ±180° 跳变 */
static void Motor_TurnByCompass(int8_t dir, float target_deg, uint32_t timeout_ms)
{
    /* 启动转弯 PWM */
    if (dir > 0) Motor_SetPWM(MOTOR_SPEED_TURN, 0, 0, 0);   /* 右转: 左轮前进 */
    else         Motor_SetPWM(0, 0, MOTOR_SPEED_TURN, 0);   /* 左转: 右轮前进 */

    if (!g_compass_ok) {
        /* 无罗盘: 退化为开环定时, 0.4s 视作约 90° (与原参数一致) */
        HAL_Delay(dir == 0 ? 0 : 400);
        Motor_Stop();
        return;
    }

    float prev = HMC5883L_GetHeading();
    float total = 0.0f;
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        HAL_Delay(10);                      /* 给罗盘留出 ~15Hz 的更新周期 */
        float cur = HMC5883L_GetHeading();
        total += Angle_Diff(cur, prev);     /* 增量累加, 自然跨越 0°/360° */
        prev = cur;
        if (fabsf(total) >= target_deg) break;
    }
    Motor_Stop();
    printf("[Turn] dir=%+d total=%.1f deg (target %.1f)\r\n",
           dir, total, target_deg);
}

/* 原地旋转 180° (左轮前进, 右轮反转) */
static void Motor_SpinByCompass(float target_deg, uint32_t timeout_ms)
{
    Motor_SetPWM(MOTOR_SPEED_TURN, 0, 0, MOTOR_SPEED_TURN);

    if (!g_compass_ok) {
        HAL_Delay(900);
        Motor_Stop();
        return;
    }

    float prev = HMC5883L_GetHeading();
    float total = 0.0f;
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < timeout_ms)
    {
        HAL_Delay(10);
        float cur = HMC5883L_GetHeading();
        total += Angle_Diff(cur, prev);
        prev = cur;
        if (fabsf(total) >= target_deg) break;
    }
    Motor_Stop();
    printf("[Spin] total=%.1f deg (target %.1f)\r\n", total, target_deg);
}

/* USER CODE END 0 */

/**
  * @brief  应用程序入口
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C2_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();

    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    InitHCSR04();
    g_compass_ok = HMC5883L_Init();

    printf("\r\n=== Maze Robot Ready ===\r\n");
    printf("HMC5883L : %s\r\n", g_compass_ok ? "OK" : "FAIL (check I2C2 wiring/pullup)");
    printf("Press KEY (PA15) to start.\r\n");

    while (HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_SET);
    HAL_Delay(50);
    printf("Start!\r\n");

    /* 直行航向参考: 起步航向, < 0 表示未锁定 */
    float heading_ref = g_compass_ok ? HMC5883L_GetHeading() : -1.0f;
    printf("Init heading: %.1f deg\r\n", heading_ref);
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */
        GPIO_PinState L = HAL_GPIO_ReadPin(IR_LEFT_GPIO_Port,  IR_LEFT_Pin);
        GPIO_PinState R = HAL_GPIO_ReadPin(IR_RIGHT_GPIO_Port, IR_RIGHT_Pin);
        int32_t       front_mm = HCSR04GetDistance();

        /* GPIO_PIN_RESET=有墙, GPIO_PIN_SET=无墙 */
        uint8_t front_blocked = (front_mm > 0) && (front_mm < DIST_FRONT_WALL_MM);

        printf("L=%d R=%d Front=%ld mm\r\n",
               (L == GPIO_PIN_SET) ? 1 : 0,
               (R == GPIO_PIN_SET) ? 1 : 0,
               (long)front_mm);

        if (front_blocked)
        {
            Motor_Stop();
            HAL_Delay(BRAKE_MS);

            if (L == GPIO_PIN_SET) {
                Motor_TurnByCompass(-1, TURN_TARGET_DEG, MAX_TURN_MS);
            } else if (R == GPIO_PIN_SET) {
                Motor_TurnByCompass(+1, TURN_TARGET_DEG, MAX_TURN_MS);
            } else {
                Motor_SpinByCompass(SPIN_TARGET_DEG, MAX_SPIN_MS);
            }
            HAL_Delay(BRAKE_MS);
            /* 转弯后重新锁定航向参考 */
            heading_ref = g_compass_ok ? HMC5883L_GetHeading() : -1.0f;
        }
        else if (R == GPIO_PIN_SET)
        {
            /* 右手法则: 右侧出现路口优先右转 */
            Motor_Stop();
            HAL_Delay(BRAKE_MS);
            Motor_TurnByCompass(+1, TURN_TARGET_DEG, MAX_TURN_MS);
            HAL_Delay(BRAKE_MS);
            heading_ref = g_compass_ok ? HMC5883L_GetHeading() : -1.0f;
        }
        else
        {
            /* 直行 (带航向校正) */
            Motor_ForwardCorrected(heading_ref);
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
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
