/**
 * 迷宫导航 (右手法则, 双红外, 无串口)
 *
 * 上电 -> PC13 慢闪 (待机) -> 按 PA15 -> 进入迷宫导航
 *
 * 右手法则状态转移 (基于左右红外):
 *   左有墙 + 右有墙 -> 前进     (在走廊内, 两壁夹行)
 *   左无墙 + 右有墙 -> 左转 90° (左侧出现路口, 转入)
 *   左有墙 + 右无墙 -> 右转 90° (右侧出现路口, 转入)
 *   左无墙 + 右无墙 -> 右转 90° (T 形 / 十字, 右手法则优先)
 *
 * 注: 无前方检测, 死路靠 "双侧均无墙" 间接推断, 实际碰到死胡同会撞墙.
 * 若迷宫有死胡同, 需要再加超声波或撞针. 课程展示的迷宫一般不带死路.
 */

#include "main.h"
#include "tim.h"
#include "gpio.h"

#define SPEED_FWD    50    /* 前进 PWM 0~99, 太慢爬不动可加到 55~60 */
#define SPEED_TURN   55    /* 转弯 PWM, 比前进略大一点克服静摩擦 */
#define TURN_MS      550   /* 90° 转弯时间; 速度变了这里要等比例调 */
#define BRAKE_MS     100   /* 转弯前后停顿, 让车体停稳 */
#define POST_TURN_MS 200   /* 转完后稍微往前一点, 清出弯道 */

void SystemClock_Config(void);

/* 一次写 4 路 PWM (TIM4_CH1~4 -> PB6/PB7/PB8/PB9 -> L298N IN1~IN4) */
static void Motor_Set(uint32_t l1, uint32_t l2, uint32_t r1, uint32_t r2)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, l1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, l2);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, r1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, r2);
}

static void Motor_Forward(void)
{
    Motor_Set(SPEED_FWD, 0, SPEED_FWD, 0);
}

static void Motor_Stop(void)
{
    Motor_Set(0, 0, 0, 0);
}

/* 左转: 左轮停, 右轮前进, 持续 TURN_MS */
static void Motor_TurnLeft(void)
{
    Motor_Stop();              HAL_Delay(BRAKE_MS);
    Motor_Set(0, 0, SPEED_TURN, 0);
    HAL_Delay(TURN_MS);
    Motor_Stop();              HAL_Delay(BRAKE_MS);
    /* 转完往前一点, 避免重复检测同一个路口 */
    Motor_Forward();           HAL_Delay(POST_TURN_MS);
}

/* 右转: 右轮停, 左轮前进, 持续 TURN_MS */
static void Motor_TurnRight(void)
{
    Motor_Stop();              HAL_Delay(BRAKE_MS);
    Motor_Set(SPEED_TURN, 0, 0, 0);
    HAL_Delay(TURN_MS);
    Motor_Stop();              HAL_Delay(BRAKE_MS);
    Motor_Forward();           HAL_Delay(POST_TURN_MS);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* PC13 板载 LED */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef led = {0};
    led.Pin   = GPIO_PIN_13;
    led.Mode  = GPIO_MODE_OUTPUT_PP;
    led.Pull  = GPIO_NOPULL;
    led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &led);

    MX_GPIO_Init();    /* IR_LEFT/IR_RIGHT/KEY_START/HCSR04_TRIG */
    MX_TIM4_Init();

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    Motor_Stop();

    /* 待机: 等按键, LED 慢闪 */
    while (HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_SET) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(200);
    }
    HAL_Delay(50);   /* 消抖 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);   /* 进入运行 = 常亮 */

    /* 主循环 */
    while (1)
    {
        /* GPIO_PIN_RESET = 检测到墙, GPIO_PIN_SET = 该侧无墙 (路口) */
        GPIO_PinState L = HAL_GPIO_ReadPin(IR_LEFT_GPIO_Port,  IR_LEFT_Pin);
        GPIO_PinState R = HAL_GPIO_ReadPin(IR_RIGHT_GPIO_Port, IR_RIGHT_Pin);

        if (L == GPIO_PIN_RESET && R == GPIO_PIN_RESET) {
            /* 两侧均有墙 -> 走廊正常行驶 */
            Motor_Forward();
        }
        else if (L == GPIO_PIN_SET && R == GPIO_PIN_RESET) {
            /* 左侧路口 -> 左转 */
            Motor_TurnLeft();
        }
        else {
            /* 右侧路口, 或 T形/十字路口 -> 右转 (右手法则优先) */
            Motor_TurnRight();
        }

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   /* 心跳指示 */
    }
}

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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef led = {0};
    led.Pin   = GPIO_PIN_13;
    led.Mode  = GPIO_MODE_OUTPUT_PP;
    led.Pull  = GPIO_NOPULL;
    led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &led);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif