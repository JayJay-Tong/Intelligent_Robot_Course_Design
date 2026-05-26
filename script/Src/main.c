/**
 * 红外避障测试 (无串口版本)
 *
 * 上电 -> PC13 LED 慢闪表示待机 -> 按下 PA15 按键 -> 进入避障运动
 *
 * 运动逻辑 (左/右红外 GPIO 输入, 检测到障碍 = 低电平):
 *     左无 / 右无  -> 前进
 *     左有 / 右无  -> 右转 (左轮前进, 右轮停)
 *     左无 / 右有  -> 左转 (左轮停,  右轮前进)
 *     左有 / 右有  -> 原地右转 (左轮前进, 右轮反转)
 *
 * 调试观察:
 *     - PC13 LED 慢闪 (200ms 周期) = 待机, 等按键
 *     - PC13 LED 快闪 (50ms 周期)  = 运行中
 *     - PC13 LED 不闪              = MCU 卡死
 */

#include "main.h"
#include "tim.h"
#include "gpio.h"

#define SPEED_FWD     60    /* 前进速度 0~99 */
#define SPEED_TURN    60    /* 转弯速度 0~99 */
#define LOOP_DELAY    50    /* 主循环周期 ms (也是 LED 翻转周期) */

void SystemClock_Config(void);

/* 一次写四路 PWM 占空比 (TIM4_CH1~CH4 -> PB6/PB7/PB8/PB9 -> L298N IN1~IN4) */
static void Motor_Set(uint32_t l1, uint32_t l2, uint32_t r1, uint32_t r2)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, l1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, l2);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, r1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, r2);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* PC13 板载 LED (心跳) */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef led = {0};
    led.Pin   = GPIO_PIN_13;
    led.Mode  = GPIO_MODE_OUTPUT_PP;
    led.Pull  = GPIO_NOPULL;
    led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &led);

    /* CubeMX 生成: IR_LEFT(PA11), IR_RIGHT(PA12), KEY_START(PA15), HCSR04_TRIG(PB1) */
    MX_GPIO_Init();
    MX_TIM4_Init();

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    Motor_Set(0, 0, 0, 0);

    /* 待机: 等按键按下, LED 慢闪 */
    while (HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_SET) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(200);
    }
    HAL_Delay(50);   /* 消抖 */

    /* 运动循环 */
    while (1)
    {
        /* GPIO_PIN_RESET=0=有障碍, GPIO_PIN_SET=1=无障碍 */
        GPIO_PinState L = HAL_GPIO_ReadPin(IR_LEFT_GPIO_Port,  IR_LEFT_Pin);
        GPIO_PinState R = HAL_GPIO_ReadPin(IR_RIGHT_GPIO_Port, IR_RIGHT_Pin);

        if (L == GPIO_PIN_SET && R == GPIO_PIN_SET) {
            /* 两侧空旷 -> 前进 */
            Motor_Set(SPEED_FWD, 0, SPEED_FWD, 0);
        }
        else if (L == GPIO_PIN_RESET && R == GPIO_PIN_SET) {
            /* 左侧有障碍 -> 右转 (左轮前进, 右轮停) */
            Motor_Set(SPEED_TURN, 0, 0, 0);
        }
        else if (L == GPIO_PIN_SET && R == GPIO_PIN_RESET) {
            /* 右侧有障碍 -> 左转 (左轮停, 右轮前进) */
            Motor_Set(0, 0, SPEED_TURN, 0);
        }
        else {
            /* 正前方有障碍 -> 原地右转 (左轮前进 + 右轮反转) */
            Motor_Set(SPEED_TURN, 0, 0, SPEED_TURN);
        }

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   /* 心跳: 50ms 周期快闪 */
        HAL_Delay(LOOP_DELAY);
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
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  /* 常亮 = 出错 */
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif