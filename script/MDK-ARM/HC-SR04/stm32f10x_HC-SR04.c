/**
 * HC-SR04 超声波测距驱动 (HAL 库版本)
 *
 * 工作流程:
 *     1) TRIG 拉高 >=10us 触发模块, 模块发出 8 个 40kHz 超声波脉冲;
 *     2) ECHO 输出高电平, 时长 = 超声波往返飞行时间;
 *     3) TIM2_CH1 输入捕获在 ECHO 上升沿记 t1, 切换到下降沿捕获记 t2;
 *     4) 距离(mm) = (t2 - t1) us * 声速(0.343 mm/us) / 2 ≈ diff * 0.1715
 *        等价计算: diff * 1715 / 10000  (避免浮点)
 */

#include "stm32f10x_HC-SR04.h"
#include "tim.h"

/* ---- 捕获状态 ---------------------------------------------------------- */
static volatile uint32_t rise_tick     = 0;
static volatile uint32_t fall_tick     = 0;
static volatile uint8_t  capture_state = 0;   /* 0=等上升沿, 1=等下降沿 */
static volatile uint8_t  capture_done  = 0;

/* ---- DWT 微秒延时 (Cortex-M3 内核计数器) ------------------------------- */
static void DWT_DelayInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { /* spin */ }
}

/* ---- 对外接口 ---------------------------------------------------------- */
void InitHCSR04(void)
{
    DWT_DelayInit();
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);

    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

int32_t HCSR04GetDistance(void)
{
    uint32_t t_start;
    uint32_t diff;

    /* 复位状态机 */
    capture_state = 0;
    capture_done  = 0;
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);

    /* 发送 >=10us 触发脉冲 (留 15us 余量) */
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
    delay_us(15);
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);

    /* 等待两次捕获完成, 最长 40ms (HC-SR04 最大回波 ~23ms @ 4m) */
    t_start = HAL_GetTick();
    while (!capture_done)
    {
        if ((HAL_GetTick() - t_start) > 40U) {
            return -1;
        }
    }

    /* 处理一次溢出 (ARR=65535) */
    if (fall_tick >= rise_tick) {
        diff = fall_tick - rise_tick;
    } else {
        diff = (0xFFFFu - rise_tick) + fall_tick + 1u;
    }

    return (int32_t)((diff * 1715U) / 10000U);   /* mm */
}

/* ---- TIM2_CH1 输入捕获中断回调 ---------------------------------------- */
/* TIM2_IRQHandler -> HAL_TIM_IRQHandler -> 本回调 (HAL 库弱符号已被覆盖) */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;
    if (htim->Channel  != HAL_TIM_ACTIVE_CHANNEL_1) return;

    if (capture_state == 0) {
        rise_tick     = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        capture_state = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
    } else {
        fall_tick     = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        capture_state = 0;
        capture_done  = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
    }
}
