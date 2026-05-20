#ifndef STM32F10X_HC_SR04_H_
#define STM32F10X_HC_SR04_H_

#include "main.h"
#include <stdint.h>

/*
 * HC-SR04 超声波测距驱动（STM32 HAL 库版本）
 *
 * 硬件配置（CubeMX 已生成，无需修改）:
 *     TRIG -> PB1   (GPIO 推挽输出, User Label = HCSR04_TRIG)
 *     ECHO -> PA0   (TIM2_CH1 输入捕获)
 *     TIM2 : Prescaler=71  -> 1 tick = 1 us
 *            ARR=65535     -> 最大可测约 65.5 ms (远超 HC-SR04 量程)
 *            TIM2 全局中断已使能 (优先级 1)
 *
 * 使用方法:
 *     1. main() 中调用 InitHCSR04() 一次;
 *     2. 主循环中调用 HCSR04GetDistance() 即可拿到距离 (单位 mm),
 *        -1 表示超时(无回波或距离超过 ~4 m).
 *     3. 两次测量之间建议间隔 >= 60 ms 以避免回波串扰.
 *
 * 注意:
 *     - 本文件实现了 HAL_TIM_IC_CaptureCallback 弱回调, 不要在其它文件
 *       中再定义同名函数, 否则会冲突.
 *     - 触发脉冲依赖 DWT 计数器实现的微秒级延时, InitHCSR04() 已自动启用.
 */

void    InitHCSR04(void);
int32_t HCSR04GetDistance(void);

#endif /* STM32F10X_HC_SR04_H_ */
