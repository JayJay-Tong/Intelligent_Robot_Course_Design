/**
 * 迷宫小车 - 纯 IR 导航 (无磁力计)
 *
 * 直线: 两侧 IR 贴墙修正
 * 转弯: 任一侧 IR 持续丢墙 -> C 型 (两次同向 90°), 用 IR 重新看到墙做对齐
 * 死路: 超声波 < 10cm -> 原地左转 90° (右手法则)
 *
 * 引脚:
 *   IR_LEFT  = PA11 (pull-up; 看到墙 = LOW)
 *   IR_RIGHT = PA12 (pull-up; 看到墙 = LOW)
 *   超声波 TRIG=PB1, ECHO=PA0
 *   电机 PWM (TIM4): CH1=左前 CH2=左后 CH3=右前 CH4=右后
 *   按键 KEY_START = PA15
 *   LED = PC13
 */

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "stm32f10x_HC-SR04.h"

/* ---- 可调参数 ---------------------------------------------------------- */
#define BASE_SPEED          60      /* 巡航基础 PWM (0~99) */
#define VERIFY_DIFF         8       /* 验证模式下的微调差速 */
#define TURN_SPEED          45      /* 原地 90° 旋转 PWM */
#define U_TURN_PWM          60      /* 180° 掉头 PWM, 略大于 TURN_SPEED 提供更强差速 */
#define VERIFY_THRESH       7       /* 连续 N 次没找回墙就判为真缺口 (~210ms) */
#define MIN_SPIN_MS         250     /* 转弯最少先转这么久, 避免初始即时返回 */
#define SPIN_TIMEOUT_MS     2200    /* 单次旋转最长保护时间 */
#define APPROACH_MS         800     /* 确认缺口后, 先往前走的预进时间, 让车尾过墙角 */
#define C_FWD_MS            1100    /* C 型中间前进时长 */
#define U_TURN_180_MS       1500    /* 死路 180° 掉头时长 (定时, 需现场校准) */
#define FRONT_STOP_MM       100      /* 前方阈值 */
#define LOOP_MS             30      /* 巡航主循环节拍 */
#define SPIN_POLL_MS        10      /* 旋转时 IR 轮询节拍 */

/* IR 读取宏: 引脚 LOW 表示看到墙 */
#define IR_L_WALL()  (HAL_GPIO_ReadPin(IR_LEFT_GPIO_Port,  IR_LEFT_Pin)  == GPIO_PIN_RESET)
#define IR_R_WALL()  (HAL_GPIO_ReadPin(IR_RIGHT_GPIO_Port, IR_RIGHT_Pin) == GPIO_PIN_RESET)

void SystemClock_Config(void);

/* ---- 电机原语 --------------------------------------------------------- */
static void motor_drive(int l_pwm, int r_pwm)
{
    if (l_pwm < 0)  l_pwm = 0;  if (l_pwm > 99) l_pwm = 99;
    if (r_pwm < 0)  r_pwm = 0;  if (r_pwm > 99) r_pwm = 99;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint16_t)l_pwm);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint16_t)r_pwm);
}

static void motor_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}

static void motor_spin_right(uint8_t pwm)   /* 顺时针 (俯视) */
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm);   /* 左轮前 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm);   /* 右轮后 */
}

static void motor_spin_left(uint8_t pwm)    /* 逆时针 (俯视) */
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pwm);   /* 左轮后 */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);   /* 右轮前 */
}

/* ---- 旋转直到两侧 IR 都看到墙 (= 对齐到新走廊) ----------------------- */
/* dir = +1 顺时针(右), -1 逆时针(左) */
static void spin_until_both_walls(int dir)
{
    if (dir > 0) motor_spin_right(TURN_SPEED);
    else         motor_spin_left(TURN_SPEED);

    HAL_Delay(MIN_SPIN_MS);                  /* 先转够最小时间, 避开初始状态 */

    uint32_t t_start = HAL_GetTick();
    while ((HAL_GetTick() - t_start) < SPIN_TIMEOUT_MS) {
        if (IR_L_WALL() && IR_R_WALL()) break;
        HAL_Delay(SPIN_POLL_MS);
    }
    motor_stop();
    HAL_Delay(150);
}

/* ---- 死路 180° 掉头 (定时, 左轮正向 + 右轮反向, 差速比 90° 转大) ---- */
static void u_turn_180(void)
{
    motor_spin_right(U_TURN_PWM);    /* CH1=左轮前进 CH4=右轮后退 */
    HAL_Delay(U_TURN_180_MS);
    motor_stop();
    HAL_Delay(200);
}

/* ---- C 型转向: 先预进 -> 转 -> 前进 -> 再转 -------------------------- */
static void c_turn(int dir)
{
    /* 预进: 让整个车身越过墙角, 避免转弯时车尾撞墙 */
    motor_drive(BASE_SPEED, BASE_SPEED);
    HAL_Delay(APPROACH_MS);
    motor_stop();
    HAL_Delay(100);

    spin_until_both_walls(dir);                  /* 第 1 个 90° */
    motor_drive(BASE_SPEED, BASE_SPEED);
    HAL_Delay(C_FWD_MS);                         /* 穿过 U 底部 */
    motor_stop();
    HAL_Delay(150);
    spin_until_both_walls(dir);                  /* 第 2 个 90° */
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();

    /* PC13 LED */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef led = {0};
    led.Pin = GPIO_PIN_13; led.Mode = GPIO_MODE_OUTPUT_PP;
    led.Pull = GPIO_NOPULL; led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &led);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    /* 电机 PWM */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    motor_stop();

    /* 超声波 */
    InitHCSR04();

    /* 等 KEY_START */
    while (HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_SET) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(200);
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(300);

    /* ---- 巡航主循环 ------------------------------------------------- */
    int verify_r = 0;
    int verify_l = 0;
    int skip_next_gap = 0;   /* 刚掉头, 下一个缺口直走不转 */
    int in_skip_pass  = 0;   /* 正在穿过被跳过的缺口, 等两侧墙都回来再恢复 */

    while (1)
    {
        uint8_t r_wall = IR_R_WALL();
        uint8_t l_wall = IR_L_WALL();
        int32_t dist   = HCSR04GetDistance();

        /* 正在穿过被跳过的缺口: 只直行, 等两侧墙回来 */
        if (in_skip_pass) {
            motor_drive(BASE_SPEED, BASE_SPEED);
            if (r_wall && l_wall) {
                in_skip_pass = 0;
                verify_r = verify_l = 0;
            }
            HAL_Delay(LOOP_MS);
            continue;
        }

        /* 优先: 前方堵了 -> 死路掉头 180° */
        if (dist > 0 && dist < FRONT_STOP_MM) {
            motor_stop();
            HAL_Delay(100);
            u_turn_180();
            skip_next_gap = 1;       /* 掉头后第一个缺口直走 */
            verify_r = verify_l = 0;
            continue;
        }

        if (r_wall && l_wall) {
            /* 两侧都看到墙: 居中, 直行 */
            verify_r = verify_l = 0;
            motor_drive(BASE_SPEED, BASE_SPEED);
        }
        else if (!r_wall && l_wall) {
            /* 右侧丢墙: 漂移 or 真缺口? 先尝试右修正, 验证 */
            verify_l = 0;
            verify_r++;
            if (verify_r >= VERIFY_THRESH) {
                if (skip_next_gap) {
                    /* 掉头后的第一个缺口: 直走穿过 */
                    skip_next_gap = 0;
                    in_skip_pass  = 1;
                    verify_r = 0;
                    motor_drive(BASE_SPEED, BASE_SPEED);
                } else {
                    /* 真缺口, C 型右转 */
                    motor_stop();
                    HAL_Delay(100);
                    c_turn(+1);
                    verify_r = 0;
                }
            } else {
                /* 验证中: 微微右转尝试贴回右墙 */
                motor_drive(BASE_SPEED + VERIFY_DIFF, BASE_SPEED - VERIFY_DIFF);
            }
        }
        else if (r_wall && !l_wall) {
            /* 左侧丢墙: 镜像处理 */
            verify_r = 0;
            verify_l++;
            if (verify_l >= VERIFY_THRESH) {
                if (skip_next_gap) {
                    skip_next_gap = 0;
                    in_skip_pass  = 1;
                    verify_l = 0;
                    motor_drive(BASE_SPEED, BASE_SPEED);
                } else {
                    motor_stop();
                    HAL_Delay(100);
                    c_turn(-1);
                    verify_l = 0;
                }
            } else {
                motor_drive(BASE_SPEED - VERIFY_DIFF, BASE_SPEED + VERIFY_DIFF);
            }
        }
        else {
            /* 两侧都没墙: 开阔地, 直行 (一般不会发生) */
            verify_r = verify_l = 0;
            motor_drive(BASE_SPEED, BASE_SPEED);
        }

        HAL_Delay(LOOP_MS);
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
    motor_stop();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef led = {0};
    led.Pin = GPIO_PIN_13; led.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC, &led);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif