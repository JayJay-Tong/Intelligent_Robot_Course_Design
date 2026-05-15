# 智能迷宫机器人 HAL 库开发详细指南

> **适用硬件**：STM32F103C8T6（Blue Pill）+ L298N + 红外传感器 + HC-SR04 + CJ-M49
> **开发方式**：STM32CubeMX（HAL 库代码生成）+ Keil MDK5

---

## 一、硬件电路详细分析

### 1.1 系统总体硬件架构

```
┌─────────────────────────────────────────────────────────────┐
│                      电源系统                                │
│  7.4V 锂电池 ─→ L298N（电机电源）                           │
│              └─→ 7805/LM2596（5V）─→ STM32 VIN              │
│                                   └─→ 红外传感器 VCC         │
│                                   └─→ HC-SR04 VCC            │
│                                   └─→ CJ-M49 VCC (3.3V via  │
│                                       STM32板载LDO)          │
└─────────────────────────────────────────────────────────────┘

┌────────────┐     PWM×4      ┌─────────┐     ┌──────────┐
│            │───────────────→│  L298N  │────→│  左电机  │
│ STM32F103  │                │  电机   │────→│  右电机  │
│  C8T6      │ GPIO×2(Input)  │  驱动   │     └──────────┘
│            │←───────────────┤         │
│            │                └─────────┘
│            │←── 左IR传感器 (PA11)
│            │←── 右IR传感器 (PA12)
│            │←── HC-SR04 ECHO (PA0 / TIM2_CH1)
│            │──→ HC-SR04 TRIG (PB1)
│            │←──→ CJ-M49 加速度计 (I2C2: PB10/PB11)
│            │←── 启动按键 (PA15)
│            │──→ LED×2 (PA1/PA2)
│            │──→ OLED 显示屏 (软件I2C: PB12/PB13，可选)
│            │──→ USART1 调试 (PA9/PA10)
└────────────┘
```

---

### 1.2 主控芯片 STM32F103C8T6

| 参数     | 值               |
| -------- | ---------------- |
| 内核     | ARM Cortex-M3    |
| 主频     | 72 MHz           |
| Flash    | 64 KB            |
| SRAM     | 20 KB            |
| 工作电压 | 2.0V ~ 3.6V      |
| 调试接口 | SWD（PA13/PA14） |

> **重要**：Blue Pill 板的 PA15 默认为 JTDI，必须在 CubeMX 中将调试接口切换为 **Serial Wire（SWD）** 才能将 PA15 用作普通 GPIO（启动按键）。

---

### 1.3 电机驱动模块 L298N

L298N 是一款双 H 桥电机驱动芯片，可同时驱动两路直流电机。

**电气参数：**

| 参数         | 值                        |
| ------------ | ------------------------- |
| 工作电压     | 5V ~ 35V                  |
| 逻辑控制电压 | 5V（TTL 电平，兼容 3.3V） |
| 最大输出电流 | 每通道 2A（峰值 3A）      |

**PWM 驱动原理：**
通过控制 IN1~IN4 的 PWM 占空比实现调速，IN1/IN2 控制左电机正反转，IN3/IN4 控制右电机正反转。

**引脚连接（保持原设计不变）：**

| STM32 引脚 | 定时器通道 | 连接 L298N 引脚 | 功能           |
| ---------- | ---------- | --------------- | -------------- |
| PB6        | TIM4_CH1   | IN1             | 左电机正转 PWM |
| PB7        | TIM4_CH2   | IN2             | 左电机反转 PWM |
| PB8        | TIM4_CH3   | IN3             | 右电机正转 PWM |
| PB9        | TIM4_CH4   | IN4             | 右电机反转 PWM |

**注意**：L298N 的 ENA、ENB 使能端直接接高电平（或短接跳线）。

**PWM 参数设计（HAL 库版）：**

- TIM4 时钟：72 MHz（APB1 = 36 MHz，APB1 定时器 ×2）
- 预分频：35（÷36）→ 计数时钟 2 MHz
- 自动重载值（ARR）：99（÷100）→ PWM 频率 **20 kHz**
- 占空比范围：CCR 值 0 ~ 99（对应 0% ~ 99%）

---

### 1.4 红外避障传感器（保持原设计）

**原理**：发射红外光，检测物体反射信号，输出数字信号（DO）。

| 传感器位置 | STM32 引脚       | 逻辑含义                     |
| ---------- | ---------------- | ---------------------------- |
| 左侧红外   | PA11（上拉输入） | 1 = 检测到障碍物，0 = 无障碍 |
| 右侧红外   | PA12（上拉输入） | 1 = 检测到障碍物，0 = 无障碍 |

**检测距离**：通过蓝色电位器调节，建议设置为 5~10 cm。

---

### 1.5 超声波传感器 HC-SR04（新增）

**工作原理：**

1. TRIG 引脚接收 ≥10 μs 的高电平触发脉冲
2. 模块自动发射 8 个 40 kHz 超声波脉冲
3. ECHO 引脚输出高电平，时长等于超声波飞行时间
4. 距离计算：`距离(cm) = ECHO 高电平时长(μs) / 58`

**时序参数：**

| 参数          | 值                   |
| ------------- | -------------------- |
| 触发脉冲宽度  | ≥ 10 μs            |
| 测量范围      | 2 cm ~ 400 cm        |
| 测量精度      | ±3 mm               |
| ECHO 最大脉宽 | ≈ 23.2 ms（400 cm） |
| 最小测量周期  | 60 ms（约 16 Hz）    |

**引脚连接（新增）：**

| HC-SR04 引脚 | STM32 引脚 | 配置                 | 说明                       |
| ------------ | ---------- | -------------------- | -------------------------- |
| VCC          | 5V         | —                   | 注意：HC-SR04 需要 5V 供电 |
| GND          | GND        | —                   | 共地                       |
| TRIG         | PB1        | GPIO Output PP       | 发送触发脉冲               |
| ECHO         | PA0        | TIM2_CH1（输入捕获） | 接收回波信号               |

> **电平说明**：ECHO 输出为 5V，而 STM32 I/O 为 3.3V 容限。STM32F103 的 GPIO 标注为 5V 容限（FT），PA0 具有 5V 容限，可直连。如不确定，建议加分压电阻（10K + 20K）降压。

**使用 TIM2 输入捕获测量 ECHO 脉宽：**

- TIM2 预分频：71（72 MHz / 72 = 1 MHz，每 tick = 1 μs）
- ARR：65535（最大计时 65.5 ms，覆盖最大测量距离）
- 捕获策略：先捕获上升沿记录 t1，切换为下降沿捕获记录 t2，距离 = (t2 - t1) / 58

---

### 1.6 三轴加速度计 CJ-M49（新增）

CJ-M49 为 I2C 接口的三轴加速度计模块，可输出 X/Y/Z 三轴加速度数据。

**典型参数（请以实物数据手册为准）：**

| 参数       | 典型值                                     |
| ---------- | ------------------------------------------ |
| 通信接口   | I2C（标准模式 100 kHz / 快速模式 400 kHz） |
| 供电电压   | 3.3V                                       |
| 测量范围   | ±2g / ±4g / ±8g（可配置）               |
| 输出分辨率 | 10 bit / 12 bit                            |
| I2C 地址   | 查看数据手册，常见为 0x1D 或 0x53          |

**引脚连接（新增）：**

| CJ-M49 引脚 | STM32 引脚        | 配置       | 说明                 |
| ----------- | ----------------- | ---------- | -------------------- |
| VCC         | 3.3V              | —         | STM32 板载 3.3V      |
| GND         | GND               | —         | 共地                 |
| SCL         | PB10              | I2C2_SCL   | 硬件 I2C2            |
| SDA         | PB11              | I2C2_SDA   | 硬件 I2C2            |
| INT         | （可选，如 PC13） | GPIO Input | 数据就绪中断（可选） |

> **上拉电阻**：I2C 总线需要上拉电阻。若 CJ-M49 模块板上已集成 4.7K 上拉，则无需额外添加。若直连裸芯片，需在 SCL 和 SDA 上各接 4.7K 上拉至 3.3V。

**在迷宫机器人中的应用：**

- 检测机器人是否卡死（加速度长时间为 0 但电机仍在运转）
- 检测倾斜角度（防止机器人翻倒）
- 辅助判断转弯是否完成（角加速度变化）

---

### 1.7 OLED 显示屏（引脚迁移，可选）

原设计中 OLED 使用 PB8/PB9（软件 I2C），这两个引脚已被 TIM4_CH3/CH4（电机 PWM）占用，**存在冲突**，必须迁移。

**新引脚分配：**

| OLED 引脚 | 原 STM32 引脚 | 新 STM32 引脚 | 配置                   |
| --------- | ------------- | ------------- | ---------------------- |
| SCL       | PB8（冲突）   | PB12          | GPIO Output OD（开漏） |
| SDA       | PB9（冲突）   | PB13          | GPIO Output OD（开漏） |

---

### 1.8 完整引脚分配总表

| STM32 引脚     | 功能                 | 配置模式          | 连接目标     | 状态           |
| -------------- | -------------------- | ----------------- | ------------ | -------------- |
| **PA0**  | HC-SR04 ECHO 捕获    | TIM2_CH1 输入捕获 | HC-SR04 ECHO | **新增** |
| PA1            | LED1                 | GPIO Output PP    | LED 正极     | 保留           |
| PA2            | LED2                 | GPIO Output PP    | LED 正极     | 保留           |
| PA9            | USART1_TX            | 复用推挽          | USB-TTL TX   | 保留           |
| PA10           | USART1_RX            | 复用浮空          | USB-TTL RX   | 保留           |
| PA11           | 左侧红外传感器       | GPIO Input PU     | 左 IR DO     | 保留           |
| PA12           | 右侧红外传感器       | GPIO Input PU     | 右 IR DO     | 保留           |
| PA13           | SWDIO                | SWD 调试          | ST-Link      | 系统占用       |
| PA14           | SWCLK                | SWD 调试          | ST-Link      | 系统占用       |
| PA15           | 启动按键             | GPIO Input PU     | 按键 S1      | 需切换 SWD     |
| **PB1**  | HC-SR04 TRIG         | GPIO Output PP    | HC-SR04 TRIG | **新增** |
| PB6            | 左电机正转 PWM       | TIM4_CH1 AF_PP    | L298N IN1    | 保留           |
| PB7            | 左电机反转 PWM       | TIM4_CH2 AF_PP    | L298N IN2    | 保留           |
| PB8            | 右电机正转 PWM       | TIM4_CH3 AF_PP    | L298N IN3    | 保留           |
| PB9            | 右电机反转 PWM       | TIM4_CH4 AF_PP    | L298N IN4    | 保留           |
| **PB10** | CJ-M49 SCL           | I2C2_SCL          | 加速度计 SCL | **新增** |
| **PB11** | CJ-M49 SDA           | I2C2_SDA          | 加速度计 SDA | **新增** |
| PB12           | OLED SCL（软件 I2C） | GPIO Output OD    | OLED SCL     | **迁移** |
| PB13           | OLED SDA（软件 I2C） | GPIO Output OD    | OLED SDA     | **迁移** |

---

### 1.9 电源方案

```
7.4V 锂电池（2S）
    │
    ├──────────────────→ L298N VCC（电机电源：6V~12V）
    │
    └──→ LM2596/7805 稳压至 5V
              │
              ├──→ STM32 Blue Pill VIN（板载 LDO 再降至 3.3V）
              ├──→ HC-SR04 VCC（5V）
              └──→ 红外传感器 VCC（5V，TTL输出兼容STM32 FT引脚）

STM32 板载 3.3V LDO 输出：
    ├──→ CJ-M49 VCC（3.3V）
    └──→ OLED VCC（3.3V，若模块板上有升压则接5V）

⚠️ 所有模块的 GND 必须共地：电池负极 = L298N GND = STM32 GND = 传感器 GND
```

---

## 二、走迷宫算法选择分析

### 2.1 右手法则（Right-Hand Rule）——当前实现

**原理**：始终保持右手触碰右侧墙壁，沿墙行走。

**状态转移表（2传感器版本）：**

| 左IR      | 右IR      | 动作         |
| --------- | --------- | ------------ |
| 0（无墙） | 1（有墙） | 前进         |
| 0（无墙） | 0（无墙） | 右转（优先） |
| 1（有墙） | 1（有墙） | 前进         |
| 1（有墙） | 0（无墙） | 右转         |

**优点**：实现极简，适合课程展示，代码量少。**缺点**：

- 只适用于"简单连通"迷宫（无孤岛），有孤岛时会绕圈
- 无前方检测，遇到死路依赖"双侧无墙"状态推断
- 容易因转弯时间误差导致偏向，在长直道结束时错过转弯

### 2.2 左手法则（Left-Hand Rule）

与右手法则完全对称，同等优缺点，适用场景相同。选择哪种取决于具体迷宫布局。

### 2.3 Flood Fill 算法

**原理**：将迷宫抽象为二维网格，用 BFS（广度优先搜索）计算每格到终点的最短距离（"洪水填充"），机器人每次移动到数值最小的相邻格。

**优点**：可找到最优路径，能处理任意连通迷宫。**缺点**：

- 需要完整的迷宫地图或实时探索建图
- 对内存和计算能力要求较高
- 需要编码器精确定位当前格子位置
- STM32F103 内存仅 20KB，适合 16×16 迷宫，实现难度大

**结论**：当前硬件（无编码器、无前方传感器）不适合直接使用。

### 2.4 增强右手法则（推荐方案，结合新传感器）

**改进**：利用 HC-SR04 增加前方距离检测，彻底解决死路判断问题。

**新状态机逻辑：**

```
HC-SR04 前方距离 < 20cm（前方有墙）？
    是 → 左IR = 无墙？ → 左转
         否 → 右IR = 无墙？ → 右转
              否 → 原地旋转 180°（死路）
    否 → 右IR = 无墙？ → 右转（右手法则优先）
         否 → 前进
```

**优势**：

- 死路判断精确，不再依赖"双侧同时无墙"的间接推断
- 前方距离可提前减速，提高转弯精度
- CJ-M49 可增加"卡死检测"逻辑（电机运转但加速度为0）

### 2.5 算法选择建议

| 方案                               | 适用场景               | 实现难度       | 可靠性       |
| ---------------------------------- | ---------------------- | -------------- | ------------ |
| 基础右手法则（2传感器）            | 简单直通迷宫           | ★             | 中           |
| **增强右手法则（+HC-SR04）** | **课程展示迷宫** | **★★** | **高** |
| Flood Fill                         | 标准竞赛迷宫           | ★★★★       | 最高         |

**最终推荐**：采用**增强右手法则**，结合 HC-SR04 前方检测 + CJ-M49 卡死检测。既保留了代码简洁性，又显著提升了通过率，是课程设计的最佳平衡点。

---

## 三、总体设计方案

### 3.1 系统功能指标

| 指标         | 目标值               |
| ------------ | -------------------- |
| 迷宫通过率   | ≥ 80%               |
| 直行速度     | 0.2 ~ 0.4 m/s        |
| 前方检测距离 | 2 ~ 40 cm（HC-SR04） |
| 侧方检测距离 | 5 ~ 10 cm（IR）      |
| 卡死检测时间 | 2 s                  |
| 单次迷宫时间 | ≤ 60 s              |

### 3.2 软件架构（HAL 库版）

```
main.c（主程序）
├── 系统初始化（HAL_Init, 时钟, 外设初始化）
├── 等待按键启动
└── 主循环 while(1)
    ├── 读取传感器
    │   ├── IR_GetLeft() / IR_GetRight()
    │   ├── HC_SR04_GetDistance()
    │   └── CJ_M49_GetAccel()
    ├── 执行迷宫算法（增强右手法则状态机）
    ├── 卡死检测（加速度异常判断）
    └── OLED 状态显示（可选）

Hardware 驱动层
├── motor.c / motor.h    ← TIM4 PWM 电机控制（HAL版）
├── ir.c / ir.h          ← 红外传感器读取（HAL GPIO）
├── hcsr04.c / hcsr04.h  ← HC-SR04 超声波（TIM2输入捕获）
├── cjm49.c / cjm49.h    ← CJ-M49 加速度计（I2C2 HAL）
├── oled.c / oled.h      ← OLED 软件I2C显示（可选）
└── key.c / key.h        ← 按键读取（HAL GPIO）

System 层（CubeMX自动生成）
├── main.c（HAL初始化框架）
├── stm32f1xx_hal_msp.c（MSP配置）
├── stm32f1xx_it.c（中断服务例程）
└── HAL 库源文件（Drivers/）
```

### 3.3 开发调试步骤与时间安排

| 阶段 | 内容                       | 预计时间 | 验收标准                     |
| ---- | -------------------------- | -------- | ---------------------------- |
| 1    | CubeMX 工程搭建 + 编译通过 | 0.5 天   | 无报错，下载成功，LED 闪烁   |
| 2    | 电机 PWM 驱动验证          | 0.5 天   | 按键控制前进/后退/转弯       |
| 3    | 红外传感器 + 按键调试      | 0.5 天   | 串口打印传感器值正确         |
| 4    | HC-SR04 超声波调试         | 0.5 天   | 串口打印距离，误差 < 5 cm    |
| 5    | CJ-M49 加速度计调试        | 0.5 天   | 串口打印三轴数据，倾斜有变化 |
| 6    | 增强右手法则算法集成       | 1 天     | 模拟走廊测试通过             |
| 7    | 迷宫整体调参测试           | 1 天     | 迷宫通过率 ≥ 80%            |
| 8    | 报告撰写 + PPT 制作        | 1 天     | 文档完整                     |

---

## 四、PPT 答辩框架（5 分钟）

### 幻灯片结构（建议 10~12 页）

---

**第 1 页：封面**

- 题目：基于 STM32 的智能迷宫机器人设计
- 课程名称、院系、学号姓名、日期

---

**第 2 页：题目简介**

- 研究背景：智能机器人技术在自动化领域的应用
- 研究目标：实现机器人在未知迷宫中的自主导航
- 技术路线：传感器感知 → 算法决策 → 电机执行
- 创新点：多传感器融合（IR + 超声波 + 加速度计）

---

**第 3 页：系统功能及指标**

| 功能         | 技术指标          |
| ------------ | ----------------- |
| 迷宫自主导航 | 通过率 ≥ 80%     |
| 前方障碍检测 | HC-SR04，2~40 cm  |
| 侧方障碍检测 | 红外，5~10 cm     |
| 卡死自动恢复 | 检测时间 2 s      |
| 实时状态显示 | OLED 屏幕（可选） |

---

**第 4 页：系统硬件结构及传感器**

- 系统框图（参考第一节架构图）
- 传感器列表：
  - HC-SR04（超声波，前方测距）
  - 红外传感器×2（左右侧壁检测）
  - CJ-M49（三轴加速度计，卡死检测）
- 主控：STM32F103C8T6
- 驱动：L298N 双路电机驱动

---

**第 5 页：系统硬件结构及传感器（续）**

- 引脚分配表（关键引脚）
- 电源架构（7.4V → 5V → 3.3V 层级）
- 实物照片（电路连接图）

---

**第 6 页：应用软件功能模块及框图**

- 软件层次图：HAL 底层 → 驱动层 → 算法层 → 主控层
- 主要模块：
  - 电机控制模块（motor.c）
  - 传感器读取模块（ir.c, hcsr04.c, cjm49.c）
  - 迷宫算法模块（增强右手法则）
  - 卡死恢复模块

---

**第 7 页：走迷宫算法说明**

- 基础右手法则原理图（简单示意）
- 增强版状态机流程图：
  ```
  前方无墙 → 右侧无墙? → 右转 → 否 → 前进
  前方有墙 → 左侧无墙? → 左转 → 否 → 右侧无墙? → 右转 → 否 → 180°掉头
  ```
- HC-SR04 与 IR 协同逻辑

---

**第 8 页：开发调试步骤及时间安排**

- 甘特图或表格形式的开发计划（参考第三节时间安排）
- 重点说明：模块化调试 → 集成测试 → 参数调优

---

**第 9 页：预期目标及测试方案**

**预期目标：**

- 在标准课程迷宫中通过率 ≥ 80%
- 完成时间 ≤ 60 s

**测试方案：**

- 单元测试：各传感器独立验证（串口打印数据）
- 集成测试：在模拟走廊中验证避障逻辑
- 系统测试：完整迷宫测试，记录 10 次通过率
- 压力测试：修改迷宫路径，验证算法鲁棒性

---

**第 10 页：分工**

| 成员          | 主要职责                                      |
| ------------- | --------------------------------------------- |
| 成员1（组长） | 软件开发：HAL 驱动 + 迷宫算法 + 系统调试      |
| 成员2         | 硬件搭建：电路连接 + CubeMX 配置 + 传感器调试 |
| 成员3         | 文档报告：作业撰写 + PPT 制作 + 测试记录      |

---

**第 11 页：总结与展望（可选）**

- 项目核心创新点总结
- 后续改进方向（编码器精确控制、Flood Fill 算法）

---

## 五、CubeMX 详细搭建流程

### 5.1 创建新工程

1. 打开 **STM32CubeMX**（建议版本 6.x）
2. 点击 **File → New Project**
3. 在 MCU 搜索框输入 `STM32F103C8T6`
4. 在列表中选中 **STM32F103C8Tx**，点击右上角 **Start Project**
5. 若弹出"Initialize all peripherals with default mode?"，选择 **No**

---

### 5.2 Pinout & Configuration — SYS 配置（最先配置）

> **这一步必须最先做**，否则 PA15 无法用作 GPIO 按键。

1. 在左侧 **Categories** 展开 **System Core → SYS**
2. **Debug** 下拉框选择：`Serial Wire`
   - 这会将调试接口从 JTAG 切换为 SWD
   - 自动解放 PA15（JTDI）、PB3（JTDO）、PB4（JTRST）为普通 GPIO
3. **Timebase Source** 保持：`SysTick`（HAL 时基）

---

### 5.3 Pinout & Configuration — RCC 时钟源配置

1. 在左侧展开 **System Core → RCC**
2. **High Speed Clock（HSE）**：选择 `Crystal/Ceramic Resonator`
   - Blue Pill 板上焊有 8 MHz 晶振
3. **Low Speed Clock（LSE）**：`Disable`（不需要 RTC）

---

### 5.4 Clock Configuration（时钟树配置）

点击顶部 **Clock Configuration** 标签页，按以下顺序配置：

```
输入时钟源配置：
  Input frequency（HSE）→ 填入：8

PLL 配置路径：
  PLLSRC → 选择：HSE（而不是 HSI/2）
  PLLMUL → 选择：×9
  结果：8 MHz × 9 = 72 MHz

系统时钟选择：
  System Clock Mux → 选择：PLLCLK
  SYSCLK = 72 MHz ✓

总线时钟配置：
  AHB Prescaler → 1（HCLK = 72 MHz）
  APB1 Prescaler → 2（PCLK1 = 36 MHz，TIM4 时钟 = 72 MHz）
  APB2 Prescaler → 1（PCLK2 = 72 MHz）

验证：
  右上角应显示 HCLK = 72 MHz
  若出现红色警告，点击"Resolve Clock Issues"自动修复
```

---

### 5.5 Pinout & Configuration — GPIO 配置

切换回 **Pinout & Configuration** 标签，在芯片引脚图上逐个点击配置：

#### 5.5.1 LED 引脚（PA1、PA2）

在引脚图上点击 **PA1**：

- 下拉选择：`GPIO_Output`
- 右侧 **GPIO Settings** 中：
  - GPIO output level：`Low`
  - GPIO mode：`Output Push Pull`
  - GPIO Pull-up/Pull-down：`No pull-up and no pull-down`
  - Maximum output speed：`Low`
  - User Label（可选）：`LED1`

对 **PA2** 重复相同操作，User Label：`LED2`

#### 5.5.2 红外传感器引脚（PA11、PA12）

点击 **PA11**：

- 下拉选择：`GPIO_Input`
- GPIO Settings：
  - GPIO mode：`Input mode`
  - GPIO Pull-up/Pull-down：`Pull-up`（上拉，与原代码一致）
  - User Label：`IR_LEFT`

点击 **PA12**：

- 同上，User Label：`IR_RIGHT`

#### 5.5.3 启动按键（PA15）

点击 **PA15**：

- 下拉选择：`GPIO_Input`
- GPIO Settings：
  - GPIO mode：`Input mode`
  - GPIO Pull-up/Pull-down：`Pull-up`
  - User Label：`KEY_START`

#### 5.5.4 HC-SR04 TRIG 引脚（PB1）

点击 **PB1**：

- 下拉选择：`GPIO_Output`
- GPIO Settings：
  - GPIO output level：`Low`
  - GPIO mode：`Output Push Pull`
  - GPIO Pull-up/Pull-down：`No pull-up and no pull-down`
  - Maximum output speed：`High`（需要输出 10 μs 快速脉冲）
  - User Label：`HCSR04_TRIG`

#### 5.5.5 OLED 软件 I2C 引脚（PB12、PB13）

点击 **PB12**：

- 下拉选择：`GPIO_Output`
- GPIO Settings：
  - GPIO mode：`Output Open Drain`（开漏模式，模拟 I2C 时钟）
  - GPIO Pull-up/Pull-down：`Pull-up`
  - Maximum output speed：`High`
  - User Label：`OLED_SCL`

点击 **PB13**：

- 同上，User Label：`OLED_SDA`

---

### 5.6 Pinout & Configuration — TIM4 配置（电机 PWM）

1. 在左侧 **Categories** 展开 **Timers → TIM4**
2. **Mode（模式区）**：

   - Channel1：`PWM Generation CH1`
   - Channel2：`PWM Generation CH2`
   - Channel3：`PWM Generation CH3`
   - Channel4：`PWM Generation CH4`
   - 此时引脚图上 PB6、PB7、PB8、PB9 自动变绿
3. **Configuration → Parameter Settings**：

| 参数                                   | 值              | 说明                         |
| -------------------------------------- | --------------- | ---------------------------- |
| Prescaler (PSC)                        | `35`          | 72 MHz / 36 = 2 MHz 计数时钟 |
| Counter Mode                           | `Up`          | 向上计数                     |
| Counter Period (ARR)                   | `99`          | 2 MHz / 100 = 20 kHz PWM     |
| Internal Clock Division                | `No Division` | 不分频                       |
| auto-reload preload                    | `Enable`      | 使能预装载                   |
| **PWM Generation Channel 1~4：** |                 |                              |
| Mode                                   | `PWM mode 1`  | 高有效 PWM                   |
| Pulse (CCR)                            | `0`           | 初始占空比为 0               |
| Fast Mode                              | `Disable`     | 不需要快速模式               |
| CH Polarity                            | `High`        | 高有效                       |

4. **不需要在 NVIC 中使能 TIM4 中断**（纯 PWM 输出不需要中断）

---

### 5.7 Pinout & Configuration — TIM2 配置（HC-SR04 输入捕获）

1. 展开 **Timers → TIM2**
2. **Mode（模式区）**：

   - Clock Source：`Internal Clock`（内部时钟）
   - Channel1：`Input Capture direct mode`（直接输入捕获）
   - 此时引脚图上 PA0 自动变绿
3. **Configuration → Parameter Settings**：

| 参数                                | 值              | 说明                              |
| ----------------------------------- | --------------- | --------------------------------- |
| Prescaler (PSC)                     | `71`          | 72 MHz / 72 = 1 MHz（1 μs/tick） |
| Counter Mode                        | `Up`          | 向上计数                          |
| Counter Period (ARR)                | `65535`       | 最大计时 65.5 ms                  |
| Internal Clock Division             | `No Division` | —                                |
| auto-reload preload                 | `Disable`     | —                                |
| **Input Capture Channel 1：** |                 |                                   |
| Polarity Selection                  | `Rising Edge` | 初始捕获上升沿                    |
| IC Selection                        | `Direct`      | 直接输入                          |
| Prescaler Division Ratio            | `No division` | 不预分频                          |
| Input Filter                        | `0`           | 不滤波                            |

4. **Configuration → NVIC Settings**：
   - TIM2 global interrupt：**勾选 Enable**
   - 优先级（Preemption Priority）：`1`（比 SysTick 低即可）

---

### 5.8 Pinout & Configuration — I2C2 配置（CJ-M49 加速度计）

1. 展开 **Connectivity → I2C2**
2. **Mode**：`I2C`
3. 引脚图上 PB10（SCL）和 PB11（SDA）自动变绿
4. **Configuration → Parameter Settings**：

| 参数              | 值                | 说明                                             |
| ----------------- | ----------------- | ------------------------------------------------ |
| I2C Speed Mode    | `Standard Mode` | 100 kHz（若 CJ-M49 支持 400 kHz 可选 Fast Mode） |
| I2C Clock Speed   | `100000`        | 100 kHz                                          |
| Duty Cycle        | `2`             | 标准占空比（Fast Mode 时为 16/9）                |
| Addressing Mode   | `7 bit`         | 7 位地址                                         |
| Dual Address Mode | `Disable`       | —                                               |
| General Call Mode | `Disable`       | —                                               |
| No Stretch Mode   | `Disable`       | 允许时钟拉伸                                     |

5. **Configuration → NVIC Settings**：
   - I2C2 event interrupt：可选（使用 HAL 轮询模式则不需要）
   - 建议先用轮询模式调试，稳定后再改中断模式

---

### 5.9 Pinout & Configuration — USART1 配置（调试串口）

1. 展开 **Connectivity → USART1**
2. **Mode**：`Asynchronous`（异步串口）
3. 引脚图上 PA9（TX）和 PA10（RX）自动变绿
4. **Configuration → Parameter Settings**：

| 参数           | 值                            |
| -------------- | ----------------------------- |
| Baud Rate      | `115200`                    |
| Word Length    | `8 Bits (including Parity)` |
| Parity         | `None`                      |
| Stop Bits      | `1`                         |
| Data Direction | `Receive and Transmit`      |
| Over Sampling  | `16 Samples`                |

5. **NVIC Settings**：不使能（使用 `printf` 重定向，轮询发送即可）

---

### 5.10 NVIC 全局中断优先级设置

在左侧 **System Core → NVIC** 中确认以下配置：

| 中断                  | 是否使能       | Preemption Priority | 说明                 |
| --------------------- | -------------- | ------------------- | -------------------- |
| SysTick interrupt     | 是（HAL 自动） | 15                  | HAL 时基，最低优先级 |
| TIM2 global interrupt | **是**   | 1                   | HC-SR04 输入捕获     |
| I2C2 event interrupt  | 可选           | 2                   | 若使用中断模式       |

> NVIC Priority Group 保持默认：`4 bits for pre-emption priority`（即 16 级抢占优先级，无子优先级）。

---

### 5.11 Project Manager 设置

点击顶部 **Project Manager** 标签页：

#### Project 子标签：

| 设置项                    | 值                                                                          |
| ------------------------- | --------------------------------------------------------------------------- |
| Project Name              | `MazeRobot`（英文，无空格）                                               |
| Project Location          | 选择你的项目根目录（如 `F:\VScode_doc\Intelligent_Robot_Course_Design\`） |
| Application Structure     | `Basic`                                                                   |
| **Toolchain / IDE** | **`MDK-ARM`**                                                       |
| Min Version               | `V5`                                                                      |
| Heap Size                 | `0x200`（512 B，够用）                                                    |
| Stack Size                | `0x400`（1 KB）                                                           |

#### Code Generator 子标签：

| 设置项                               | 推荐选项                                                                         | 说明                                                |
| ------------------------------------ | -------------------------------------------------------------------------------- | --------------------------------------------------- |
| STM32Cube Firmware Library Package   | `Copy only the necessary library files`                                        | 只复制用到的 HAL 源文件，减小工程体积               |
| Generated files                      | ☑`Generate peripheral initialization as a pair of .c/.h files per peripheral` | 每个外设生成独立的 .c/.h 文件，结构清晰             |
| Keep User Code when re-generating    | ☑**必须勾选**                                                             | 重新生成时保留 `USER CODE BEGIN / END` 之间的代码 |
| Delete previously generated files... | ☑                                                                               | 清理旧的生成文件                                    |

---

### 5.12 生成代码

1. 点击右上角橙色按钮 **GENERATE CODE**
2. 若弹出固件包下载提示，确认下载对应版本（STM32Cube_FW_F1_V1.8.x）
3. 生成完成后点击 **Open Project**，自动用 Keil5 打开工程

**生成后的工程目录结构：**

```
MazeRobot/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32f1xx_hal_conf.h
│   │   ├── stm32f1xx_it.h
│   │   ├── tim.h        ← TIM2/TIM4 初始化声明
│   │   ├── i2c.h        ← I2C2 初始化声明
│   │   ├── usart.h      ← USART1 初始化声明
│   │   └── gpio.h       ← GPIO 初始化声明
│   └── Src/
│       ├── main.c       ← 主程序入口（在 USER CODE 块中写逻辑）
│       ├── stm32f1xx_it.c  ← 中断服务例程
│       ├── stm32f1xx_hal_msp.c
│       ├── tim.c        ← TIM2/TIM4 HAL 初始化代码
│       ├── i2c.c        ← I2C2 HAL 初始化代码
│       ├── usart.c      ← USART1 HAL 初始化代码
│       └── gpio.c       ← GPIO HAL 初始化代码
├── Drivers/
│   ├── STM32F1xx_HAL_Driver/   ← HAL 库源码（自动生成）
│   └── CMSIS/                  ← CMSIS 内核文件
├── MDK-ARM/
│   └── MazeRobot.uvprojx       ← Keil 工程文件
└── MazeRobot.ioc               ← CubeMX 工程文件（保留，后续修改用）
```

---

## 六、Keil5 代码构建流程

### 6.1 工程配置确认

打开 `MDK-ARM/MazeRobot.uvprojx`，进行以下检查：

1. **Target Options → C/C++**：

   - C99 Mode：建议勾选（`--c99`）
   - Define：`USE_HAL_DRIVER,STM32F103xB`（CubeMX 已自动配置）
   - Include Paths：确认包含 `Core/Inc` 和 `Drivers/` 路径
2. **添加自定义驱动文件夹**：

   - 在 Keil 左侧 Project 树中，右键点击项目根节点 → **Add Group**
   - 新建组：`Hardware`
   - 将后续创建的 `motor.c`、`ir.c`、`hcsr04.c`、`cjm49.c` 等加入此组
3. **Include 路径添加**：

   - Target Options → C/C++ → Include Paths 中添加 `../Hardware`（相对 MDK-ARM 目录）

---

### 6.2 微秒延时实现（HAL 必备工具）

CubeMX 只提供 `HAL_Delay()`（毫秒级），HC-SR04 需要微秒延时。

在 `Core/Src/main.c` 的 `USER CODE BEGIN PV` 块中（或新建 `Hardware/delay_us.c`）：

```c
/* 使用 DWT 计数器实现微秒延时（Cortex-M3 支持）*/
void delay_us_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}
```

在 `main.c` 的 `USER CODE BEGIN 2` 中调用：`delay_us_init();`

---

### 6.3 电机控制模块（motor.c / motor.h）

**新建 `Hardware/motor.h`：**

```c
#ifndef __MOTOR_H
#define __MOTOR_H

#include "tim.h"

void Motor_Init(void);
void Motor_SetSpeed(int16_t left, int16_t right);
void Motor_Forward(uint8_t speed, uint32_t ms);
void Motor_Backward(uint8_t speed, uint32_t ms);
void Motor_TurnLeft(uint8_t speed, uint32_t ms);
void Motor_TurnRight(uint8_t speed, uint32_t ms);
void Motor_SpinLeft(uint8_t speed, uint32_t ms);
void Motor_SpinRight(uint8_t speed, uint32_t ms);
void Motor_Stop(uint32_t ms);

#endif
```

**新建 `Hardware/motor.c`：**

```c
#include "motor.h"
#include "main.h"

/* 设置 TIM4 各通道 CCR（占空比 0~99）*/
#define SET_PWM(ch, val) \
    __HAL_TIM_SET_COMPARE(&htim4, ch, (val) > 99 ? 99 : (val))

void Motor_Init(void)
{
    /* 启动 TIM4 的 4 个 PWM 通道 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    /* 初始停止 */
    SET_PWM(TIM_CHANNEL_1, 0);
    SET_PWM(TIM_CHANNEL_2, 0);
    SET_PWM(TIM_CHANNEL_3, 0);
    SET_PWM(TIM_CHANNEL_4, 0);
}

/* left: 左电机速度 (-99~99)，正数前进，负数后退
   right: 右电机速度 (-99~99) */
void Motor_SetSpeed(int16_t left, int16_t right)
{
    uint8_t l1 = 0, l2 = 0, r1 = 0, r2 = 0;
    if (left > 0)       { l1 = left;  l2 = 0;     }
    else if (left < 0)  { l1 = 0;     l2 = -left; }
    if (right > 0)      { r1 = right; r2 = 0;     }
    else if (right < 0) { r1 = 0;     r2 = -right;}

    SET_PWM(TIM_CHANNEL_1, l1);
    SET_PWM(TIM_CHANNEL_2, l2);
    SET_PWM(TIM_CHANNEL_3, r1);
    SET_PWM(TIM_CHANNEL_4, r2);
}

void Motor_Forward(uint8_t speed, uint32_t ms)
{
    Motor_SetSpeed(speed, speed);
    HAL_Delay(ms);
}

void Motor_Backward(uint8_t speed, uint32_t ms)
{
    Motor_SetSpeed(-speed, -speed);
    HAL_Delay(ms);
}

void Motor_TurnLeft(uint8_t speed, uint32_t ms)
{
    Motor_SetSpeed(0, speed);
    HAL_Delay(ms);
}

void Motor_TurnRight(uint8_t speed, uint32_t ms)
{
    Motor_SetSpeed(speed, 0);
    HAL_Delay(ms);
}

void Motor_SpinLeft(uint8_t speed, uint32_t ms)
{
    Motor_SetSpeed(-speed, speed);
    HAL_Delay(ms);
}

void Motor_SpinRight(uint8_t speed, uint32_t ms)
{
    Motor_SetSpeed(speed, -speed);
    HAL_Delay(ms);
}

void Motor_Stop(uint32_t ms)
{
    Motor_SetSpeed(0, 0);
    HAL_Delay(ms);
}
```

---

### 6.4 红外传感器模块（ir.c / ir.h）

**新建 `Hardware/ir.h`：**

```c
#ifndef __IR_H
#define __IR_H

#include "gpio.h"

void IR_Init(void);       /* CubeMX 已生成，此函数可省略 */
uint8_t IR_GetLeft(void); /* 返回 1=有障碍，0=无障碍 */
uint8_t IR_GetRight(void);

#endif
```

**新建 `Hardware/ir.c`：**

```c
#include "ir.h"
#include "main.h"

/* IR_LEFT → PA11（GPIO_PIN_11），IR_RIGHT → PA12（GPIO_PIN_12）*/
/* User Label 在 main.h 中已自动生成宏定义：IR_LEFT_Pin, IR_LEFT_GPIO_Port */

uint8_t IR_GetLeft(void)
{
    return HAL_GPIO_ReadPin(IR_LEFT_GPIO_Port, IR_LEFT_Pin);
}

uint8_t IR_GetRight(void)
{
    return HAL_GPIO_ReadPin(IR_RIGHT_GPIO_Port, IR_RIGHT_Pin);
}
```

> **注意**：`main.h` 中 CubeMX 会根据 User Label 自动生成：
>
> ```c
> #define IR_LEFT_Pin      GPIO_PIN_11
> #define IR_LEFT_GPIO_Port GPIOA
> #define IR_RIGHT_Pin     GPIO_PIN_12
> #define IR_RIGHT_GPIO_Port GPIOA
> ```

---

### 6.5 HC-SR04 超声波模块（hcsr04.c / hcsr04.h）

**新建 `Hardware/hcsr04.h`：**

```c
#ifndef __HCSR04_H
#define __HCSR04_H

#include "tim.h"
#include <stdint.h>

void HCSR04_Init(void);
uint16_t HCSR04_GetDistance(void); /* 返回距离，单位 cm；超出量程返回 999 */

/* 在 stm32f1xx_it.c 中调用此回调 */
void HCSR04_TIM_IC_Callback(void);

#endif
```

**新建 `Hardware/hcsr04.c`：**

```c
#include "hcsr04.h"
#include "main.h"

/* delay_us 声明（在 main.c 或 delay_us.c 中定义）*/
extern void delay_us(uint32_t us);

static volatile uint32_t ic_val1 = 0;
static volatile uint32_t ic_val2 = 0;
static volatile uint8_t  capture_state = 0; /* 0=等待上升沿, 1=等待下降沿 */
static volatile uint16_t distance_cm = 0;

void HCSR04_Init(void)
{
    /* 启动 TIM2 输入捕获，使能中断 */
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

/* 在 stm32f1xx_it.c 的 TIM2_IRQHandler 中调用 HAL_TIM_IRQHandler(&htim2)，
   HAL 会自动调用 HAL_TIM_IC_CaptureCallback */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (capture_state == 0)
        {
            /* 捕获到上升沿，记录 t1，切换为下降沿 */
            ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            capture_state = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
        }
        else
        {
            /* 捕获到下降沿，记录 t2，计算距离 */
            ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            uint32_t diff = (ic_val2 >= ic_val1) ?
                            (ic_val2 - ic_val1) :
                            (0xFFFFU - ic_val1 + ic_val2 + 1U); /* 处理溢出 */
            distance_cm = (uint16_t)(diff / 58U);
            if (distance_cm > 400) distance_cm = 999;
            capture_state = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
        }
    }
}

uint16_t HCSR04_GetDistance(void)
{
    /* 发送触发脉冲 */
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
    delay_us(12);
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);

    /* 等待捕获完成（最大等待 30 ms）*/
    uint32_t start = HAL_GetTick();
    while (capture_state != 0)
    {
        if (HAL_GetTick() - start > 30) return 999; /* 超时 */
    }
    return distance_cm;
}
```

**修改 `Core/Src/stm32f1xx_it.c`（在 USER CODE 块中）：**

```c
void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM2_IRQn 0 */
    /* USER CODE END TIM2_IRQn 0 */
    HAL_TIM_IRQHandler(&htim2);
    /* USER CODE BEGIN TIM2_IRQn 1 */
    /* USER CODE END TIM2_IRQn 1 */
}
```

> CubeMX 重新生成代码时会保留 USER CODE 块，TIM2_IRQHandler 的框架已自动生成，`HAL_TIM_IRQHandler` 调用也已自动插入，无需手动修改。

---

### 6.6 CJ-M49 加速度计模块（cjm49.c / cjm49.h）

> **注意**：请以实物数据手册为准确认 I2C 地址和寄存器定义。以下以 ADXL345 兼容寄存器格式为模板。

**新建 `Hardware/cjm49.h`：**

```c
#ifndef __CJM49_H
#define __CJM49_H

#include "i2c.h"
#include <stdint.h>

/* ⚠️ 根据数据手册修改以下地址 */
#define CJM49_I2C_ADDR    (0x53 << 1)  /* 7位地址左移1位（HAL I2C 规范） */

/* 加速度数据结构体 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelData_t;

uint8_t CJM49_Init(void);
uint8_t CJM49_ReadAccel(AccelData_t *data);
uint8_t CJM49_IsStuck(void);   /* 卡死检测：返回 1 表示可能卡死 */

#endif
```

**新建 `Hardware/cjm49.c`：**

```c
#include "cjm49.h"
#include "main.h"

/* ⚠️ 以下寄存器地址请对照 CJ-M49 数据手册核实 */
#define REG_DEVID        0x00  /* 设备 ID 寄存器 */
#define REG_POWER_CTL    0x2D  /* 电源控制 */
#define REG_DATA_FORMAT  0x31  /* 数据格式 */
#define REG_DATAX0       0x32  /* X 轴数据低字节 */
#define EXPECTED_DEVID   0xE5  /* ADXL345 的设备 ID，请按手册修改 */

static HAL_StatusTypeDef WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c2, CJM49_I2C_ADDR,
                             reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, 100);
}

static HAL_StatusTypeDef ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c2, CJM49_I2C_ADDR,
                            reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, 100);
}

uint8_t CJM49_Init(void)
{
    uint8_t id = 0;
    /* 读取设备 ID 验证通信 */
    if (ReadRegs(REG_DEVID, &id, 1) != HAL_OK) return 0;
    /* ⚠️ 若设备 ID 不匹配，检查 I2C 地址和寄存器定义 */
    /* if (id != EXPECTED_DEVID) return 0; */

    /* 配置数据格式：±2g，全分辨率 */
    WriteReg(REG_DATA_FORMAT, 0x08);
    /* 使能测量模式 */
    WriteReg(REG_POWER_CTL, 0x08);
    return 1;
}

uint8_t CJM49_ReadAccel(AccelData_t *data)
{
    uint8_t buf[6];
    if (ReadRegs(REG_DATAX0, buf, 6) != HAL_OK) return 0;
    /* 小端格式组合 16 位有符号整数 */
    data->x = (int16_t)((buf[1] << 8) | buf[0]);
    data->y = (int16_t)((buf[3] << 8) | buf[2]);
    data->z = (int16_t)((buf[5] << 8) | buf[4]);
    return 1;
}

/* 简单卡死检测：连续 N 次采样，若合加速度变化 < 阈值则认为卡死 */
uint8_t CJM49_IsStuck(void)
{
    static int16_t prev_x = 0, prev_y = 0;
    static uint8_t stuck_cnt = 0;
    AccelData_t a;

    if (!CJM49_ReadAccel(&a)) return 0;

    int32_t dx = (int32_t)(a.x - prev_x);
    int32_t dy = (int32_t)(a.y - prev_y);
    int32_t diff = dx * dx + dy * dy;

    prev_x = a.x;
    prev_y = a.y;

    if (diff < 100)   /* 阈值：根据实际调整 */
        stuck_cnt++;
    else
        stuck_cnt = 0;

    return (stuck_cnt > 20); /* 连续 20 次无变化（约 2s）认为卡死 */
}
```

---

### 6.7 按键模块（key.c / key.h）

**新建 `Hardware/key.h`：**

```c
#ifndef __KEY_H
#define __KEY_H

#include "gpio.h"

uint8_t Key_GetNum(void); /* 轮询读取，按下返回 1，否则返回 0 */

#endif
```

**新建 `Hardware/key.c`：**

```c
#include "key.h"
#include "main.h"

uint8_t Key_GetNum(void)
{
    if (HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(20); /* 消抖 */
        if (HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_RESET)
        {
            while (HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_RESET);
            return 1;
        }
    }
    return 0;
}
```

> PA15 为上拉输入，按键按下 → PA15 变低 → `GPIO_PIN_RESET`

---

### 6.8 printf 重定向（串口调试）

在 `Core/Src/usart.c` 的 `USER CODE BEGIN 0` 块中添加：

```c
/* USER CODE BEGIN 0 */
#include <stdio.h>
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;
}
/* USER CODE END 0 */
```

Keil5 中还需在 **Target Options → Target** 中勾选 **Use MicroLIB**（使用 MicroLib 以支持 printf）。

---

### 6.9 主程序逻辑（main.c）

在 `Core/Src/main.c` 中，在各 `USER CODE BEGIN` 块中填入以下代码：

```c
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "ir.h"
#include "hcsr04.h"
#include "cjm49.h"
#include "key.h"
#include <stdio.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
#define SPEED_FORWARD    70   /* 前进速度 0~99，根据实物调整 */
#define SPEED_TURN       65   /* 转弯速度 */
#define DIST_FRONT_WALL  20   /* 前方视为有墙的距离阈值（cm） */
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
delay_us_init();      /* 初始化 DWT 微秒延时 */
Motor_Init();         /* 启动 TIM4 PWM */
HCSR04_Init();        /* 启动 TIM2 输入捕获 */

/* 初始化 CJ-M49，失败时 LED 报错提示 */
if (!CJM49_Init()) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    printf("CJM49 init failed!\r\n");
}

printf("MazeRobot HAL Ready. Press KEY to start.\r\n");

/* 等待按键启动 */
while (!Key_GetNum());

printf("Start!\r\n");
HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
/* USER CODE END 2 */

/* USER CODE BEGIN WHILE */
while (1)
{
    uint8_t L = IR_GetLeft();            /* 1=有墙，0=无墙 */
    uint8_t R = IR_GetRight();
    uint16_t front = HCSR04_GetDistance(); /* cm */

    /* 卡死检测 */
    if (CJM49_IsStuck()) {
        Motor_Stop(200);
        Motor_Backward(SPEED_FORWARD, 500);
        Motor_SpinRight(SPEED_TURN, 900);
        printf("Stuck! Recovering...\r\n");
        continue;
    }

    /* 增强右手法则状态机 */
    if (front < DIST_FRONT_WALL)
    {
        /* 前方有墙 */
        Motor_Stop(200);
        if (L == 0) {
            /* 左侧无墙，左转 */
            Motor_TurnLeft(SPEED_TURN, 400);
        } else if (R == 0) {
            /* 右侧无墙，右转 */
            Motor_TurnRight(SPEED_TURN, 400);
        } else {
            /* 三面均有墙，死路，180° 掉头 */
            Motor_SpinRight(SPEED_TURN, 900);
        }
    }
    else
    {
        /* 前方无墙 */
        if (R == 0) {
            /* 右侧无墙，右手法则优先右转 */
            Motor_Stop(100);
            Motor_TurnRight(SPEED_TURN, 400);
        } else {
            /* 右侧有墙，前进 */
            Motor_Forward(SPEED_FORWARD, 50);
        }
    }

    /* 调试输出（可注释掉以提高速度）*/
    printf("L=%d R=%d Front=%d cm\r\n", L, R, front);
}
/* USER CODE END WHILE */
```

---

### 6.10 编译与下载

1. **编译**：点击 Keil 工具栏 **Build**（F7）

   - 正常情况：`0 Error(s), 0 Warning(s)`
   - 常见错误：
     - `undefined symbol delay_us`：检查 delay_us_init/delay_us 是否声明和定义
     - `undefined symbol htim4`：检查 `tim.h` 是否被 `motor.c` 包含
     - `cannot open source file "motor.h"`：检查 Include Paths 是否包含 Hardware 文件夹
2. **下载**：

   - 连接 ST-Link V2 到 Blue Pill 的 SWD 接口（SWDIO=PA13, SWCLK=PA14, GND, 3.3V）
   - Target Options → Debug → ST-Link Debugger → Settings → 确认识别到芯片
   - 点击 **Download**（F8）
   - 下载成功后按下 RESET，串口助手应打印：`MazeRobot HAL Ready. Press KEY to start.`
3. **关键调参顺序**：

   - 先单独测试 HC-SR04（串口打印距离，手挡传感器验证）
   - 再测试 CJ-M49（倾斜开发板，观察 xyz 数据变化）
   - 再测试电机（逐步增大 speed，确认方向正确）
   - 最后调 `DIST_FRONT_WALL`、`SPEED_TURN`、转弯时间（400 ms / 900 ms）

---

## 七、常见问题排查

| 问题现象             | 可能原因                            | 解决方案                                                    |
| -------------------- | ----------------------------------- | ----------------------------------------------------------- |
| PA15 按键不触发      | CubeMX 调试模式未切换为 SWD         | 回到 CubeMX → SYS → Debug 改为 Serial Wire，重新生成      |
| TIM4 PWM 无输出      | `HAL_TIM_PWM_Start()` 未调用      | 确认 Motor_Init() 被调用，且在主循环前                      |
| HC-SR04 始终返回 999 | ECHO 引脚电平未正确接收             | 检查 PA0 是否有 5V 信号，考虑加分压；检查 TIM2 中断是否使能 |
| I2C2 通信失败        | SCL/SDA 缺少上拉电阻                | 在 PB10/PB11 各接 4.7K 上拉至 3.3V                          |
| CJM49 Init 返回 0    | I2C 地址错误                        | 用逻辑分析仪或 I2C 扫描代码查找真实地址                     |
| 电机转向相反         | 电机接线正负极反了                  | 交换 L298N OUT1/OUT2 或修改 Motor_SetSpeed 逻辑             |
| OLED 无显示          | PB8/PB9 软件 I2C 未迁移到 PB12/PB13 | 修改 OLED 驱动中的端口定义                                  |
| 编译报 MicroLIB 错误 | 未勾选 Use MicroLIB                 | Target Options → Target → ☑ Use MicroLIB                 |
