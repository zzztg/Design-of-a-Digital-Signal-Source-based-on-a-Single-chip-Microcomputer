# 双路 DDS 数字信号源 — 工程文档

> STM32F407VGT6 + AD9834 × 2 双通道 DDS 信号发生器
> 频率 1Hz~10MHz, 幅值 ±18V, 相位 0~360° 可调

---

## 目录

1. [硬件架构](#1-硬件架构)
2. [引脚分配](#2-引脚分配)
3. [AD9834 驱动 (BSP)](#3-ad9834-驱动-bsp)
4. [外设配置 (STM32CubeMX)](#4-外设配置-stm32cubeMX)
5. [系统设计伪代码](#5-系统设计伪代码)
6. [交互逻辑](#6-交互逻辑)
7. [算法说明](#7-算法说明)
8. [后续工作](#8-后续工作)

---

## 1. 硬件架构

```
                    ┌──────────────────────────────────────────────┐
                    │                 STM32F407                   │
                    │                                            │
  ┌────────┐        │  TIM2 (编码器) ← ENC1 (频率)               │
  │  EC11   │────────┤  TIM3 (编码器) ← ENC2 (幅值)               │
  │  ×3     │────────┤  TIM4 (编码器) ← ENC3 (相位)               │
  │ +按键   │        │  PC0~PC2   ← 编码器按键                    │
  └────────┘        │                                            │
                    │  SPI2  ──→ AD9834_A (CS=PB12)              │
  ┌────────┐        │  SPI2  ──→ AD9834_B (CS=PB11)              │
  │ 独立按键 │        │  SPI2  ──→ DAC_A   (CS=PAx)  → PGA → ±18V │
  │ (预留)  │        │  SPI2  ──→ DAC_B   (CS=PAx)  → PGA → ±18V │
  └────────┘        │                                            │
                    │  I2C1  ──→ OLED 128×64                     │
  ┌────────┐        │  USART1 ──→ CH340 (printf, RX/TX)          │
  │  PC上位机 │───────┤                                            │
  └────────┘        └──────────────────────────────────────────────┘
```

### 幅值链路

```
AD9834 输出 (~0.6Vpp) → 可编程增益放大器 (PGA) → 功率放大器 → ±18V
                            ↑
                         DAC 控制电压 (SPI2, 独立 CS)
```

---

## 2. 引脚分配

### AD9834 片选

| 信号 | 引脚 | 说明 |
|------|------|------|
| CS_A | PB12 | AD9834 通道 A 片选 |
| CS_B | PB11 | AD9834 通道 B 片选 |

### SPI2 (AD9834 + DAC 共用总线)

| 信号 | 引脚 | 说明 |
|------|------|------|
| SPI2_SCK | PB13 | SPI 时钟 |
| SPI2_MOSI | PB15 | 主机输出从机输入 |

### USART1 (printf 调试)

| 信号 | 引脚 | 说明 |
|------|------|------|
| USART1_TX | PA9 | 发送 |
| USART1_RX | PA10 | 接收 |
| 波特率 | — | 115200, 8N1 |

### 旋转编码器 (定时器编码器模式)

| 编码器 | 定时器 | 作用 | 按键引脚 (假设) |
|--------|--------|------|----------------|
| ENC1 | TIM2 | 频率调节 | PC0 |
| ENC2 | TIM3 | 幅值调节 | PC1 |
| ENC3 | TIM4 | 相位调节 | PC2 |

### 独立按键 (预留)

| 按键 | 作用 | 引脚 |
|------|------|------|
| KEY_CH | 目标通道切换 A/B/BOTH | TBD |
| KEY_WAVE | 波形切换 Sine/Triangle/Sine+Square | TBD |

### OLED (I2C1)

| 信号 | 引脚 |
|------|------|
| I2C1_SCL | PB6 |
| I2C1_SDA | PB7 |

---

## 3. AD9834 驱动 (BSP)

### 文件结构

```
BSP/
├── ad9834.h     # 驱动头文件: 类型定义 + 函数声明
└── ad9834.c     # 驱动实现
```

### ad9834.h — 类型定义

```c
/* ===== 寄存器联合体 (位域映射) ===== */

// 控制寄存器 (16bit)
typedef union AD9834_ControlRegister {
    uint16_t value;
    struct {
        uint16_t reserved_db0 : 1;
        uint16_t mode        : 1;   // 模式: 0=正弦, 1=三角
        uint16_t reserved_db2 : 1;
        uint16_t div2        : 1;   // 方波分频: 0=/1, 1=/2
        uint16_t sign_pib    : 1;
        uint16_t opbiten     : 1;   // 方波使能
        uint16_t sleep12     : 1;
        uint16_t sleep1      : 1;
        uint16_t reset       : 1;   // 复位
        uint16_t pin_sw      : 1;
        uint16_t psel        : 1;   // 相位寄存器选择
        uint16_t fsel        : 1;   // 频率寄存器选择
        uint16_t hlb         : 1;
        uint16_t b28         : 1;   // 一次写入 28 位模式
        uint16_t reserved_db14 : 1;
        uint16_t reserved_db15 : 1;
    };
} ad9834_control_register_t;

// 频率寄存器 (14bit 数据, SPI 分两次写入)
typedef union AD9834_FrequencyRegister {
    uint16_t value;
    struct {
        uint16_t data          : 14;
        uint16_t reserved_db14 : 1;
        uint16_t reserved_db15 : 1;
    };
} ad9834_frequency_register_t;

// 相位寄存器 (12bit 数据)
typedef union AD9834_PhaseRegister {
    uint16_t value;
    struct {
        uint16_t data          : 12;
        uint16_t reserved_db12 : 1;
        uint16_t reserved_db13 : 1;
        uint16_t reserved_db14 : 1;
        uint16_t reserved_db15 : 1;
    };
} ad9834_phase_register_t;

/* ===== 枚举 ===== */

typedef enum { FREQ0_REG, FREQ1_REG } ad9834_frequency_reg_t;
typedef enum { PHASE0_REG, PHASE1_REG } ad9834_phase_reg_t;
typedef enum { AD9834_WaveSine, AD9834_WaveTriangle, AD9834_WaveSineWithSquare } ad9834_wave_t;
typedef enum { AD9834_BitReset, AD9834_BitSet } ad9834_bit_t;

/* ===== 实例结构体 (面向对象) ===== */

typedef struct {
    SPI_HandleTypeDef *hspi;                // SPI 句柄 (可指定, 不硬编码)
    uint16_t cs_pin;                        // 片选引脚号
    GPIO_TypeDef *cs_port;                  // 片选引脚端口
    ad9834_control_register_t control_reg;  // 影子控制寄存器
} ad9834_t;
```

### ad9834.c — API 实现

```c
/* === 内部 SPI 写 (CS 自动管理) === */
static inline void AD9834_SPIWrite(ad9834_t *self, uint8_t *data, size_t len)
{
    HAL_GPIO_WritePin(self->cs_port, self->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(self->hspi, data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(self->cs_port, self->cs_pin, GPIO_PIN_SET);
}

/* === 初始化 === */
void AD9834_Init(ad9834_t *self, SPI_HandleTypeDef *hspi, uint16_t cs_pin, GPIO_TypeDef *cs_port)
{
    self->hspi = hspi;
    self->cs_pin = cs_pin;
    self->cs_port = cs_port;
    self->control_reg.value = 0;
    self->control_reg.b28 = AD9834_BitSet;  // 使能 28 位写入模式
}

/* === 复位 / 释放复位 === */
void AD9834_SetReset(ad9834_t *self, ad9834_bit_t reset);

/* === 设置频率: 频率(Hz) → 28bit 频率字 === */
void AD9834_SetFrequency(ad9834_t *self, ad9834_frequency_reg_t freg, uint32_t frequency)
{
    uint32_t data = (uint32_t)(frequency * pow(2, 28) / 75000000.0);
    // 写入 LSB (14bit) → MSB (14bit)
    // 根据 freg 设置 D15/D14 (FREQ0=0011, FREQ1=1011)
}

/* === 设置相位: 角度(0~360) → 12bit 相位字 === */
void AD9834_SetPhase(ad9834_t *self, ad9834_phase_reg_t preg, uint16_t angle);

/* === 选择输出寄存器 === */
void AD9834_SelectFrequencyRegOutput(ad9834_t *self, ad9834_frequency_reg_t fsel);
void AD9834_SelectPhaseRegOutput(ad9834_t *self, ad9834_phase_reg_t preg);

/* === 设置波形 (正弦/三角/正弦+方波) === */
void AD9834_SetOutputWave(ad9834_t *self, ad9834_wave_t wave)
{
    // 配置 mode, opbiten, div2 三个控制位
    // 正弦: mode=0, opbiten=0
    // 三角: mode=1, opbiten=0
    // 正弦+方波: mode=0, opbiten=1, div2=1
}
```

### 影子控制寄存器设计

```
                     控制寄存器值  (内存中维护)
                           │
             修改相应位 (bitfield 操作)
                           │
                 组装 2 字节 SPI 数据
                           │
                    SPI 写入物理芯片
```

**优点**: 每次 SPI 写入都是完整控制字，无需先读芯片寄存器。

---

## 4. 外设配置 (STM32CubeMX)

### 系统时钟

```
HSE (8MHz) → PLL (×42, /4, /2) → SYSCLK = 168MHz
                                     ├── AHB = 168MHz
                                     ├── APB1 = 42MHz  (TIM2/3/4)
                                     └── APB2 = 84MHz  (SPI2, USART1)
```

### SPI2 配置

| 参数 | 值 |
|------|-----|
| Mode | Full-Duplex Master |
| Data Size | 8 Bit |
| Clock Polarity (CPOL) | HIGH |
| Clock Phase (CPHA) | 1 Edge |
| NSS | Software |
| Baud Rate | Prescaler 2 (42MHz) |
| First Bit | MSB |

### TIM2/3/4 编码器模式

| 参数 | 值 |
|------|-----|
| Mode | Encoder Mode TI1 + TI2 |
| Counter Mode | Up/Down (自动) |
| Period | 65535 |
| Encoder Mode | T1+T2 (4 倍频) |

### USART1 配置

| 参数 | 值 |
|------|-----|
| Baud Rate | 115200 |
| Word Length | 8 Bit |
| Parity | None |
| Stop Bits | 1 |

### GPIO

| 引脚 | 模式 | 初始值 |
|------|------|--------|
| PB12 (CS_A) | Output PP, High | HIGH |
| PB11 (CS_B) | Output PP, High | HIGH |
| PC0~PC2 (按键) | Input (Pull-up) | - |

---

## 5. 系统设计伪代码

### 系统状态结构

```c
/* 编码器数据结构 */
typedef struct {
    TIM_HandleTypeDef *tim;     // 关联定时器
    int16_t last_counter;       // 上次计数值
    uint16_t btn_pin;           // 按键引脚
    GPIO_TypeDef *btn_port;     // 按键端口
    uint8_t btn_state;          // 消抖状态机
    uint32_t btn_last_tick;     // 按键计时
} encoder_t;

/* 通道参数 */
typedef struct {
    uint32_t frequency_hz;      // 1 ~ 10,000,000 Hz
    int32_t  amplitude_mv;      // -18,000 ~ +18,000 mV
    uint16_t phase_x10;         // 0 ~ 3600 (0.0° ~ 360.0°)
} channel_param_t;

/* 目标通道枚举 */
typedef enum {
    TARGET_CH_A = 0,
    TARGET_CH_B = 1,
    TARGET_CH_BOTH = 2
} target_ch_t;

/* 系统状态 */
typedef struct {
    target_ch_t selected_ch;            // 当前目标通道 (KEY_CH 切换)
    ad9834_wave_t wave_type;            // 当前波形 (KEY_WAVE 切换)
    channel_param_t ch_a;               // CH A 参数
    channel_param_t ch_b;               // CH B 参数
    uint8_t digit_idx[3];               // 每个编码器的位索引
    struct {
        bool freq_dirty;
        bool amp_dirty;
        bool phase_dirty;
        bool wave_dirty;
        bool display_dirty;
    } update_flags;
} system_state_t;

/* 按键事件 */
typedef enum { EVT_NONE, EVT_PRESS } btn_event_t;
```

### 参数边界

```c
/* 频率 */
#define FREQ_MIN      1          // 1 Hz
#define FREQ_MAX      10000000   // 10 MHz
#define FREQ_DIGIT_MAX 6         // digit 0~6 (1Hz ~ 1MHz 位)

/* 幅值 */
#define AMP_MIN       (-18000)   // -18 V
#define AMP_MAX       18000      // +18 V
#define AMP_DIGIT_MAX 3          // digit 0~3 (10mV ~ 10V 位)

/* 相位 (10 倍精度) */
#define PHASE_MIN     0          // 0.0°
#define PHASE_MAX     3600       // 360.0°
#define PHASE_DIGIT_MAX 3        // digit 0~3 (0.1° ~ 100° 位)
```

### 主循环流程

```
while (1):
    // ===== STEP 1: 读取编码器增量 =====
    delta[0] = Encoder_ReadDelta(&enc_freq)   // TIM2
    delta[1] = Encoder_ReadDelta(&enc_amp)    // TIM3
    delta[2] = Encoder_ReadDelta(&enc_phase)  // TIM4

    // ===== STEP 2: 扫描编码器按键 (短按切位) =====
    for i in 0..2:
        if Button_Scan(&enc[i]) == EVT_PRESS:
            max_digit = [FREQ_DIGIT_MAX, AMP_DIGIT_MAX, PHASE_DIGIT_MAX][i]
            state.digit_idx[i] = (state.digit_idx[i] + 1) % (max_digit + 1)
            state.update_flags.display_dirty = true

    // ===== STEP 3: 独立按键 (预留) =====
    // KEY_CH:   state.selected_ch = (state.selected_ch + 1) % 3
    // KEY_WAVE: state.wave_type = (state.wave_type + 1) % 3
    //           state.update_flags.wave_dirty = true

    // ===== STEP 4: 编码器旋转调参 =====
    for each target_ch matching state.selected_ch (A, B, or Both):
        if delta[0] != 0:  // 频率
            step = {1,10,100,1k,10k,100k,1M}[state.digit_idx[0]]
            *freq += delta[0] * step
            clamp(*freq, FREQ_MIN, FREQ_MAX)
            update_flags.freq_dirty = true

        if delta[1] != 0:  // 幅值
            step = {10,100,1000,10000}[state.digit_idx[1]]
            *amp += delta[1] * step
            clamp(*amp, AMP_MIN, AMP_MAX)
            update_flags.amp_dirty = true

        if delta[2] != 0:  // 相位
            step = {1,10,100,1000}[state.digit_idx[2]]
            *phase_x10 += delta[2] * step
            clamp(*phase_x10, PHASE_MIN, PHASE_MAX)
            update_flags.phase_dirty = true

    // ===== STEP 5: 硬件更新 =====
    UpdateDDS(&state)          // AD9834: 频率+相位+波形
    UpdateAmplitude(&state)    // DAC+PGA: 幅值

    // ===== STEP 6: OLED 刷新 (100ms) =====
    if HAL_GetTick() - last_display >= 100:
        OLED_Refresh(&state)

    // ===== STEP 7: 空闲休眠 =====
    // 无事件时: __WFI() 等中断唤醒
```

### 编码器增量读取

```c
int16_t Encoder_ReadDelta(encoder_t *enc):
    current = __HAL_TIM_GET_COUNTER(enc->tim)
    raw = current - enc->last_counter
    if raw > 32767:    raw -= 65536    // 16位回绕
    if raw < -32768:   raw += 65536
    enc->last_counter = current
    if abs(raw) < 2:   return 0        // 死区去抖
    return raw > 0 ? 1 : -1            // 归一化
```

### 按键消抖状态机

```c
btn_event_t Button_Scan(encoder_t *enc):
    pin_level = HAL_GPIO_ReadPin(enc->btn_port, enc->btn_pin)
    switch enc->btn_state:
        case BTN_IDLE:
            if pin_level == GPIO_PIN_RESET:         // 按下
                enc->btn_state = BTN_DEBOUNCE
                enc->btn_last_tick = HAL_GetTick()
        case BTN_DEBOUNCE:
            if pin_level == GPIO_PIN_RESET:
                if tick - last_tick >= 10ms:         // 10ms 消抖
                    enc->btn_state = BTN_PRESSED
            else:
                enc->btn_state = BTN_IDLE            // 抖动
        case BTN_PRESSED:
            if pin_level == GPIO_PIN_SET:             // 释放
                enc->btn_state = BTN_IDLE
                return EVT_PRESS                      // 触发一次
    return EVT_NONE
```

### AD9834 硬件更新

```c
void UpdateDDS(system_state_t *state):
    if state->update_flags.freq_dirty:
        AD9834_SetFrequency(&g_ad9834_a, FREQ0_REG, state->ch_a.frequency_hz)
        AD9834_SetFrequency(&g_ad9834_b, FREQ0_REG, state->ch_b.frequency_hz)
        update_flags.freq_dirty = false

    if state->update_flags.phase_dirty:
        AD9834_SetPhase(&g_ad9834_a, PHASE0_REG, state->ch_a.phase_x10 / 10)
        AD9834_SetPhase(&g_ad9834_b, PHASE0_REG, state->ch_b.phase_x10 / 10)
        update_flags.phase_dirty = false

    if state->update_flags.wave_dirty:
        AD9834_SetOutputWave(&g_ad9834_a, state->wave_type)
        AD9834_SetOutputWave(&g_ad9834_b, state->wave_type)
        update_flags.wave_dirty = false
```

### DAC 幅值控制

```c
void UpdateAmplitude(system_state_t *state):
    if state->update_flags.amp_dirty:
        // 12bit DAC: 0~18000mV → 0~4095
        dac_a = abs(state->ch_a.amplitude_mv) * 4095 / 18000
        dac_b = abs(state->ch_b.amplitude_mv) * 4095 / 18000
        SPI_WriteDAC(CS_DAC_A, dac_a)
        SPI_WriteDAC(CS_DAC_B, dac_b)
        // 极性: 负幅值翻转极性 GPIO
        HAL_GPIO_WritePin(POL_A, state->ch_a.amplitude_mv < 0)
        HAL_GPIO_WritePin(POL_B, state->ch_b.amplitude_mv < 0)
        update_flags.amp_dirty = false
```

### OLED 显示

```c
void OLED_Refresh(system_state_t *state):
    OLED_Clear()
    OLED_Printf(0, 0, "Freq A:%luHz",  state->ch_a.frequency_hz)
    OLED_Printf(0, 1, "Freq B:%luHz",  state->ch_b.frequency_hz)
    OLED_Printf(0, 2, "Amp A:%+ldmV", state->ch_a.amplitude_mv)
    OLED_Printf(0, 3, "Amp B:%+ldmV", state->ch_b.amplitude_mv)
    OLED_Printf(0, 4, "Phase A:%u.%u°", state->ch_a.phase_x10/10, state->ch_a.phase_x10%10)
    OLED_Printf(0, 5, "Phase B:%u.%u°", state->ch_b.phase_x10/10, state->ch_b.phase_x10%10)
    OLED_Printf(0, 6, "CH:%s Wave:%s Dig:F%d A%d P%d",
        ch_name[selected_ch], wave_name[wave_type],
        digit_idx[0], digit_idx[1], digit_idx[2])
    OLED_ShowCursor(cursor_pos_for_selected_digit)
```

---

## 6. 交互逻辑

### 操作映射

| 操作 | 作用 | 实现 |
|------|------|------|
| **ENC1 旋转** | 增大/减小频率 (按 digit 位权) | TIM2 编码器模式 |
| **ENC2 旋转** | 增大/减小幅值 (按 digit 位权) | TIM3 编码器模式 |
| **ENC3 旋转** | 增大/减小相位 (按 digit 位权) | TIM4 编码器模式 |
| **ENC1 按键** | 循环切换频率的编辑位 | 轮询消抖 + 状态机 |
| **ENC2 按键** | 循环切换幅值的编辑位 | 轮询消抖 + 状态机 |
| **ENC3 按键** | 循环切换相位的编辑位 | 轮询消抖 + 状态机 |
| **KEY_CH** (预留) | 切换目标通道 A/B/BOTH | 独立按键中断或轮询 |
| **KEY_WAVE** (预留) | 循环切换波形 | 独立按键中断或轮询 |

### 状态机流程

```
                            ┌──────────────┐
                   KEY_CH   │              │   KEY_CH
              ┌────────────→┤  TARGET_CH_A  ├←────────────┐
              │             │              │              │
              │             └──────┬───────┘              │
              │                    │ KEY_CH               │
              │             ┌──────▼───────┐              │
              │             │              │              │
              └─────────────┤ TARGET_CH_B  ├──────────────┘
                            │              │
                            └──────┬───────┘
                                   │ KEY_CH
                            ┌──────▼───────┐
                            │              │
                            │TARGET_CH_BOTH│
                            │              │
                            └──────────────┘

        选定通道后, 3 个编码器分别调节该通道的频率/幅值/相位
        编码器按键切换的是当前参数的编辑位 (digit), 而非通道
```

### Digit 编辑示例

```
频率 1234567 Hz, digit_idx=2 (100Hz 位):
  向右转 1 格 → 1234567 + 100 = 1234667 Hz
  向左转 1 格 → 1234567 - 100 = 1234467 Hz

频率 1999999 Hz, digit_idx=5 (100kHz 位):
  向右转 1 格 → 1999999 + 100000 = 2099999 Hz (自动进位)

幅值 +10500 mV, digit_idx=1 (100mV 位):
  向右转 1 格 → 10500 + 100 = 10600 mV
```

---

## 7. 算法说明

### AD9834 频率字计算

```
频率字 = frequency × 2²⁸ / MCLK
       = frequency × 268435456 / 75000000

写入方式 (B28=1):
  先写 LSB (14bit) → 再写 MSB (14bit)
  SPI 数据格式: [D15..D8] [D7..D0]
  频率寄存器地址编码: D15, D14
    FREQ0: LSB = 0x2000 | data, MSB = 0x4000 | data
    FREQ1: LSB = 0x8000 | data, MSB = 0xA000 | data
```

### AD9834 相位字计算

```
相位字 = angle × 2¹² / 360
       = angle × 4096 / 360

写入格式: D15=1, D14=1, D13=地址(0/1), D11..D0=数据
```

### DAC 幅值映射

```
DAC_Out = abs(amplitude_mv) × 4095 / 18000
         = abs(amplitude_mv) × 4095 / 18000

幅值极性: 通过独立 GPIO 控制极性继电器或 PGA 差分输出
```

---

## 8. 后续工作

- [ ] **独立按键接入**: KEY_CH (通道切换) + KEY_WAVE (波形切换)
  - 可使用 GPIO EXTI 中断或定时器轮询
- [ ] **DAC 硬件选型**: 确认 DAC 型号 (MCP4921, DAC7551 等), 设计 PGA 增益电路
- [ ] **OLED 驱动**: 实现 `OLED_Init()`, `OLED_Printf()`, `OLED_ShowCursor()`
- [ ] **频率双路独立**: 当前双路同频, 如需独立频率需增加硬件 (或改用双 DDS)
- [ ] **幅值 ±18V 链路**: 设计 DAC → PGA → 功放 → ±18V 电路, 含极性控制
- [ ] **上位机通信**: 通过 USART1 实现 PC 远程控制协议
- [ ] **校准**: 频率准确度校准、幅值校准、相位偏移校准
- [ ] 基于该伪代码填充具体的 C 代码实现

---

*生成日期: 2026-05-29*
*MCU: STM32F407VGT6 | DDS: AD9834 × 2 | 编码器: EC11 × 3*
