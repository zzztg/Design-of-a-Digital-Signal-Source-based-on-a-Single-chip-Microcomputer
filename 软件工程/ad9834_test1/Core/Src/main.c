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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad9834.h"
#include <stdio.h>
#include <stdlib.h>

/* ============================================================
 * 伪代码 — 双路 DDS 数字信号源系统设计
 * 硬件资源:
 *   TIM2 编码器模式 → 频率调节 (Encoder1)
 *   TIM3 编码器模式 → 幅值调节 (Encoder2)
 *   TIM4 编码器模式 → 相位调节 (Encoder3)
 *   SPI2 → AD9834 (DDS 频率+相位) + DAC (幅值控制) 共用总线
 *          AD9834_A 片选 PB12, AD9834_B 片选 PB11
 *          DAC_A 片选 xx, DAC_B 片选 xx (独立 CS)
 *   PGA (可编程增益放大器) → 配合 DAC 实现 ±18V 输出
 *   I2C1 → OLED 显示屏
 *   USART1 → printf 调试
 * ============================================================ */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 * 编码器数据结构
 * encoder_t:
 *     tim: TIM_HandleTypeDef*   // 关联的定时器(编码器模式)
 *     last_counter: int16_t     // 上次计数值
 *     delta: int16_t            // 本次增量
 *     button: GPIO_Pin          // 按键引脚
 *     button_port: GPIO_Port    // 按键端口
 *     btn_state: uint8_t        // 按键消抖状态机
 *     btn_last_tick: uint32_t   // 按键最后触发时间
 *
 * encoder_button_state:          // 按键消抖 FSM (10ms 轮询)
 *   STATE_IDLE:                  // 等待按下
 *     on 按下 → STATE_DEBOUNCE
 *   STATE_DEBOUNCE:              // 消抖等待
 *     if 10ms 后仍按下 → STATE_PRESSED
 *     else → STATE_IDLE
 *   STATE_PRESSED:               // 已确认按下
 *     if 释放 → 触发 EVT_PRESS  → STATE_IDLE
 */

/*
 * 选中位索引 (digit index per parameter)
 *   频率: digit=0 → 1Hz, digit=1 → 10Hz, ..., digit=6 → 10MHz
 *   幅值: digit=0 → 10mV, digit=1 → 100mV, ..., digit=3 → 10V
 *   相位: digit=0 → 0.1°, digit=1 → 1°, digit=2 → 10°, digit=3 → 100°
 */

/*
 * 目标通道枚举
 * target_ch_t:
 *     TARGET_CH_A = 0     // 仅调节 CH A
 *     TARGET_CH_B = 1     // 仅调节 CH B
 *     TARGET_CH_BOTH = 2  // 同时调节
 */

/*
 * 系统状态结构
 * system_state_t:
 *     selected_ch: target_ch_t     // 当前目标通道 (由独立按键 KEY_CH 切换)
 *     wave_type: ad9834_wave_t     // 当前波形类型 (由独立按键 KEY_WAVE 切换)
 *     ch_a:                        // CH A 参数
 *         frequency_hz: uint32_t   // 1 ~ 10,000,000
 *         amplitude_mv: int32_t    // -18000 ~ +18000
 *         phase_x10: uint16_t      // 0 ~ 3600 (0.0° ~ 360.0°)
 *     ch_b:                        // CH B 参数 (同上)
 *     digit_idx: [3]               // 每个编码器当前的位索引
 *     update_flags:                // 脏标记
 *         freq_dirty: bool
 *         amp_dirty: bool
 *         phase_dirty: bool
 *         wave_dirty: bool
 *         display_dirty: bool
 */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * EC11 编码器按键消抖
 */
#define BTN_DEBOUNCE_MS    10     // 消抖延时 (ms)

/*
 * 独立按键 (预留, 后续添加)
 *   KEY_CH:   目标通道切换  CH_A → CH_B → BOTH → CH_A
 *   KEY_WAVE: 波形切换     Sine → Triangle → Sine+Square → Sine
 *   目前为伪代码占位, 后续接入 GPIO EXTI 中断或轮询
 */

/*
 * 频率边界
 */
#define FREQ_MIN           1
#define FREQ_MAX           10000000

/*
 * 幅值边界 (mV)
 */
#define AMP_MIN           (-18000)
#define AMP_MAX           18000

/*
 * 相位边界 (内部 10 倍精度: 0~3600 对应 0.0°~360.0°)
 */
#define PHASE_MIN          0
#define PHASE_MAX          3600

/*
 * 每个参数的位权表 (digit index → 步进值)
 *   频率: digit 0..6 → 1, 10, 100, 1000, 10000, 100000, 1000000 Hz
 *   幅值: digit 0..3 → 10, 100, 1000, 10000 mV  (即 0.01V, 0.1V, 1V, 10V)
 *   相位: digit 0..3 → 1, 10, 100, 1000 (即 0.1°, 1°, 10°, 100°)
 */
#define FREQ_DIGIT_MAX     6     // digit 0~6
#define AMP_DIGIT_MAX      3     // digit 0~3
#define PHASE_DIGIT_MAX    3     // digit 0~3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ad9834_t g_ad9834_a;
ad9834_t g_ad9834_b;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * ============================================================
 *  伪代码: 编码器增量读取
 * ============================================================
 *
 * int16_t Encoder_ReadDelta(encoder_t *enc):
 *     current = __HAL_TIM_GET_COUNTER(enc->tim)
 *     raw = current - enc->last_counter
 *     if raw > 32767:   raw -= 65536    // 16位回绕处理
 *     if raw < -32768:  raw += 65536
 *     enc->last_counter = current
 *     // 死区: |raw| < 2 认为是抖动, 返回0
 *     if abs(raw) < 2:  return 0
 *     // 归一化: 每次只返回 ±1 (转1格)
 *     return raw > 0 ? 1 : -1
 *
 *
 * ============================================================
 *  伪代码: 按键消抖状态机 (10ms 周期调用)
 * ============================================================
 *
 * typedef enum { BTN_IDLE, BTN_DEBOUNCE, BTN_PRESSED } btn_state_t;
 * typedef enum { EVT_NONE, EVT_PRESS } btn_event_t;
 *
 * btn_event_t Button_Scan(encoder_t *enc):
 *     pin_level = HAL_GPIO_ReadPin(enc->btn_port, enc->btn_pin)
 *     tick = HAL_GetTick()
 *     switch enc->btn_state:
 *         case BTN_IDLE:
 *             if pin_level == GPIO_PIN_RESET:       // EC11 按键低电平=按下
 *                 enc->btn_state = BTN_DEBOUNCE
 *                 enc->btn_last_tick = tick
 *         case BTN_DEBOUNCE:
 *             if pin_level == GPIO_PIN_RESET:
 *                 if tick - enc->btn_last_tick >= BTN_DEBOUNCE_MS:
 *                     enc->btn_state = BTN_PRESSED  // 确认按下
 *             else:
 *                 enc->btn_state = BTN_IDLE         // 抖动, 复位
 *         case BTN_PRESSED:
 *             if pin_level == GPIO_PIN_SET:           // 释放
 *                 enc->btn_state = BTN_IDLE
 *                 return EVT_PRESS                   // 触发一次
 *     return EVT_NONE
 *
 *
 * ============================================================
 *  伪代码: 按位编辑 — 频率
 * ============================================================
 *
 * void AdjustFrequencyByDigit(uint32_t *freq, int digit_idx, int direction):
 *     // digit_idx: 0=1Hz, 1=10Hz, ..., 6=10MHz
 *     // direction: +1 增加, -1 减小
 *     step[] = {1, 10, 100, 1000, 10000, 100000, 1000000}
 *     new = *freq + direction * step[digit_idx]
 *     // 借位/进位处理: 加到某位溢出时自动进到下一位
 *     if new < FREQ_MIN:  new = FREQ_MIN
 *     if new > FREQ_MAX:  new = FREQ_MAX
 *     *freq = new
 *
 * // 举例: freq=1234567 Hz, digit_idx=2 (100Hz位), direction=+1
 * //    → 1234567 + 100 = 1234667 Hz
 * // 举例: freq=1999999 Hz, digit_idx=5 (100kHz位), direction=+1
 * //    → 1999999 + 100000 = 2099999 Hz (进位自动)
 *
 *
 * ============================================================
 *  伪代码: 按位编辑 — 幅值
 * ============================================================
 *
 * void AdjustAmplitudeByDigit(int32_t *amp, int digit_idx, int direction):
 *     step[] = {10, 100, 1000, 10000}  // mV
 *     new = *amp + direction * step[digit_idx]
 *     if new < AMP_MIN:  new = AMP_MIN
 *     if new > AMP_MAX:  new = AMP_MAX
 *     *amp = new
 *
 *
 * ============================================================
 *  伪代码: 按位编辑 — 相位
 * ============================================================
 *
 * void AdjustPhaseByDigit(uint16_t *phase_x10, int digit_idx, int direction):
 *     step[] = {1, 10, 100, 1000}      // 对应 0.1°, 1°, 10°, 100°
 *     new = *phase_x10 + direction * step[digit_idx]
 *     if new < PHASE_MIN:  new = PHASE_MIN
 *     if new > PHASE_MAX:  new = PHASE_MAX
 *     *phase_x10 = new
 *
 *
 * ============================================================
 *  伪代码: 更新硬件 — AD9834 (频率+相位+波形)
 * ============================================================
 *
 * void UpdateDDS(system_state_t *state):
 *     // 频率脏 → 写 AD9834
 *     if state->update_flags.freq_dirty:
 *         AD9834_SetFrequency(&g_ad9834_a, FREQ0_REG,
 *                             state->ch_a.frequency_hz)
 *         AD9834_SetFrequency(&g_ad9834_b, FREQ0_REG,
 *                             state->ch_b.frequency_hz)
 *         state->update_flags.freq_dirty = false
 *
 *     // 相位脏 → 写 AD9834
 *     if state->update_flags.phase_dirty:
 *         AD9834_SetPhase(&g_ad9834_a, PHASE0_REG,
 *                         state->ch_a.phase_x10 / 10)
 *         AD9834_SetPhase(&g_ad9834_b, PHASE0_REG,
 *                         state->ch_b.phase_x10 / 10)
 *         state->update_flags.phase_dirty = false
 *
 *     // 波形脏 → 写 AD9834
 *     if state->update_flags.wave_dirty:
 *         AD9834_SetOutputWave(&g_ad9834_a, state->wave_type)
 *         AD9834_SetOutputWave(&g_ad9834_b, state->wave_type)
 *         state->update_flags.wave_dirty = false
 *
 *
 * ============================================================
 *  伪代码: 更新硬件 — DAC (幅值)
 * ============================================================
 *
 * void UpdateAmplitude(system_state_t *state):
 *     if state->update_flags.amp_dirty:
 *         // DAC 输出 = map(幅值 0~18000mV, 0~4095)
 *         dac_a = abs(state->ch_a.amplitude_mv) * 4095 / 18000
 *         dac_b = abs(state->ch_b.amplitude_mv) * 4095 / 18000
 *         SPI_WriteDAC(DAC_CS_A, dac_a)
 *         SPI_WriteDAC(DAC_CS_B, dac_b)
 *         // 极性控制: 负幅值 → 翻转极性 GPIO
 *         HAL_GPIO_WritePin(POL_A_Port, POL_A_Pin,
 *             state->ch_a.amplitude_mv < 0 ? SET : RESET)
 *         HAL_GPIO_WritePin(POL_B_Port, POL_B_Pin,
 *             state->ch_b.amplitude_mv < 0 ? SET : RESET)
 *         state->update_flags.amp_dirty = false
 *
 *
 * ============================================================
 *  伪代码: OLED 显示刷新
 * ============================================================
 *
 * void OLED_Refresh(system_state_t *state):
 *     OLED_Clear()
 *     // 第1行: 频率 (选中位加光标)
 *     OLED_Printf(0, 0, "Freq A:%luHz", state->ch_a.frequency_hz)
 *     OLED_Printf(0, 1, "Freq B:%luHz", state->ch_b.frequency_hz)
 *     // 第3行: 幅值
 *     OLED_Printf(0, 2, "Amp A:%+ldmV", state->ch_a.amplitude_mv)
 *     OLED_Printf(0, 3, "Amp B:%+ldmV", state->ch_b.amplitude_mv)
 *     // 第5行: 相位
 *     OLED_Printf(0, 4, "Phase A:%u.%u°",
 *         state->ch_a.phase_x10/10, state->ch_a.phase_x10%10)
 *     OLED_Printf(0, 5, "Phase B:%u.%u°",
 *         state->ch_b.phase_x10/10, state->ch_b.phase_x10%10)
 *     // 第7行: 状态栏 (当前选中的通道/位)
 *     OLED_Printf(0, 6, "F-d%d A-d%d P-d%d",
 *         freq_digit, amp_digit, phase_digit)
 *     // 在选中位上显示下划线光标
 *     OLED_ShowCursor(cursor_pos)
 */

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
  /*
   * 伪代码: 初始化阶段
   *
   * // 1. 初始化三个编码器定时器 (编码器模式)
   * MX_TIM2_Init();                        // ENC1 → 频率
   * MX_TIM3_Init();                        // ENC2 → 幅值
   * MX_TIM4_Init();                        // ENC3 → 相位
   * HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
   * HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
   * HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
   *
   * // 2. 初始化编码器按键 GPIO (EC11 按键引脚, CubeMX 已配置)
   * //    假设: ENC1_BTN → PC0, ENC2_BTN → PC1, ENC3_BTN → PC2
   *
   * // 3. 初始化 DAC (幅值控制, 与 AD9834 共用 SPI2)
   * MX_DAC_Init();  // 初始化 DAC 片选等
   *
   * // 4. 初始化 OLED (I2C1)
   * MX_I2C1_Init();
   * OLED_Init();  OLED_Clear();
   *
   * // 5. 系统参数初始值
   * state.selected_ch = TARGET_CH_BOTH  // 默认同时调节两路
   * state.wave_type   = AD9834_WaveSine // 默认正弦波
   * state.ch_a.frequency_hz = 1000
   * state.ch_a.amplitude_mv = 1000
   * state.ch_a.phase_x10   = 0
   * state.ch_b.frequency_hz = 1000
   * state.ch_b.amplitude_mv = 1000
   * state.ch_b.phase_x10   = 0
   * state.digit_idx[0] = 3    // 频率默认选中 kHz 位
   * state.digit_idx[1] = 1    // 幅值默认选中 100mV 位
   * state.digit_idx[2] = 1    // 相位默认选中 1° 位
   */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  printf("*** 双路 DDS 信号源启动 ***\r\n");

  /*  伪代码: 初始化示例配置
   *
   *  // 初始化两个 AD9834 实例
   *  AD9834_Init(&g_ad9834_a, CS_A_Pin, CS_A_GPIO_Port);
   *  AD9834_Init(&g_ad9834_b, CS_B_Pin, CS_B_GPIO_Port);
   *
   *  // 恢复复位 (AD9834 上电后需要复位+释放)
   *  AD9834_SetReset(&g_ad9834_a, AD9834_BitSet);
   *  AD9834_SetReset(&g_ad9834_a, AD9834_BitReset);
   *  AD9834_SetReset(&g_ad9834_b, AD9834_BitSet);
   *  AD9834_SetReset(&g_ad9834_b, AD9834_BitReset);
   *
   *  // 设置默认参数 1kHz 正弦波
   *  AD9834_SetFrequency(&g_ad9834_a, FREQ0_REG, state.ch_a.frequency_hz);
   *  AD9834_SetPhase(&g_ad9834_a, PHASE0_REG, 0);
   *  AD9834_SelectFrequencyRegOutput(&g_ad9834_a, FREQ0_REG);
   *  AD9834_SelectPhaseRegOutput(&g_ad9834_a, PHASE0_REG);
   *  AD9834_SetOutputWave(&g_ad9834_a, AD9834_WaveSine);
   *
   *  AD9834_SetFrequency(&g_ad9834_b, FREQ0_REG, state.ch_b.frequency_hz);
   *  AD9834_SetPhase(&g_ad9834_b, PHASE0_REG, 0);
   *  AD9834_SelectFrequencyRegOutput(&g_ad9834_b, FREQ0_REG);
   *  AD9834_SelectPhaseRegOutput(&g_ad9834_b, PHASE0_REG);
   *  AD9834_SetOutputWave(&g_ad9834_b, AD9834_WaveSine);
   */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /*
       * ============================================================
       *  伪代码: 主循环 — 状态机驱动 (先选通道 → 再调参数)
       * ============================================================
       *
       * // ========== STEP 1: 读取编码器增量 ==========
       * delta[0] = Encoder_ReadDelta(&enc_freq)   // TIM2
       * delta[1] = Encoder_ReadDelta(&enc_amp)    // TIM3
       * delta[2] = Encoder_ReadDelta(&enc_phase)  // TIM4
       *
       * // ========== STEP 2: 扫描编码器按键 ==========
       * for i in 0..2:
       *     if Button_Scan(&enc[i]) == EVT_PRESS:
       *         max_digit = [FREQ_DIGIT_MAX,
       *                      AMP_DIGIT_MAX,
       *                      PHASE_DIGIT_MAX][i]
       *         // 编码器按键 → 循环切换当前参数的编辑位
       *         state.digit_idx[i] =
       *             (state.digit_idx[i] + 1) % (max_digit + 1)
       *         printf("ENC%d digit now: %d\r\n", i, digit_idx)
       *         state.update_flags.display_dirty = true
       *
       * // ========== STEP 3: 扫描独立按键 (预留) ==========
       *
       * // [KEY_CH] 通道切换 — 后续接入独立按键
       * // if KEY_CH_Pressed():
       * //     state.selected_ch = (state.selected_ch + 1) % 3
       * //     printf("Target: %s\r\n", ch_name[selected_ch])
       * //     state.update_flags.display_dirty = true
       *
       * // [KEY_WAVE] 波形切换 — 后续接入独立按键
       * // if KEY_WAVE_Pressed():
       * //     state.wave_type = (state.wave_type + 1) % 3
       * //     state.update_flags.wave_dirty = true
       * //     state.update_flags.display_dirty = true
       *
       * // 当前流程 (独立按键未接入前的默认行为):
       * //   selected_ch = TARGET_CH_BOTH (同时调节两路)
       * //   wave_type   = AD9834_WaveSine
       *
       * // ========== STEP 4: 编码器旋转 → 调参 ==========
       * // 只修改 state.selected_ch 指定的通道
       * for each target_ch in [CH_A, CH_B] that
       *         matches state.selected_ch:
       *
       *     // --- 频率 ---
       *     if delta[0] != 0:
       *         step = {1,10,100,1k,10k,100k,1M}[state.digit_idx[0]]
       *         *freq += delta[0] * step
       *         clamp(*freq, FREQ_MIN, FREQ_MAX)
       *         update_flags.freq_dirty = true
       *
       *     // --- 幅值 ---
       *     if delta[1] != 0:
       *         step = {10,100,1000,10000}[state.digit_idx[1]]
       *         *amp += delta[1] * step
       *         clamp(*amp, AMP_MIN, AMP_MAX)
       *         update_flags.amp_dirty = true
       *
       *     // --- 相位 ---
       *     if delta[2] != 0:
       *         step = {1,10,100,1000}[state.digit_idx[2]]
       *         *phase_x10 += delta[2] * step
       *         clamp(*phase_x10, PHASE_MIN, PHASE_MAX)
       *         update_flags.phase_dirty = true
       *
       *     printf("CH%c Freq:%lu Amp:%+ld Phase:%u.%u°\r\n",
       *            ch, freq, amp, phase/10, phase%10)
       *
       * // ========== STEP 5: 硬件更新 ==========
       * UpdateDDS(&state)         // 频率/相位/波形 → AD9834
       * UpdateAmplitude(&state)   // 幅值 → DAC+PGA
       *
       * // ========== STEP 6: OLED 刷新 (100ms) ==========
       * if HAL_GetTick() - last_display >= 100:
       *     last_display = HAL_GetTick()
       *     OLED_Refresh(&state)
       *
       * // ========== STEP 7: 空闲休眠 ==========
       * // if all deltas == 0 and no events:
       * //     __WFI()  // 等待中断唤醒
       *
       */// end pseudo-code
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
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
