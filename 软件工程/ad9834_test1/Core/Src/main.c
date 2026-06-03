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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ad9834.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief 编码器数据结构
 */
typedef struct {
    TIM_HandleTypeDef *tim;
    int16_t last_counter;
} encoder_t;

typedef enum {
    BTN_IDLE = 0, BTN_DEBOUNCE, BTN_PRESSED,
} btn_state_t;

typedef enum {
    EVT_NONE = 0, EVT_PRESS,
} btn_event_t;

typedef struct {
    uint16_t pin;
    GPIO_TypeDef *port;
    btn_state_t state;
    uint32_t last_tick;
} button_t;

typedef enum {
    TARGET_CH_A = 0, TARGET_CH_B = 1, TARGET_CH_BOTH = 2,
} target_ch_t;

typedef struct {
    uint32_t frequency_hz;
    int32_t amplitude_mv;
    uint16_t phase_x10;
} channel_params_t;

typedef struct {
    uint8_t freq_dirty : 1;
    uint8_t amp_dirty : 1;
    uint8_t phase_dirty : 1;
    uint8_t wave_dirty : 1;
} update_flags_t;

typedef struct {
    target_ch_t selected_ch;
    ad9834_wave_t wave_type;
    channel_params_t ch_a;
    channel_params_t ch_b;
    uint8_t digit_idx[3];
    update_flags_t flags;
} system_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BTN_DEBOUNCE_MS         10

#define FREQ_MIN                1
#define FREQ_MAX                10000000UL

#define AMP_MIN                 (-18000)
#define AMP_MAX                 18000

#define PHASE_MIN               0
#define PHASE_MAX               3600

#define FREQ_DIGIT_MAX          6
#define AMP_DIGIT_MAX           3
#define PHASE_DIGIT_MAX         3

static const uint32_t FREQ_STEPS[]  = {1, 10, 100, 1000, 10000, 100000, 1000000};
static const int32_t  AMP_STEPS[]   = {10, 100, 1000, 10000};
static const uint16_t PHASE_STEPS[] = {1, 10, 100, 1000};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static ad9834_t g_ad9834_a;
static ad9834_t g_ad9834_b;

static encoder_t g_enc_freq;
static encoder_t g_enc_amp;
static encoder_t g_enc_phase;

static button_t g_btn_freq  = {KEY_ENC1_Pin, KEY_ENC1_GPIO_Port};
static button_t g_btn_amp   = {KEY_ENC2_Pin, KEY_ENC2_GPIO_Port};
static button_t g_btn_phase = {KEY_ENC3_Pin, KEY_ENC3_GPIO_Port};
static button_t g_btn_ch    = {KEY_SW1_Pin, KEY_SW1_GPIO_Port};
static button_t g_btn_wave  = {KEY_SW2_Pin, KEY_SW2_GPIO_Port};

static system_state_t g_state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static int16_t Encoder_ReadDelta(encoder_t *enc);
static btn_event_t Button_Scan(button_t *btn);
static void AdjustFrequency(channel_params_t *ch, int8_t delta, uint8_t digit_idx);
static void AdjustAmplitude(channel_params_t *ch, int8_t delta, uint8_t digit_idx);
static void AdjustPhase(channel_params_t *ch, int8_t delta, uint8_t digit_idx);
static void SetWaveformGPIO(ad9834_wave_t wave);
static void UpdateDDS(system_state_t *state);
static void UpdateAmplitude(system_state_t *state);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int16_t Encoder_ReadDelta(encoder_t *enc)
{
    int16_t current = (int16_t)__HAL_TIM_GET_COUNTER(enc->tim);
    int16_t raw = current - enc->last_counter;
    enc->last_counter = current;
    if (raw < -32767) raw += 65536;
    if (raw > 32767)  raw -= 65536;
    if (raw > -2 && raw < 2) return 0;
    return raw > 0 ? 1 : -1;
}

static btn_event_t Button_Scan(button_t *btn)
{
    GPIO_PinState level = HAL_GPIO_ReadPin(btn->port, btn->pin);
    uint32_t tick = HAL_GetTick();
    switch (btn->state) {
    case BTN_IDLE:
        if (level == GPIO_PIN_RESET) {
            btn->state = BTN_DEBOUNCE;
            btn->last_tick = tick;
        }
        break;
    case BTN_DEBOUNCE:
        if (level == GPIO_PIN_RESET) {
            if (tick - btn->last_tick >= BTN_DEBOUNCE_MS)
                btn->state = BTN_PRESSED;
        } else {
            btn->state = BTN_IDLE;
        }
        break;
    case BTN_PRESSED:
        if (level == GPIO_PIN_SET) {
            btn->state = BTN_IDLE;
            return EVT_PRESS;
        }
        break;
    }
    return EVT_NONE;
}

static void AdjustFrequency(channel_params_t *ch, int8_t delta, uint8_t digit_idx)
{
    if (digit_idx > FREQ_DIGIT_MAX) return;
    int32_t new_val = (int32_t)ch->frequency_hz + delta * (int32_t)FREQ_STEPS[digit_idx];
    if (new_val < FREQ_MIN)          new_val = FREQ_MIN;
    if (new_val > (int32_t)FREQ_MAX) new_val = (int32_t)FREQ_MAX;
    ch->frequency_hz = (uint32_t)new_val;
}

static void AdjustAmplitude(channel_params_t *ch, int8_t delta, uint8_t digit_idx)
{
    if (digit_idx > AMP_DIGIT_MAX) return;
    int32_t new_val = ch->amplitude_mv + delta * AMP_STEPS[digit_idx];
    if (new_val < AMP_MIN) new_val = AMP_MIN;
    if (new_val > AMP_MAX) new_val = AMP_MAX;
    ch->amplitude_mv = new_val;
}

static void AdjustPhase(channel_params_t *ch, int8_t delta, uint8_t digit_idx)
{
    if (digit_idx > PHASE_DIGIT_MAX) return;
    int16_t new_val = (int16_t)ch->phase_x10 + delta * (int16_t)PHASE_STEPS[digit_idx];
    if (new_val < PHASE_MIN) new_val = PHASE_MIN;
    if (new_val > PHASE_MAX) new_val = PHASE_MAX;
    ch->phase_x10 = (uint16_t)new_val;
}

static void SetWaveformGPIO(ad9834_wave_t wave)
{
    GPIO_PinState level = (wave == AD9834_WaveSineWithSquare)
                          ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(WAVE_SEL_A_GPIO_Port, WAVE_SEL_A_Pin, level);
    HAL_GPIO_WritePin(WAVE_SEL_B_GPIO_Port, WAVE_SEL_B_Pin, level);
}

static void UpdateDDS(system_state_t *state)
{
    if (!state->flags.freq_dirty && !state->flags.phase_dirty && !state->flags.wave_dirty)
        return;

    HAL_GPIO_WritePin(SYNC_RESET_GPIO_Port, SYNC_RESET_Pin, GPIO_PIN_SET);

    if (state->flags.wave_dirty) {
        AD9834_SetOutputWave(&g_ad9834_a, state->wave_type);
        AD9834_SetOutputWave(&g_ad9834_b, state->wave_type);
        SetWaveformGPIO(state->wave_type);
        state->flags.wave_dirty = 0;
    }
    if (state->flags.freq_dirty) {
        AD9834_SetFrequency(&g_ad9834_a, FREQ0_REG, state->ch_a.frequency_hz);
        AD9834_SetFrequency(&g_ad9834_b, FREQ0_REG, state->ch_b.frequency_hz);
        AD9834_SelectFrequencyRegOutput(&g_ad9834_a, FREQ0_REG);
        AD9834_SelectFrequencyRegOutput(&g_ad9834_b, FREQ0_REG);
        state->flags.freq_dirty = 0;
    }
    if (state->flags.phase_dirty) {
        AD9834_SetPhase(&g_ad9834_a, PHASE0_REG, state->ch_a.phase_x10 / 10);
        AD9834_SetPhase(&g_ad9834_b, PHASE0_REG, state->ch_b.phase_x10 / 10);
        AD9834_SelectPhaseRegOutput(&g_ad9834_a, PHASE0_REG);
        AD9834_SelectPhaseRegOutput(&g_ad9834_b, PHASE0_REG);
        state->flags.phase_dirty = 0;
    }

    HAL_GPIO_WritePin(SYNC_RESET_GPIO_Port, SYNC_RESET_Pin, GPIO_PIN_RESET);
}

static void UpdateAmplitude(system_state_t *state)
{
    if (!state->flags.amp_dirty) return;
    /*
     * 伪代码: DAC 幅值控制
     * uint16_t dac_a = abs(state->ch_a.amplitude_mv) * 4095 / 18000;
     * uint16_t dac_b = abs(state->ch_b.amplitude_mv) * 4095 / 18000;
     * HAL_GPIO_WritePin(CS_DAC_GPIO_Port, CS_DAC_Pin, GPIO_PIN_RESET);
     * uint8_t data_a[] = { (dac_a >> 8) & 0xFF, dac_a & 0xFF };
     * HAL_SPI_Transmit(&hspi2, data_a, 2, HAL_MAX_DELAY);
     * HAL_GPIO_WritePin(CS_DAC_GPIO_Port, CS_DAC_Pin, GPIO_PIN_SET);
     * // 同上写入 DAC_B
     * // 极性: HAL_GPIO_WritePin(POL_x_Port, POL_x_Pin,
     * //   state->ch_x.amplitude_mv < 0 ? SET : RESET);
     */
    state->flags.amp_dirty = 0;
}

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

  /* (系统状态初始化在 MX 初始化完成后进行) */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  printf("*** DDS START ***\r\n");

  AD9834_Init(&g_ad9834_a, &hspi2, CS_A_Pin, CS_A_GPIO_Port);
  AD9834_Init(&g_ad9834_b, &hspi2, CS_B_Pin, CS_B_GPIO_Port);

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  g_enc_freq.tim  = &htim2;
  g_enc_freq.last_counter = 0;
  g_enc_amp.tim   = &htim3;
  g_enc_amp.last_counter = 0;
  g_enc_phase.tim = &htim1;
  g_enc_phase.last_counter = 0;

  g_state.selected_ch = TARGET_CH_BOTH;
  g_state.wave_type   = AD9834_WaveSine;
  g_state.ch_a.frequency_hz  = 1000;
  g_state.ch_a.amplitude_mv  = 1000;
  g_state.ch_a.phase_x10     = 0;
  g_state.ch_b.frequency_hz  = 1000;
  g_state.ch_b.amplitude_mv  = 1000;
  g_state.ch_b.phase_x10     = 0;
  g_state.digit_idx[0] = 3;
  g_state.digit_idx[1] = 1;
  g_state.digit_idx[2] = 1;
  g_state.flags.freq_dirty  = 1;
  g_state.flags.phase_dirty = 1;
  g_state.flags.wave_dirty  = 1;
  g_state.flags.amp_dirty   = 0;

  UpdateDDS(&g_state);

  printf("A:%lu %+ld %u.%u  B:%lu %+ld %u.%u\r\n",
         g_state.ch_a.frequency_hz, g_state.ch_a.amplitude_mv,
         g_state.ch_a.phase_x10/10, g_state.ch_a.phase_x10%10,
         g_state.ch_b.frequency_hz, g_state.ch_b.amplitude_mv,
         g_state.ch_b.phase_x10/10, g_state.ch_b.phase_x10%10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      const uint8_t digit_max[] = {FREQ_DIGIT_MAX, AMP_DIGIT_MAX, PHASE_DIGIT_MAX};

      int8_t df = (int8_t)Encoder_ReadDelta(&g_enc_freq);
      int8_t da = (int8_t)Encoder_ReadDelta(&g_enc_amp);
      int8_t dp = (int8_t)Encoder_ReadDelta(&g_enc_phase);

      if (Button_Scan(&g_btn_freq) == EVT_PRESS) {
          g_state.digit_idx[0] = (g_state.digit_idx[0] + 1) % (digit_max[0] + 1);
          printf("F%d\r\n", g_state.digit_idx[0]);
      }
      if (Button_Scan(&g_btn_amp) == EVT_PRESS) {
          g_state.digit_idx[1] = (g_state.digit_idx[1] + 1) % (digit_max[1] + 1);
          printf("A%d\r\n", g_state.digit_idx[1]);
      }
      if (Button_Scan(&g_btn_phase) == EVT_PRESS) {
          g_state.digit_idx[2] = (g_state.digit_idx[2] + 1) % (digit_max[2] + 1);
          printf("P%d\r\n", g_state.digit_idx[2]);
      }

      if (Button_Scan(&g_btn_ch) == EVT_PRESS) {
          g_state.selected_ch = (g_state.selected_ch + 1) % 3;
          printf("CH:%s\r\n", g_state.selected_ch==0?"A":g_state.selected_ch==1?"B":"AB");
      }
      if (Button_Scan(&g_btn_wave) == EVT_PRESS) {
          g_state.wave_type = (ad9834_wave_t)((g_state.wave_type + 1) % 3);
          g_state.flags.wave_dirty = 1;
          printf("W:%s\r\n", g_state.wave_type==0?"Sin":g_state.wave_type==1?"Tri":"Sqr");
      }

      {
          bool chg = false;
          uint8_t m = g_state.selected_ch == TARGET_CH_BOTH ? 3
                    : g_state.selected_ch == TARGET_CH_A ? 1 : 2;
          if (m & 1) {
              if (df) { AdjustFrequency(&g_state.ch_a, df, g_state.digit_idx[0]); g_state.flags.freq_dirty = 1; chg = true; }
              if (da) { AdjustAmplitude(&g_state.ch_a, da, g_state.digit_idx[1]);  g_state.flags.amp_dirty = 1;  chg = true; }
              if (dp) { AdjustPhase(&g_state.ch_a, dp, g_state.digit_idx[2]);     g_state.flags.phase_dirty = 1; chg = true; }
          }
          if (m & 2) {
              if (df) { AdjustFrequency(&g_state.ch_b, df, g_state.digit_idx[0]); g_state.flags.freq_dirty = 1; chg = true; }
              if (da) { AdjustAmplitude(&g_state.ch_b, da, g_state.digit_idx[1]);  g_state.flags.amp_dirty = 1;  chg = true; }
              if (dp) { AdjustPhase(&g_state.ch_b, dp, g_state.digit_idx[2]);     g_state.flags.phase_dirty = 1; chg = true; }
          }
          if (chg) {
              printf("A%lu %+ld %u.%u  B%lu %+ld %u.%u\r\n",
                     g_state.ch_a.frequency_hz, g_state.ch_a.amplitude_mv,
                     g_state.ch_a.phase_x10/10, g_state.ch_a.phase_x10%10,
                     g_state.ch_b.frequency_hz, g_state.ch_b.amplitude_mv,
                     g_state.ch_b.phase_x10/10, g_state.ch_b.phase_x10%10);
          }
      }

      UpdateDDS(&g_state);
      UpdateAmplitude(&g_state);

      HAL_Delay(10);
  }
  /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


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
