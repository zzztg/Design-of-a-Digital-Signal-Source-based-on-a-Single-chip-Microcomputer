/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_DAC_Pin GPIO_PIN_10
#define CS_DAC_GPIO_Port GPIOB
#define CS_B_Pin GPIO_PIN_11
#define CS_B_GPIO_Port GPIOB
#define CS_A_Pin GPIO_PIN_12
#define CS_A_GPIO_Port GPIOB
#define SYNC_RESET_Pin GPIO_PIN_14
#define SYNC_RESET_GPIO_Port GPIOB
#define PC_A_Pin GPIO_PIN_8
#define PC_A_GPIO_Port GPIOC
#define PC_B_Pin GPIO_PIN_9
#define PC_B_GPIO_Port GPIOC
#define WAVE_SEL_A_Pin GPIO_PIN_10
#define WAVE_SEL_A_GPIO_Port GPIOC
#define WAVE_SEL_B_Pin GPIO_PIN_11
#define WAVE_SEL_B_GPIO_Port GPIOC
#define KEY_ENC1_Pin GPIO_PIN_0
#define KEY_ENC1_GPIO_Port GPIOD
#define KEY_ENC2_Pin GPIO_PIN_1
#define KEY_ENC2_GPIO_Port GPIOD
#define KEY_ENC3_Pin GPIO_PIN_2
#define KEY_ENC3_GPIO_Port GPIOD
#define KEY_SW1_Pin GPIO_PIN_3
#define KEY_SW1_GPIO_Port GPIOD
#define KEY_SW2_Pin GPIO_PIN_4
#define KEY_SW2_GPIO_Port GPIOD
#define FC_A_Pin GPIO_PIN_8
#define FC_A_GPIO_Port GPIOB
#define FC_B_Pin GPIO_PIN_9
#define FC_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
