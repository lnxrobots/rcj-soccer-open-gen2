/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M1_TIM_Pin GPIO_PIN_0
#define M1_TIM_GPIO_Port GPIOC
#define M1_CWE_Pin GPIO_PIN_1
#define M1_CWE_GPIO_Port GPIOC
#define M1_CCWE_Pin GPIO_PIN_2
#define M1_CCWE_GPIO_Port GPIOC
#define GPIO_LED_Pin GPIO_PIN_4
#define GPIO_LED_GPIO_Port GPIOA
#define M2_CWE_Pin GPIO_PIN_4
#define M2_CWE_GPIO_Port GPIOC
#define M2_CCWE_Pin GPIO_PIN_5
#define M2_CCWE_GPIO_Port GPIOC
#define M2_TIM_Pin GPIO_PIN_2
#define M2_TIM_GPIO_Port GPIOB
#define M3_CCWE_Pin GPIO_PIN_7
#define M3_CCWE_GPIO_Port GPIOC
#define M3_CWE_Pin GPIO_PIN_8
#define M3_CWE_GPIO_Port GPIOC
#define M4_CCWE_Pin GPIO_PIN_10
#define M4_CCWE_GPIO_Port GPIOA
#define M4_CWE_Pin GPIO_PIN_12
#define M4_CWE_GPIO_Port GPIOA
#define KICK_Pin GPIO_PIN_15
#define KICK_GPIO_Port GPIOA
#define MD_TIM_Pin GPIO_PIN_4
#define MD_TIM_GPIO_Port GPIOB
#define MD_CWE_Pin GPIO_PIN_5
#define MD_CWE_GPIO_Port GPIOB
#define MD_CCWE_Pin GPIO_PIN_6
#define MD_CCWE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
