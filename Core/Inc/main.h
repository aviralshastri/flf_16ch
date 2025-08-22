/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENCODERB1_Pin GPIO_PIN_0
#define ENCODERB1_GPIO_Port GPIOA
#define ENCODERB2_Pin GPIO_PIN_1
#define ENCODERB2_GPIO_Port GPIOA
#define ESC_Pin GPIO_PIN_2
#define ESC_GPIO_Port GPIOA
#define ENCODERA1_Pin GPIO_PIN_5
#define ENCODERA1_GPIO_Port GPIOA
#define OUT_Pin GPIO_PIN_7
#define OUT_GPIO_Port GPIOA
#define S3_Pin GPIO_PIN_0
#define S3_GPIO_Port GPIOB
#define S2_Pin GPIO_PIN_1
#define S2_GPIO_Port GPIOB
#define S1_Pin GPIO_PIN_2
#define S1_GPIO_Port GPIOB
#define S0_Pin GPIO_PIN_10
#define S0_GPIO_Port GPIOB
#define INA1_Pin GPIO_PIN_8
#define INA1_GPIO_Port GPIOA
#define INA2_Pin GPIO_PIN_9
#define INA2_GPIO_Port GPIOA
#define INB1_Pin GPIO_PIN_10
#define INB1_GPIO_Port GPIOA
#define INB2_Pin GPIO_PIN_11
#define INB2_GPIO_Port GPIOA
#define STBY_Pin GPIO_PIN_12
#define STBY_GPIO_Port GPIOA
#define ENCODERA2_Pin GPIO_PIN_3
#define ENCODERA2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
