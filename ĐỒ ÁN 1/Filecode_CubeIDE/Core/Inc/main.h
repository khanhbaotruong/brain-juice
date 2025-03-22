/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define CS_Pin GPIO_PIN_0
#define CS_GPIO_Port GPIOA
#define TANG_Pin GPIO_PIN_3
#define TANG_GPIO_Port GPIOA
#define TANG_EXTI_IRQn EXTI3_IRQn
#define GIAM_Pin GPIO_PIN_4
#define GIAM_GPIO_Port GPIOA
#define GIAM_EXTI_IRQn EXTI4_IRQn
#define zero_coss_Pin GPIO_PIN_0
#define zero_coss_GPIO_Port GPIOB
#define zero_coss_EXTI_IRQn EXTI0_IRQn
#define Tuanap_Pin GPIO_PIN_1
#define Tuanap_GPIO_Port GPIOB
#define OKE_Pin GPIO_PIN_10
#define OKE_GPIO_Port GPIOB
#define MODE_Pin GPIO_PIN_11
#define MODE_GPIO_Port GPIOB
#define firing_pin_Pin GPIO_PIN_8
#define firing_pin_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
