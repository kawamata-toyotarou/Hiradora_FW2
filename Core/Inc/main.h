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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_SW_Pin GPIO_PIN_13
#define USER_SW_GPIO_Port GPIOC
#define USER_SW_EXTI_IRQn EXTI15_10_IRQn
#define k_Pin GPIO_PIN_0
#define k_GPIO_Port GPIOA
#define WAKE_Pin GPIO_PIN_7
#define WAKE_GPIO_Port GPIOE
#define PWM_UL_Pin GPIO_PIN_8
#define PWM_UL_GPIO_Port GPIOE
#define PWM_UH_Pin GPIO_PIN_9
#define PWM_UH_GPIO_Port GPIOE
#define PWM_VL_Pin GPIO_PIN_10
#define PWM_VL_GPIO_Port GPIOE
#define PWM_VH_Pin GPIO_PIN_11
#define PWM_VH_GPIO_Port GPIOE
#define PWM_WL_Pin GPIO_PIN_12
#define PWM_WL_GPIO_Port GPIOE
#define PWM_VHE13_Pin GPIO_PIN_13
#define PWM_VHE13_GPIO_Port GPIOE
#define SPI1_SS_Pin GPIO_PIN_15
#define SPI1_SS_GPIO_Port GPIOA
#define PWM_LED_Pin GPIO_PIN_6
#define PWM_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
