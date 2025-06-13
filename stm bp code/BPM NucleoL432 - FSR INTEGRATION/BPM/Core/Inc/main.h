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
#include "stm32l4xx_hal.h"

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
#define pump_Pin GPIO_PIN_0
#define pump_GPIO_Port GPIOB
#define valve_Pin GPIO_PIN_1
#define valve_GPIO_Port GPIOB
#define SIGNAL_Pin GPIO_PIN_11
#define SIGNAL_GPIO_Port GPIOA
#define A_Pin GPIO_PIN_12
#define A_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_3
#define S2_GPIO_Port GPIOB
#define B_Pin GPIO_PIN_4
#define B_GPIO_Port GPIOB
#define Power_Pin GPIO_PIN_5
#define Power_GPIO_Port GPIOB
#define S1_Pin GPIO_PIN_6
#define S1_GPIO_Port GPIOB
#define S0_Pin GPIO_PIN_7
#define S0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
