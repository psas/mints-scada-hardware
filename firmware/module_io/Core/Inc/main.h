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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
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
#define CONFIG0_Pin GPIO_PIN_0
#define CONFIG0_GPIO_Port GPIOA
#define CONFIG1_Pin GPIO_PIN_1
#define CONFIG1_GPIO_Port GPIOA
#define ADDR4_Pin GPIO_PIN_4
#define ADDR4_GPIO_Port GPIOA
#define ADDR1_Pin GPIO_PIN_5
#define ADDR1_GPIO_Port GPIOA
#define ADDR8_Pin GPIO_PIN_6
#define ADDR8_GPIO_Port GPIOA
#define ADDR2_Pin GPIO_PIN_7
#define ADDR2_GPIO_Port GPIOA
#define OUT0_Pin GPIO_PIN_0
#define OUT0_GPIO_Port GPIOB
#define OUT1_Pin GPIO_PIN_1
#define OUT1_GPIO_Port GPIOB
#define OUT2_Pin GPIO_PIN_2
#define OUT2_GPIO_Port GPIOB
#define CTRL_Pin GPIO_PIN_12
#define CTRL_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOA
#define OUT3_Pin GPIO_PIN_3
#define OUT3_GPIO_Port GPIOB
#define OUT4_Pin GPIO_PIN_4
#define OUT4_GPIO_Port GPIOB
#define OUT5_Pin GPIO_PIN_5
#define OUT5_GPIO_Port GPIOB
#define OUT6_Pin GPIO_PIN_6
#define OUT6_GPIO_Port GPIOB
#define OUT7_Pin GPIO_PIN_7
#define OUT7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
