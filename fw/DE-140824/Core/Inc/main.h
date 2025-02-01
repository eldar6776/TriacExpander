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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "LuxNET.h"
#include "TinyFrame.h"
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
void RS485_Tick(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R3_Pin GPIO_PIN_13
#define LED_R3_GPIO_Port GPIOC
#define ADDRESS5_Pin GPIO_PIN_14
#define ADDRESS5_GPIO_Port GPIOC
#define LED_R2_Pin GPIO_PIN_15
#define LED_R2_GPIO_Port GPIOC
#define LED_R1_Pin GPIO_PIN_0
#define LED_R1_GPIO_Port GPIOF
#define LED_R0_Pin GPIO_PIN_1
#define LED_R0_GPIO_Port GPIOF
#define CURRENT1_Pin GPIO_PIN_0
#define CURRENT1_GPIO_Port GPIOA
#define CURRENT2_Pin GPIO_PIN_4
#define CURRENT2_GPIO_Port GPIOA
#define ADDRESS4_Pin GPIO_PIN_5
#define ADDRESS4_GPIO_Port GPIOA
#define ADDRESS3_Pin GPIO_PIN_6
#define ADDRESS3_GPIO_Port GPIOA
#define ADDRESS2_Pin GPIO_PIN_7
#define ADDRESS2_GPIO_Port GPIOA
#define ADDRESS1_Pin GPIO_PIN_0
#define ADDRESS1_GPIO_Port GPIOB
#define ADDRESS0_Pin GPIO_PIN_1
#define ADDRESS0_GPIO_Port GPIOB
#define THY_GATE15_Pin GPIO_PIN_2
#define THY_GATE15_GPIO_Port GPIOB
#define THY_GATE14_Pin GPIO_PIN_10
#define THY_GATE14_GPIO_Port GPIOB
#define THY_GATE13_Pin GPIO_PIN_11
#define THY_GATE13_GPIO_Port GPIOB
#define THY_GATE12_Pin GPIO_PIN_12
#define THY_GATE12_GPIO_Port GPIOB
#define THY_GATE11_Pin GPIO_PIN_13
#define THY_GATE11_GPIO_Port GPIOB
#define THY_GATE10_Pin GPIO_PIN_14
#define THY_GATE10_GPIO_Port GPIOB
#define THY_GATE9_Pin GPIO_PIN_15
#define THY_GATE9_GPIO_Port GPIOB
#define THY_GATE8_Pin GPIO_PIN_8
#define THY_GATE8_GPIO_Port GPIOA
#define THY_GATE7_Pin GPIO_PIN_11
#define THY_GATE7_GPIO_Port GPIOA
#define THY_GATE6_Pin GPIO_PIN_12
#define THY_GATE6_GPIO_Port GPIOA
#define THY_GATE5_Pin GPIO_PIN_6
#define THY_GATE5_GPIO_Port GPIOF
#define THY_GATE4_Pin GPIO_PIN_7
#define THY_GATE4_GPIO_Port GPIOF
#define THY_GATE3_Pin GPIO_PIN_15
#define THY_GATE3_GPIO_Port GPIOA
#define THY_GATE2_Pin GPIO_PIN_3
#define THY_GATE2_GPIO_Port GPIOB
#define THY_GATE1_Pin GPIO_PIN_4
#define THY_GATE1_GPIO_Port GPIOB
#define THY_GATE0_Pin GPIO_PIN_5
#define THY_GATE0_GPIO_Port GPIOB
#define LED_C3_Pin GPIO_PIN_6
#define LED_C3_GPIO_Port GPIOB
#define LED_C2_Pin GPIO_PIN_7
#define LED_C2_GPIO_Port GPIOB
#define LED_C1_Pin GPIO_PIN_8
#define LED_C1_GPIO_Port GPIOB
#define LED_C0_Pin GPIO_PIN_9
#define LED_C0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
