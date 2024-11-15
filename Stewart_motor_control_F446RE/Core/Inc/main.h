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
#define C6_In1_Pin GPIO_PIN_2
#define C6_In1_GPIO_Port GPIOC
#define C6_In2_Pin GPIO_PIN_3
#define C6_In2_GPIO_Port GPIOC
#define C1_pot_Pin GPIO_PIN_0
#define C1_pot_GPIO_Port GPIOA
#define C2_pot_Pin GPIO_PIN_1
#define C2_pot_GPIO_Port GPIOA
#define C3_pot_Pin GPIO_PIN_2
#define C3_pot_GPIO_Port GPIOA
#define C4_pot_Pin GPIO_PIN_3
#define C4_pot_GPIO_Port GPIOA
#define C5_pot_Pin GPIO_PIN_4
#define C5_pot_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define C6_pot_Pin GPIO_PIN_6
#define C6_pot_GPIO_Port GPIOA
#define C5_In1_Pin GPIO_PIN_0
#define C5_In1_GPIO_Port GPIOB
#define C5_In2_Pin GPIO_PIN_1
#define C5_In2_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_2
#define CE_GPIO_Port GPIOB
#define CSN_Pin GPIO_PIN_10
#define CSN_GPIO_Port GPIOB
#define C4_In2_Pin GPIO_PIN_12
#define C4_In2_GPIO_Port GPIOB
#define C4_In1_Pin GPIO_PIN_13
#define C4_In1_GPIO_Port GPIOB
#define C3_In2_Pin GPIO_PIN_14
#define C3_In2_GPIO_Port GPIOB
#define C3_In1_Pin GPIO_PIN_15
#define C3_In1_GPIO_Port GPIOB
#define C2_In2_Pin GPIO_PIN_6
#define C2_In2_GPIO_Port GPIOC
#define C2_In1_Pin GPIO_PIN_7
#define C2_In1_GPIO_Port GPIOC
#define C1_In2_Pin GPIO_PIN_8
#define C1_In2_GPIO_Port GPIOC
#define C1_In1_Pin GPIO_PIN_9
#define C1_In1_GPIO_Port GPIOC
#define C1_PWM_Pin GPIO_PIN_8
#define C1_PWM_GPIO_Port GPIOA
#define C2_PWM_Pin GPIO_PIN_9
#define C2_PWM_GPIO_Port GPIOA
#define C3_PWM_Pin GPIO_PIN_10
#define C3_PWM_GPIO_Port GPIOA
#define C4_PWM_Pin GPIO_PIN_11
#define C4_PWM_GPIO_Port GPIOA
#define C5_PWM_Pin GPIO_PIN_15
#define C5_PWM_GPIO_Port GPIOA
#define suction_enable_Pin GPIO_PIN_2
#define suction_enable_GPIO_Port GPIOD
#define C6_PWM_Pin GPIO_PIN_9
#define C6_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
