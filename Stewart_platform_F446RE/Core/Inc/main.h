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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define joy_rot_z_Pin GPIO_PIN_0
#define joy_rot_z_GPIO_Port GPIOC
#define joy_x_Pin GPIO_PIN_0
#define joy_x_GPIO_Port GPIOA
#define joy_y_Pin GPIO_PIN_1
#define joy_y_GPIO_Port GPIOA
#define joy_z_Pin GPIO_PIN_4
#define joy_z_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define joy_rot_x_Pin GPIO_PIN_0
#define joy_rot_x_GPIO_Port GPIOB
#define joy_rot_y_Pin GPIO_PIN_1
#define joy_rot_y_GPIO_Port GPIOB
#define Home_Button_Pin GPIO_PIN_8
#define Home_Button_GPIO_Port GPIOC
#define suction_enable_btn_Pin GPIO_PIN_9
#define suction_enable_btn_GPIO_Port GPIOC
#define CE_Pin GPIO_PIN_4
#define CE_GPIO_Port GPIOB
#define CSN_Pin GPIO_PIN_5
#define CSN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
