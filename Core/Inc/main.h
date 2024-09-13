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
#define UART_STATE_LED_Pin GPIO_PIN_13
#define UART_STATE_LED_GPIO_Port GPIOC
#define Speed_1_Pin GPIO_PIN_1
#define Speed_1_GPIO_Port GPIOA
#define Speed_2_Pin GPIO_PIN_2
#define Speed_2_GPIO_Port GPIOA
#define Speed_3_Pin GPIO_PIN_3
#define Speed_3_GPIO_Port GPIOA
#define Speed_4_Pin GPIO_PIN_4
#define Speed_4_GPIO_Port GPIOA
#define Speed_5_Pin GPIO_PIN_5
#define Speed_5_GPIO_Port GPIOA
#define Semi_Pin GPIO_PIN_6
#define Semi_GPIO_Port GPIOA
#define Side_Sensing_Pin GPIO_PIN_0
#define Side_Sensing_GPIO_Port GPIOB
#define Shearing_Pin GPIO_PIN_1
#define Shearing_GPIO_Port GPIOB
#define Side_Trimmer_Pin GPIO_PIN_10
#define Side_Trimmer_GPIO_Port GPIOB
#define All_Wheel_Pin GPIO_PIN_11
#define All_Wheel_GPIO_Port GPIOB
#define Crab_Pin GPIO_PIN_12
#define Crab_GPIO_Port GPIOB
#define Zero_Turn_Pin GPIO_PIN_13
#define Zero_Turn_GPIO_Port GPIOB
#define Width_In_Pin GPIO_PIN_14
#define Width_In_GPIO_Port GPIOB
#define Width_Out_Pin GPIO_PIN_15
#define Width_Out_GPIO_Port GPIOB
#define Lever_Fwd_Pin GPIO_PIN_11
#define Lever_Fwd_GPIO_Port GPIOA
#define Lever_Right_Pin GPIO_PIN_12
#define Lever_Right_GPIO_Port GPIOA
#define Lever_Rev_Pin GPIO_PIN_15
#define Lever_Rev_GPIO_Port GPIOA
#define Lever_Left_Pin GPIO_PIN_3
#define Lever_Left_GPIO_Port GPIOB
#define UART_OUT2_Pin GPIO_PIN_4
#define UART_OUT2_GPIO_Port GPIOB
#define UART_OUT1_Pin GPIO_PIN_5
#define UART_OUT1_GPIO_Port GPIOB
#define LED_MAIN_OUT2_Pin GPIO_PIN_6
#define LED_MAIN_OUT2_GPIO_Port GPIOB
#define LED_MAIN_OUT1_Pin GPIO_PIN_7
#define LED_MAIN_OUT1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
