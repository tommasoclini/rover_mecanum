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
#define TIM1_PRESCALER 10-1
#define TIM1_PERIOD 8400
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define FL_MOTOR_A_Pin GPIO_PIN_0
#define FL_MOTOR_A_GPIO_Port GPIOC
#define FL_MOTOR_B_Pin GPIO_PIN_1
#define FL_MOTOR_B_GPIO_Port GPIOC
#define FR_MOTOR_A_Pin GPIO_PIN_2
#define FR_MOTOR_A_GPIO_Port GPIOC
#define FR_MOTOR_B_Pin GPIO_PIN_3
#define FR_MOTOR_B_GPIO_Port GPIOC
#define BR_ENCODER__Pin GPIO_PIN_0
#define BR_ENCODER__GPIO_Port GPIOA
#define BR_ENCODER_B_Pin GPIO_PIN_1
#define BR_ENCODER_B_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define FL_ENCODER_A_Pin GPIO_PIN_5
#define FL_ENCODER_A_GPIO_Port GPIOA
#define FR_ENCODER_A_Pin GPIO_PIN_6
#define FR_ENCODER_A_GPIO_Port GPIOA
#define FR_ENCODER_B_Pin GPIO_PIN_7
#define FR_ENCODER_B_GPIO_Port GPIOA
#define BL_MOTOR_A_Pin GPIO_PIN_5
#define BL_MOTOR_A_GPIO_Port GPIOC
#define ROVER_TX_Pin GPIO_PIN_6
#define ROVER_TX_GPIO_Port GPIOC
#define ROVER_RX_Pin GPIO_PIN_7
#define ROVER_RX_GPIO_Port GPIOC
#define FL_MOTOR_EN_Pin GPIO_PIN_8
#define FL_MOTOR_EN_GPIO_Port GPIOA
#define FR_MOTOR_EN_Pin GPIO_PIN_9
#define FR_MOTOR_EN_GPIO_Port GPIOA
#define BL_MOTOR_EN_Pin GPIO_PIN_10
#define BL_MOTOR_EN_GPIO_Port GPIOA
#define BR_MOTOR_EN_Pin GPIO_PIN_11
#define BR_MOTOR_EN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define BL_MOOR_B_Pin GPIO_PIN_10
#define BL_MOOR_B_GPIO_Port GPIOC
#define BR_MOTRO_A_Pin GPIO_PIN_11
#define BR_MOTRO_A_GPIO_Port GPIOC
#define BR_MOTOR_B_Pin GPIO_PIN_12
#define BR_MOTOR_B_GPIO_Port GPIOC
#define FL_ENCODER_B_Pin GPIO_PIN_3
#define FL_ENCODER_B_GPIO_Port GPIOB
#define BL_ENCODER_A_Pin GPIO_PIN_6
#define BL_ENCODER_A_GPIO_Port GPIOB
#define BL_ENCODER_B_Pin GPIO_PIN_7
#define BL_ENCODER_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */