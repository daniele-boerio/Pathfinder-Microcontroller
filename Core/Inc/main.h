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
void Control_Motors(int, int);
void move_backward(unsigned long, unsigned long);
void move_forward(unsigned long, unsigned long);
void move_left(unsigned long, unsigned long);
void move_right(unsigned long, unsigned long);
void Process_Received_String(char*);
void checkStates(void);
void moveToGoal(Point);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENABLE_RIGHT_Pin GPIO_PIN_2
#define ENABLE_RIGHT_GPIO_Port GPIOE
#define ENABLE_LEFT_Pin GPIO_PIN_3
#define ENABLE_LEFT_GPIO_Port GPIOE
#define DIR_RIGHT_Pin GPIO_PIN_5
#define DIR_RIGHT_GPIO_Port GPIOD
#define DIR_LEFT_Pin GPIO_PIN_6
#define DIR_LEFT_GPIO_Port GPIOD
#define STEP_RIGHT_Pin GPIO_PIN_7
#define STEP_RIGHT_GPIO_Port GPIOD
#define STEP_LEFT_Pin GPIO_PIN_3
#define STEP_LEFT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define RX_BUFFER_SIZE 100
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
