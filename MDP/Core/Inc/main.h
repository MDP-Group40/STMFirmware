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
#include <math.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define SERVO_STRAIGHT 	150
#define SERVO_LEFT 		100
#define SERVO_RIGHT 	215
#define RIGHT_TURN		3470
#define LEFT_TURN 		1700
#define TARGET_LEFT   1000
#define TARGET_RIGHT  1200
extern I2C_HandleTypeDef hi2c1;

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SCL_Pin GPIO_PIN_5
#define SCL_GPIO_Port GPIOE
#define SDA_Pin GPIO_PIN_6
#define SDA_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_2
#define AIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_3
#define AIN1_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOA
#define RESET_Pin GPIO_PIN_7
#define RESET_GPIO_Port GPIOE
#define DC_Pin GPIO_PIN_8
#define DC_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOE
#define IR_LEFT_Pin GPIO_PIN_12
#define IR_LEFT_GPIO_Port GPIOE
#define IR_RIGHT_Pin GPIO_PIN_13
#define IR_RIGHT_GPIO_Port GPIOE
#define PWM_SERVO_Pin GPIO_PIN_14
#define PWM_SERVO_GPIO_Port GPIOE
#define ULTRA_ECHO_Pin GPIO_PIN_12
#define ULTRA_ECHO_GPIO_Port GPIOD
#define ULTRA_TRIG_Pin GPIO_PIN_13
#define ULTRA_TRIG_GPIO_Port GPIOD
#define PWMLEFT_Pin GPIO_PIN_6
#define PWMLEFT_GPIO_Port GPIOC
#define PWMRIGHT_Pin GPIO_PIN_7
#define PWMRIGHT_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define ICM20948_I2C					(&hi2c1)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
