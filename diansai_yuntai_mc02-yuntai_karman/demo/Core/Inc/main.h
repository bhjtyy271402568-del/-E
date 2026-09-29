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
#include "stm32h7xx_hal.h"

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
#define GALVO_CHX_Pin GPIO_PIN_2
#define GALVO_CHX_GPIO_Port GPIOE
#define GALVO_CLK_Pin GPIO_PIN_3
#define GALVO_CLK_GPIO_Port GPIOE
#define GALVO_SYN_Pin GPIO_PIN_4
#define GALVO_SYN_GPIO_Port GPIOE
#define GALVO_CHY_Pin GPIO_PIN_5
#define GALVO_CHY_GPIO_Port GPIOE
#define ADC1_CH4_VBAT_Pin GPIO_PIN_4
#define ADC1_CH4_VBAT_GPIO_Port GPIOA
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOC
#define CS1_GYRO_Pin GPIO_PIN_5
#define CS1_GYRO_GPIO_Port GPIOC
#define INT1_ACCEL_Pin GPIO_PIN_0
#define INT1_ACCEL_GPIO_Port GPIOB
#define INT1_GYRO_Pin GPIO_PIN_1
#define INT1_GYRO_GPIO_Port GPIOB
#define USER_KEY_Pin GPIO_PIN_15
#define USER_KEY_GPIO_Port GPIOA
#define LASER_WARN_IN1_Pin GPIO_PIN_10
#define LASER_WARN_IN1_GPIO_Port GPIOC
#define LASER_WARN_IN2_Pin GPIO_PIN_11
#define LASER_WARN_IN2_GPIO_Port GPIOC
#define LASER_IP0_Pin GPIO_PIN_0
#define LASER_IP0_GPIO_Port GPIOD
#define LASER_IP1_Pin GPIO_PIN_1
#define LASER_IP1_GPIO_Port GPIOD
#define LASER_IP2_Pin GPIO_PIN_2
#define LASER_IP2_GPIO_Port GPIOD
#define LASER_IP3_Pin GPIO_PIN_3
#define LASER_IP3_GPIO_Port GPIOD
#define LASER_IP4_Pin GPIO_PIN_4
#define LASER_IP4_GPIO_Port GPIOD
#define LASER_IP5_Pin GPIO_PIN_5
#define LASER_IP5_GPIO_Port GPIOD
#define LASER_IP6_Pin GPIO_PIN_6
#define LASER_IP6_GPIO_Port GPIOD
#define LASER_IP7_Pin GPIO_PIN_7
#define LASER_IP7_GPIO_Port GPIOD
#define LASER_MO_Pin GPIO_PIN_4
#define LASER_MO_GPIO_Port GPIOB
#define LASER_PA_Pin GPIO_PIN_5
#define LASER_PA_GPIO_Port GPIOB
#define LASER_PWM_Pin GPIO_PIN_6
#define LASER_PWM_GPIO_Port GPIOB
#define LASER_LATCH_Pin GPIO_PIN_7
#define LASER_LATCH_GPIO_Port GPIOB
#define LASER_RED_Pin GPIO_PIN_8
#define LASER_RED_GPIO_Port GPIOB
#define LASER_ESTOP_Pin GPIO_PIN_9
#define LASER_ESTOP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
