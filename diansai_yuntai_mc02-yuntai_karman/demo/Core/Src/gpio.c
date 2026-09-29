/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PA8   ------> RCC_MCO_1
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GALVO_CHX_Pin|GALVO_CLK_Pin|GALVO_SYN_Pin|GALVO_CHY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS1_ACCEL_Pin|CS1_GYRO_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LASER_IP0_Pin|LASER_IP1_Pin|LASER_IP2_Pin|LASER_IP3_Pin
                          |LASER_IP4_Pin|LASER_IP5_Pin|LASER_IP6_Pin|LASER_IP7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LASER_MO_Pin|LASER_PA_Pin|LASER_LATCH_Pin|LASER_RED_Pin
                          |LASER_ESTOP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GALVO_CHX_Pin GALVO_CLK_Pin GALVO_SYN_Pin GALVO_CHY_Pin */
  GPIO_InitStruct.Pin = GALVO_CHX_Pin|GALVO_CLK_Pin|GALVO_SYN_Pin|GALVO_CHY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_ACCEL_Pin CS1_GYRO_Pin */
  GPIO_InitStruct.Pin = CS1_ACCEL_Pin|CS1_GYRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_ACCEL_Pin INT1_GYRO_Pin */
  GPIO_InitStruct.Pin = INT1_ACCEL_Pin|INT1_GYRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_KEY_Pin */
  GPIO_InitStruct.Pin = USER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LASER_WARN_IN1_Pin */
  GPIO_InitStruct.Pin = LASER_WARN_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LASER_WARN_IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LASER_WARN_IN2_Pin */
  GPIO_InitStruct.Pin = LASER_WARN_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LASER_WARN_IN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LASER_IP0_Pin LASER_IP1_Pin LASER_IP2_Pin LASER_IP3_Pin
                           LASER_IP4_Pin LASER_IP5_Pin LASER_IP6_Pin LASER_IP7_Pin */
  GPIO_InitStruct.Pin = LASER_IP0_Pin|LASER_IP1_Pin|LASER_IP2_Pin|LASER_IP3_Pin
                          |LASER_IP4_Pin|LASER_IP5_Pin|LASER_IP6_Pin|LASER_IP7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LASER_MO_Pin LASER_PA_Pin LASER_LATCH_Pin LASER_RED_Pin
                           LASER_ESTOP_Pin */
  GPIO_InitStruct.Pin = LASER_MO_Pin|LASER_PA_Pin|LASER_LATCH_Pin|LASER_RED_Pin
                          |LASER_ESTOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
