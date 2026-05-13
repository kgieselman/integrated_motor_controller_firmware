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

/** Configure pins
     PH0-OSC_IN(PH0)   ------> RCC_OSC_IN
     PA13(JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14(JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DEBUG_BUZZER_Pin|DEBUG_LED_0_Pin|DEBUG_LED_1_Pin|DEBUG_LED_2_Pin
                          |TP11_PC8_Pin|IMU_SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_0_MODE_Pin|MOTOR_0_ENABLE_Pin|EEPROM_WRITE_PROTECT_Pin|TP10_PA10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_1_ENABLE_Pin|MOTOR_1_MODE_Pin|TP12_PB12_Pin|TP13_PB13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EXTI13_DEBUG_BUTTON_Pin EXTI3_MOTOR_0_FAULT_Pin EXTI7_MOTOR_1_FAULT_Pin */
  GPIO_InitStruct.Pin = EXTI13_DEBUG_BUTTON_Pin|EXTI3_MOTOR_0_FAULT_Pin|EXTI7_MOTOR_1_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_BUZZER_Pin DEBUG_LED_0_Pin DEBUG_LED_1_Pin DEBUG_LED_2_Pin
                           TP11_PC8_Pin IMU_SPI_CS_Pin */
  GPIO_InitStruct.Pin = DEBUG_BUZZER_Pin|DEBUG_LED_0_Pin|DEBUG_LED_1_Pin|DEBUG_LED_2_Pin
                          |TP11_PC8_Pin|IMU_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_0_MODE_Pin MOTOR_0_ENABLE_Pin EEPROM_WRITE_PROTECT_Pin TP10_PA10_Pin */
  GPIO_InitStruct.Pin = MOTOR_0_MODE_Pin|MOTOR_0_ENABLE_Pin|EEPROM_WRITE_PROTECT_Pin|TP10_PA10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_1_ENABLE_Pin MOTOR_1_MODE_Pin TP12_PB12_Pin TP13_PB13_Pin */
  GPIO_InitStruct.Pin = MOTOR_1_ENABLE_Pin|MOTOR_1_MODE_Pin|TP12_PB12_Pin|TP13_PB13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_DETECTED_Pin */
  GPIO_InitStruct.Pin = USB_DETECTED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_DETECTED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI2_IMU_INT_1_Pin */
  GPIO_InitStruct.Pin = EXTI2_IMU_INT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI2_IMU_INT_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI8_IMU_INT2_Pin */
  GPIO_InitStruct.Pin = EXTI8_IMU_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI8_IMU_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI7_IRQn);

  HAL_NVIC_SetPriority(EXTI8_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI8_IRQn);

  HAL_NVIC_SetPriority(EXTI13_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI13_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
