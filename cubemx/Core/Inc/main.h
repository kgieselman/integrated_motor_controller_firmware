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
#include "stm32h5xx_hal.h"

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
#define EXTI13_DEBUG_BUTTON_Pin GPIO_PIN_13
#define EXTI13_DEBUG_BUTTON_GPIO_Port GPIOC
#define EXTI13_DEBUG_BUTTON_EXTI_IRQn EXTI13_IRQn
#define DEBUG_BUZZER_Pin GPIO_PIN_14
#define DEBUG_BUZZER_GPIO_Port GPIOC
#define DEBUG_LED_0_Pin GPIO_PIN_15
#define DEBUG_LED_0_GPIO_Port GPIOC
#define DEBUG_LED_1_Pin GPIO_PIN_0
#define DEBUG_LED_1_GPIO_Port GPIOC
#define DEBUG_LED_2_Pin GPIO_PIN_1
#define DEBUG_LED_2_GPIO_Port GPIOC
#define TIM4_CH4_ENCODER_0_Pin GPIO_PIN_2
#define TIM4_CH4_ENCODER_0_GPIO_Port GPIOC
#define EXTI3_MOTOR_0_FAULT_Pin GPIO_PIN_3
#define EXTI3_MOTOR_0_FAULT_GPIO_Port GPIOC
#define EXTI3_MOTOR_0_FAULT_EXTI_IRQn EXTI3_IRQn
#define TIM2_CH1_SERVO_0_Pin GPIO_PIN_0
#define TIM2_CH1_SERVO_0_GPIO_Port GPIOA
#define TIM2_CH2_SERVO_1_Pin GPIO_PIN_1
#define TIM2_CH2_SERVO_1_GPIO_Port GPIOA
#define TIM2_CH3_SERVO_2_Pin GPIO_PIN_2
#define TIM2_CH3_SERVO_2_GPIO_Port GPIOA
#define MOTOR_0_MODE_Pin GPIO_PIN_3
#define MOTOR_0_MODE_GPIO_Port GPIOA
#define ADC1_INP18_MOTOR_0_IPROPI_Pin GPIO_PIN_4
#define ADC1_INP18_MOTOR_0_IPROPI_GPIO_Port GPIOA
#define MOTOR_0_ENABLE_Pin GPIO_PIN_5
#define MOTOR_0_ENABLE_GPIO_Port GPIOA
#define TIM3_CH1_MOTOR_0_IN_2_Pin GPIO_PIN_6
#define TIM3_CH1_MOTOR_0_IN_2_GPIO_Port GPIOA
#define TIM3_CH2_MOTOR_0_IN_1_Pin GPIO_PIN_7
#define TIM3_CH2_MOTOR_0_IN_1_GPIO_Port GPIOA
#define ADC1_INP4_VBATT_Pin GPIO_PIN_4
#define ADC1_INP4_VBATT_GPIO_Port GPIOC
#define ADC1_INP8_MOTOR_1_IPROPI_Pin GPIO_PIN_5
#define ADC1_INP8_MOTOR_1_IPROPI_GPIO_Port GPIOC
#define TIM3_CH3_MOTOR_1_IN_2_Pin GPIO_PIN_0
#define TIM3_CH3_MOTOR_1_IN_2_GPIO_Port GPIOB
#define TIM3_CH4_MOTOR_1_IN_1_Pin GPIO_PIN_1
#define TIM3_CH4_MOTOR_1_IN_1_GPIO_Port GPIOB
#define MOTOR_1_ENABLE_Pin GPIO_PIN_2
#define MOTOR_1_ENABLE_GPIO_Port GPIOB
#define MOTOR_1_MODE_Pin GPIO_PIN_10
#define MOTOR_1_MODE_GPIO_Port GPIOB
#define TP12_PB12_Pin GPIO_PIN_12
#define TP12_PB12_GPIO_Port GPIOB
#define TP13_PB13_Pin GPIO_PIN_13
#define TP13_PB13_GPIO_Port GPIOB
#define USART1_MCU_TO_GROWTH_Pin GPIO_PIN_14
#define USART1_MCU_TO_GROWTH_GPIO_Port GPIOB
#define USART1_GROWTH_TO_MCU_Pin GPIO_PIN_15
#define USART1_GROWTH_TO_MCU_GPIO_Port GPIOB
#define TIM8_CH1_ENCODER_1_Pin GPIO_PIN_6
#define TIM8_CH1_ENCODER_1_GPIO_Port GPIOC
#define EXTI7_MOTOR_1_FAULT_Pin GPIO_PIN_7
#define EXTI7_MOTOR_1_FAULT_GPIO_Port GPIOC
#define EXTI7_MOTOR_1_FAULT_EXTI_IRQn EXTI7_IRQn
#define TP11_PC8_Pin GPIO_PIN_8
#define TP11_PC8_GPIO_Port GPIOC
#define GROWTH_I2C3_SDA_Pin GPIO_PIN_9
#define GROWTH_I2C3_SDA_GPIO_Port GPIOC
#define GROWTH_I2C3_SCL_Pin GPIO_PIN_8
#define GROWTH_I2C3_SCL_GPIO_Port GPIOA
#define EEPROM_WRITE_PROTECT_Pin GPIO_PIN_9
#define EEPROM_WRITE_PROTECT_GPIO_Port GPIOA
#define TP10_PA10_Pin GPIO_PIN_10
#define TP10_PA10_GPIO_Port GPIOA
#define USB_DETECTED_Pin GPIO_PIN_15
#define USB_DETECTED_GPIO_Port GPIOA
#define UART4_MCU_TO_CRSF_Pin GPIO_PIN_10
#define UART4_MCU_TO_CRSF_GPIO_Port GPIOC
#define UART4_CRSF_TO_MCU_Pin GPIO_PIN_11
#define UART4_CRSF_TO_MCU_GPIO_Port GPIOC
#define IMU_SPI_CS_Pin GPIO_PIN_12
#define IMU_SPI_CS_GPIO_Port GPIOC
#define EXTI2_IMU_INT_1_Pin GPIO_PIN_2
#define EXTI2_IMU_INT_1_GPIO_Port GPIOD
#define EXTI2_IMU_INT_1_EXTI_IRQn EXTI2_IRQn
#define IMU_SPI1_SCK_Pin GPIO_PIN_3
#define IMU_SPI1_SCK_GPIO_Port GPIOB
#define IMU_SPI1_MISO_Pin GPIO_PIN_4
#define IMU_SPI1_MISO_GPIO_Port GPIOB
#define IMU_SPI1_MOSI_Pin GPIO_PIN_5
#define IMU_SPI1_MOSI_GPIO_Port GPIOB
#define EEPROM_I2C1_SCL_Pin GPIO_PIN_6
#define EEPROM_I2C1_SCL_GPIO_Port GPIOB
#define EEPROM_I2C1_SDA_Pin GPIO_PIN_7
#define EEPROM_I2C1_SDA_GPIO_Port GPIOB
#define EXTI8_IMU_INT2_Pin GPIO_PIN_8
#define EXTI8_IMU_INT2_GPIO_Port GPIOB
#define EXTI8_IMU_INT2_EXTI_IRQn EXTI8_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
