/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define USART2_TX_Motor_Pin GPIO_PIN_2
#define USART2_TX_Motor_GPIO_Port GPIOA
#define USART2_RX_Motor_Pin GPIO_PIN_3
#define USART2_RX_Motor_GPIO_Port GPIOA
#define ADC_Pot_Pin GPIO_PIN_5
#define ADC_Pot_GPIO_Port GPIOA
#define USART1_RX_Arduino_Pin GPIO_PIN_9
#define USART1_RX_Arduino_GPIO_Port GPIOA
#define USART1_RX_ArduinoA10_Pin GPIO_PIN_10
#define USART1_RX_ArduinoA10_GPIO_Port GPIOA
#define USART6_TX_PC_Pin GPIO_PIN_11
#define USART6_TX_PC_GPIO_Port GPIOA
#define USART6_RX_PC_Pin GPIO_PIN_12
#define USART6_RX_PC_GPIO_Port GPIOA
#define MicroSwitch_Up_Pin GPIO_PIN_8
#define MicroSwitch_Up_GPIO_Port GPIOB
#define MicroSwitch_Up_EXTI_IRQn EXTI9_5_IRQn
#define MicroSwitch_Down_Pin GPIO_PIN_9
#define MicroSwitch_Down_GPIO_Port GPIOB
#define MicroSwitch_Down_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
