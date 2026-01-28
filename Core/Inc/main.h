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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define FLAME_Pin GPIO_PIN_1
#define FLAME_GPIO_Port GPIOC
#define LED_LEFT_Pin GPIO_PIN_2
#define LED_LEFT_GPIO_Port GPIOC
#define LED_RIGHT_Pin GPIO_PIN_3
#define LED_RIGHT_GPIO_Port GPIOC
#define TEMPER_Pin GPIO_PIN_0
#define TEMPER_GPIO_Port GPIOA
#define LCD_RES_Pin GPIO_PIN_1
#define LCD_RES_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define ECHO2_Pin GPIO_PIN_4
#define ECHO2_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_6
#define LCD_DC_GPIO_Port GPIOA
#define ECHO1_Pin GPIO_PIN_0
#define ECHO1_GPIO_Port GPIOB
#define LBB_Pin GPIO_PIN_10
#define LBB_GPIO_Port GPIOB
#define ECHO3_Pin GPIO_PIN_12
#define ECHO3_GPIO_Port GPIOB
#define TRIG3_Pin GPIO_PIN_13
#define TRIG3_GPIO_Port GPIOB
#define ECHO4_Pin GPIO_PIN_14
#define ECHO4_GPIO_Port GPIOB
#define TRIG4_Pin GPIO_PIN_15
#define TRIG4_GPIO_Port GPIOB
#define C6_Pin GPIO_PIN_6
#define C6_GPIO_Port GPIOC
#define C6_EXTI_IRQn EXTI9_5_IRQn
#define TRIG1_Pin GPIO_PIN_7
#define TRIG1_GPIO_Port GPIOC
#define TRIG2_Pin GPIO_PIN_8
#define TRIG2_GPIO_Port GPIOA
#define LBF_Pin GPIO_PIN_9
#define LBF_GPIO_Port GPIOA
#define LFB_Pin GPIO_PIN_10
#define LFB_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LFF_Pin GPIO_PIN_3
#define LFF_GPIO_Port GPIOB
#define RBF_Pin GPIO_PIN_4
#define RBF_GPIO_Port GPIOB
#define RBB_Pin GPIO_PIN_5
#define RBB_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_6
#define LCD_CS_GPIO_Port GPIOB
#define RFF_Pin GPIO_PIN_8
#define RFF_GPIO_Port GPIOB
#define RFB_Pin GPIO_PIN_9
#define RFB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
