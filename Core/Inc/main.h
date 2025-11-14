/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "types.h"

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
#define OVERCONSUMPTION_MS 120000
#define BLINK_MS 1000
#define REPORTCYCLE_MS 5000
#define BTN_Pin GPIO_PIN_14
#define BTN_GPIO_Port GPIOC
#define LED_STATUS_Pin GPIO_PIN_15
#define LED_STATUS_GPIO_Port GPIOC
#define SSR0_Pin GPIO_PIN_0
#define SSR0_GPIO_Port GPIOA
#define SSR1_Pin GPIO_PIN_1
#define SSR1_GPIO_Port GPIOA
#define SSR2_Pin GPIO_PIN_2
#define SSR2_GPIO_Port GPIOA
#define SSR3_Pin GPIO_PIN_3
#define SSR3_GPIO_Port GPIOA
#define SSR4_Pin GPIO_PIN_4
#define SSR4_GPIO_Port GPIOA
#define SSR5_Pin GPIO_PIN_5
#define SSR5_GPIO_Port GPIOA
#define SSR6_Pin GPIO_PIN_6
#define SSR6_GPIO_Port GPIOA
#define SSR7_Pin GPIO_PIN_7
#define SSR7_GPIO_Port GPIOA
#define SSR8_Pin GPIO_PIN_0
#define SSR8_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_1
#define LED0_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_4
#define LED5_GPIO_Port GPIOB
#define LED6_Pin GPIO_PIN_5
#define LED6_GPIO_Port GPIOB
#define LED7_Pin GPIO_PIN_6
#define LED7_GPIO_Port GPIOB
#define LED8_Pin GPIO_PIN_7
#define LED8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define NB_LED 9
#define NB_SSR 9
#define EMPTY_SLOT 0xff

#define STR_CR 0x0D
#define STR_LF 0x0A

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
