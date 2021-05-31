/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define relay2_Pin GPIO_PIN_14
#define relay2_GPIO_Port GPIOC
#define ADC1_VBUS_Pin GPIO_PIN_0
#define ADC1_VBUS_GPIO_Port GPIOA
#define ADC2_IU_Pin GPIO_PIN_1
#define ADC2_IU_GPIO_Port GPIOA
#define ADC1_IV_Pin GPIO_PIN_2
#define ADC1_IV_GPIO_Port GPIOA
#define VFBK_2_Pin GPIO_PIN_3
#define VFBK_2_GPIO_Port GPIOA
#define nRF24_CSN_Pin GPIO_PIN_4
#define nRF24_CSN_GPIO_Port GPIOA
#define nRF24_IRQ_Pin GPIO_PIN_0
#define nRF24_IRQ_GPIO_Port GPIOB
#define nRF24_IRQ_EXTI_IRQn EXTI0_IRQn
#define nRF24_CE_Pin GPIO_PIN_1
#define nRF24_CE_GPIO_Port GPIOB
#define DBG_Pin GPIO_PIN_11
#define DBG_GPIO_Port GPIOB
#define Encoder_Z_Pin GPIO_PIN_4
#define Encoder_Z_GPIO_Port GPIOB
#define Encoder_Z_EXTI_IRQn EXTI4_IRQn
#define INP_4_Pin GPIO_PIN_5
#define INP_4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
