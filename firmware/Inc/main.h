/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f0xx_hal.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"

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
#define IIN_Pin GPIO_PIN_0
#define IIN_GPIO_Port GPIOA
#define TTIP_Pin GPIO_PIN_1
#define TTIP_GPIO_Port GPIOA
#define UIN_Pin GPIO_PIN_2
#define UIN_GPIO_Port GPIOA
#define B1_Pin GPIO_PIN_3
#define B1_GPIO_Port GPIOA
#define B1_1_Pin GPIO_PIN_13
#define B1_1_GPIO_Port GPIOC
#define B2_Pin GPIO_PIN_4
#define B2_GPIO_Port GPIOA
#define TREF_Pin GPIO_PIN_5
#define TREF_GPIO_Port GPIOA
#define STUSB_SCL_Pin GPIO_PIN_10
#define STUSB_SCL_GPIO_Port GPIOB
#define STUSB_SDA_Pin GPIO_PIN_11
#define STUSB_SDA_GPIO_Port GPIOB
#define INT_N_Pin GPIO_PIN_12
#define INT_N_GPIO_Port GPIOB
#define PWMOUT_Pin GPIO_PIN_8
#define PWMOUT_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
