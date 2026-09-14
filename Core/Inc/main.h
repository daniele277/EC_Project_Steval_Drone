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
#include "stm32f4xx_hal.h"

#include "hci_tl_interface.h"
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
#define LPS22HH_CS_Pin GPIO_PIN_13
#define LPS22HH_CS_GPIO_Port GPIOC
#define USB_Monitor_Pin GPIO_PIN_14
#define USB_Monitor_GPIO_Port GPIOC
#define BLE_IRQ_Pin GPIO_PIN_4
#define BLE_IRQ_GPIO_Port GPIOA
#define BLE_IRQ_EXTI_IRQn EXTI4_IRQn
#define BLE_CS_Pin GPIO_PIN_0
#define BLE_CS_GPIO_Port GPIOB
#define VBAT_SENSE_Pin GPIO_PIN_1
#define VBAT_SENSE_GPIO_Port GPIOB
#define BLE_RSTN_Pin GPIO_PIN_2
#define BLE_RSTN_GPIO_Port GPIOB
#define LSM6DSR_CS_Pin GPIO_PIN_8
#define LSM6DSR_CS_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
