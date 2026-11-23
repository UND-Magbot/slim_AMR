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
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include "RS485_KincoFD125.h"
#include "usbcdc.h"
#include "tim.h"
#include "tricycle.h"
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
uint32_t api_getCurtMs(void);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define J_LEFT_Pin GPIO_PIN_6
#define J_LEFT_GPIO_Port GPIOC
#define J_RIGHT_Pin GPIO_PIN_5
#define J_RIGHT_GPIO_Port GPIOE
#define J_UP_Pin GPIO_PIN_8
#define J_UP_GPIO_Port GPIOD
#define E_STOP_Pin GPIO_PIN_9
#define E_STOP_GPIO_Port GPIOD
#define J_DOWN_Pin GPIO_PIN_10
#define J_DOWN_GPIO_Port GPIOD

#define USART2_DE_Pin GPIO_PIN_7
#define USART2_DE_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

#define MAIN_LOOP_TICK_COUNT        1000U   /* 1000 * 10us = 10ms */
#define MAIN_LOOP_1MS_COUNT        	100U   /* 100 * 10us = 1ms */
#define USB_HEARTBEAT_PERIOD_MS     100U
#define DEBUG_STATUS_PERIOD_MS      500U

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
