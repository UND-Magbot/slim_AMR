/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbcdc.h
  * @brief          : Header for usbcdc.c file.
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
#ifndef __USBCDC_H
#define __USBCDC_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef struct
{
  float cmdVelLinearX;
  float cmdVelAngularZ;
  bool  cmdVelValid;
  uint32_t cmdVelTimestampMs;

  float walkVelocity;
  bool  walkValid;
  uint32_t walkTimestampMs;

  float steerPosition;
  bool  steerValid;
  uint32_t steerTimestampMs;
} UsbCdcCommandState_t;

void App_UsbCdc_Handler(void);
void UsbCdc_GetCommandState(UsbCdcCommandState_t *state);
void UsbCdc_ClearCommandFlags(void);


#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_H */
