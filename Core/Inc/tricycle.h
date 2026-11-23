/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tricycle.h
  * @brief   This file contains all the function prototypes for
  *          the tricycle.c file
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
#ifndef __TRICYCLE_H__
#define __TRICYCLE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
typedef struct
{
  float CmdVelLinearX;
  float CmdVelAngularZ;
  
  float TargetLinearX;
  float TargetAngularZ;

  float CurrentLinearX;
  float CurrentAngularZ;

  uint32_t steerTimestampMs;
} ControllerState_t;
/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
extern ControllerState_t g_control;
void Api_CheckBelanced(float *steerCmd, float *walkCmd);
void Api_ChangeRadsToRad(float *steerCmd, float *walkCmd);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ TRICYCLE_H__ */
