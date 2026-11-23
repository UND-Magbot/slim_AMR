/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tricycle.c
  * @brief   Tricycle drive command processing.
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

/* Includes ------------------------------------------------------------------*/
#include "tricycle.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"

#include <math.h>
#include <stdbool.h>

/* USER CODE BEGIN 0 */
#define REMOTE_CMD_DEADBAND              0.01f

#define TRICYCLE_MAX_LINEAR_MPS          0.5f			/* max speed */
#define TRICYCLE_MAX_ANGULAR_RADPS       1.6f
#define TRICYCLE_WHEEL_BASE_M            0.5f            /* Wheel base(between front wheel and rear wheel) in meters */
#define TRICYCLE_STEER_LIMIT_RAD         1.396f          /* 70 deg : 1.221730476f , 80 deg : 1.396f */
#define TRICYCLE_LINEAR_EPSILON          0.01f
#define TRICYCLE_COS_EPSILON             0.087155743f   /* cos(85 deg) */
#define TRICYCLE_SPEED_RAMP_MPS2         0.8f
#define TRICYCLE_MIN_LINEAR_TURN_MPS     0.05f          /* Minimum linear speed when in-place turning */  

ControllerState_t g_control;

static float apply_deadband(float value, float deadband)
{
  if (fabsf(value) < deadband)
  {
    return 0.0f;
  }
  return value;
}

static float apply_ramp(float target, float current, float maxRate, float dt)
{
  float delta = target - current;
  float maxStep = maxRate * dt;
  if ((dt <= 0.0f) || (fabsf(delta) <= maxStep))
  {
    return target;
  }
  return current + copysignf(maxStep, delta);
}

void Api_CheckBelanced(float *steerCmd, float *walkCmd)
{
  if ((steerCmd == NULL) || (walkCmd == NULL))
  {
    return;
  }

  /* App_KeyHandler historically invokes this function as (walk, steer).
     Preserve the existing call-site ordering for now. */

  float rawLinear = *walkCmd;
  float rawAngular = *steerCmd;

  g_control.CmdVelLinearX = rawLinear;
  g_control.CmdVelAngularZ = rawAngular;

  float limitedLinear = CheckMinMax(rawLinear, -TRICYCLE_MAX_LINEAR_MPS, TRICYCLE_MAX_LINEAR_MPS);
  float limitedAngular = CheckMinMax(rawAngular, -TRICYCLE_MAX_ANGULAR_RADPS, TRICYCLE_MAX_ANGULAR_RADPS);

  limitedLinear = apply_deadband(limitedLinear, REMOTE_CMD_DEADBAND);
  limitedAngular = apply_deadband(limitedAngular, REMOTE_CMD_DEADBAND);

  float steerAngle = 0.0f;
  bool linearIsZero = (fabsf(limitedLinear) < TRICYCLE_LINEAR_EPSILON);
  bool hasSteerCmd = (fabsf(limitedAngular) >= REMOTE_CMD_DEADBAND);

  if (linearIsZero && hasSteerCmd)
  {
    limitedLinear = TRICYCLE_MIN_LINEAR_TURN_MPS;
    linearIsZero = false;
  }

  if (!linearIsZero)
  {
    float ratio = (TRICYCLE_WHEEL_BASE_M * limitedAngular) / limitedLinear;
    steerAngle = atanf(ratio);
  }

  steerAngle = CheckMinMax(steerAngle, -TRICYCLE_STEER_LIMIT_RAD, TRICYCLE_STEER_LIMIT_RAD);

  if (!linearIsZero)
  {
    /* Update angular command so that the limited steering angle and linear velocity remain consistent. */
    float tanSteer = tanf(steerAngle);
    limitedAngular = (TRICYCLE_WHEEL_BASE_M > 0.0f) ? (limitedLinear * tanSteer / TRICYCLE_WHEEL_BASE_M) : 0.0f;
    limitedAngular = CheckMinMax(limitedAngular, -TRICYCLE_MAX_ANGULAR_RADPS, TRICYCLE_MAX_ANGULAR_RADPS);
  }

  float driveLinear = limitedLinear;
  if (!linearIsZero && fabsf(steerAngle) > 0.0f)
  {
    float cosSteer = cosf(steerAngle);
    if (fabsf(cosSteer) < TRICYCLE_COS_EPSILON)
    {
      cosSteer = (cosSteer >= 0.0f) ? TRICYCLE_COS_EPSILON : -TRICYCLE_COS_EPSILON;
    }
    driveLinear = limitedLinear / cosSteer;
  }
  driveLinear = CheckMinMax(driveLinear, -TRICYCLE_MAX_LINEAR_MPS, TRICYCLE_MAX_LINEAR_MPS);

  g_control.TargetLinearX = driveLinear;
  g_control.TargetAngularZ = limitedAngular;

  uint32_t nowMs = api_getCurtMs();
  float dt = 0.0f;
  if (g_control.steerTimestampMs != 0U)
  {
    uint32_t diff = HAL_TimDiff(nowMs, g_control.steerTimestampMs);
    dt = diff / 1000.0f;
  }
  g_control.steerTimestampMs = nowMs;

  if (g_control.steerTimestampMs == 0U || dt <= 0.0f)
  {
    g_control.CurrentLinearX = g_control.TargetLinearX;
  }
  else
  {
    g_control.CurrentLinearX = apply_ramp(g_control.TargetLinearX,
                                          g_control.CurrentLinearX,
                                          TRICYCLE_SPEED_RAMP_MPS2,
                                          dt);
  }

  g_control.CurrentAngularZ = g_control.TargetAngularZ;

  *walkCmd = g_control.CurrentLinearX;
  *steerCmd = g_control.CurrentAngularZ;
}

void Api_ChangeRadsToRad(float *steerCmd, float *walkCmd)
{
  if (steerCmd == NULL)
  {
    return;
  }

  float angularZ = *steerCmd;
  float linearX = *walkCmd;
  float steerAngle = 0.0f;

  if (fabsf(linearX) >= TRICYCLE_LINEAR_EPSILON)
  {
    float ratio = (TRICYCLE_WHEEL_BASE_M * angularZ) / linearX;
    steerAngle = atanf(ratio);
  }

  steerAngle = CheckMinMax(steerAngle, -TRICYCLE_STEER_LIMIT_RAD, TRICYCLE_STEER_LIMIT_RAD);
  *steerCmd = steerAngle;
}
/* USER CODE END 0 */
