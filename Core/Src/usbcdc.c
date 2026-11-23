
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : system.c
  * @brief          : Main call system code
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
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "rs485.h"
#include "system.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "RS485_KincoFD125.h"
#include "usbcdc.h"
#include "usbd_cdc_if.h"
#include "l2db4830.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define USB_CMD_TIMEOUT_MS          500U
#define USB_ODOM_PUBLISH_PERIOD_MS   20U
#define STEER_MAX_RAD               1.570796327f   /* +/- 90 deg */
#define WHEEL_RADIUS_M              0.08f
#define WHEEL_BASE_M                0.4f

#define CALIB_GAIN                  1.41045f    /* 2m 이동시 1.43m로 return 되는거 보정 상수 */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static UsbCdcCommandState_t g_cmdState;
static float g_odomPosX = 0.0f;
static float g_odomPosY = 0.0f;
static float g_odomYaw = 0.0f;
static uint32_t g_lastOdomIntegrateMs = 0U;
static uint32_t g_lastOdomPublishMs = 0U;
static float g_jointPosition[2] = {0.0f, 0.0f};
static float g_jointVelocity[2] = {0.0f, 0.0f};

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void UsbCdc_HandleLine(char *line);
static float clampf(float value, float minVal, float maxVal);
/* USER CODE END PFP */

void App_UsbCdc_Handler(void)
{
  uint8_t rxBuffer[128];
  uint32_t rxLength = 0U;
  uint32_t elapsedMs = api_getCurtMs();

  memset(rxBuffer, 0, sizeof(rxBuffer));

  if (USB_CDC_TryReceive(rxBuffer, sizeof(rxBuffer), &rxLength) && (rxLength > 0U))
  {
    if (rxLength >= sizeof(rxBuffer))
    {
      rxLength = sizeof(rxBuffer) - 1U;
    }
    rxBuffer[rxLength] = '\0';
    printf("USB RX (%lu ms): %s\r\n", api_getCurtMs(), rxBuffer);
    UsbCdc_HandleLine((char *)rxBuffer);
  }

  /* Integrate odometry (prefer encoder feedback, fallback to cmd_vel) */
  {
    float linear = 0.0f;
    float angular = 0.0f;
    float fbWalkRpm = 0.0f;
    float fbWalkRad = 0.0f;
    float fbSteerRad = 0.0f;
    float fbSteerVel = 0.0f;
    bool feedbackValid = KincoFD125_GetJointFeedback(&fbWalkRpm, &fbWalkRad, &fbSteerRad, &fbSteerVel);

    if (feedbackValid)
    {
      float steer = clampf(fbSteerRad, -STEER_MAX_RAD, STEER_MAX_RAD);
      float linearEnc = fbWalkRad * WHEEL_RADIUS_M * CALIB_GAIN;
      linear = linearEnc;
      angular = (WHEEL_BASE_M > 0.0f) ? (linearEnc * tanf(steer) / WHEEL_BASE_M) : 0.0f;
    }
    else if (HAL_TimIsFresh(elapsedMs, g_cmdState.cmdVelTimestampMs, USB_CMD_TIMEOUT_MS) && g_cmdState.cmdVelValid)
    {
      linear = g_cmdState.cmdVelLinearX;
      angular = g_cmdState.cmdVelAngularZ;
    }
    
    uint32_t diffMs = (g_lastOdomIntegrateMs == 0U) ? 0U : HAL_TimDiff(elapsedMs, g_lastOdomIntegrateMs);
    float dt = diffMs / 1000.0f;
    if (diffMs == 0U)
    {
      dt = 0.0f;
    }
    g_lastOdomIntegrateMs = elapsedMs;
    if (dt > 0.0f)
    {
      g_odomYaw += angular * dt;
      float cosYaw = cosf(g_odomYaw);
      float sinYaw = sinf(g_odomYaw);
      g_odomPosX += linear * cosYaw * dt;
      g_odomPosY += linear * sinYaw * dt;
    }
  }

  /* Publish odometry at 20 ms period */
  uint32_t publishDiffMs = HAL_TimDiff(elapsedMs, g_lastOdomPublishMs);
  if (publishDiffMs >= USB_ODOM_PUBLISH_PERIOD_MS)
  {
	if (USB_CDC_IsReady())
	{
	  float linear = 0.0f;
	  float angular = 0.0f;
	  float fbWalkRpm = 0.0f;
	  float fbWalkRad = 0.0f;
	  float fbSteerRad = 0.0f;
	  float fbSteerVel = 0.0f;
	  bool feedbackValid = KincoFD125_GetJointFeedback(&fbWalkRpm, &fbWalkRad, &fbSteerRad, &fbSteerVel);

	  if (feedbackValid)
	  {
		  float steer = clampf(fbSteerRad, -STEER_MAX_RAD, STEER_MAX_RAD);
		  float linearEnc = fbWalkRad * WHEEL_RADIUS_M;
		  linear = linearEnc;
		  angular = (WHEEL_BASE_M > 0.0f) ? (linearEnc * tanf(steer) / WHEEL_BASE_M) : 0.0f;
	  }
	  else if (HAL_TimIsFresh(elapsedMs, g_cmdState.cmdVelTimestampMs, USB_CMD_TIMEOUT_MS) && g_cmdState.cmdVelValid)
	  {
		  linear = g_cmdState.cmdVelLinearX;
		  angular = g_cmdState.cmdVelAngularZ;
	  }

	  char message[160];
	  float yaw = g_odomYaw;
	  float linearY = 0.0f;
	  int length = snprintf(message, sizeof(message),
							"ODOM %.6f %.6f %.6f %.6f %.6f %.6f\r\n",
							g_odomPosX,
							g_odomPosY,
							yaw,
							linear,
							linearY,
							angular);
	  if (length > 0)
	  {
		  USB_CDC_SendBuffer((uint8_t *)message, (uint16_t)length);
		  // printf("%s /r/n",message);
	  }
	  /* Update joint velocities and positions for joint_states */
	  {
      float dt = (g_lastOdomPublishMs == 0U || publishDiffMs == UINT32_MAX)? (USB_ODOM_PUBLISH_PERIOD_MS / 1000.0f) : (publishDiffMs / 1000.0f);
      float fbWalkRpm = 0.0f;
      float fbWalkRad = 0.0f;
      float fbSteerRad = g_jointPosition[1];
      float fbSteerVel = 0.0f;
      bool feedbackValid = KincoFD125_GetJointFeedback(&fbWalkRpm, &fbWalkRad, &fbSteerRad, &fbSteerVel);

      if (feedbackValid)
      {
        g_jointVelocity[0] = fbWalkRad;
        g_jointPosition[0] += fbWalkRad * dt;

        g_jointPosition[1] = fbSteerRad;
        g_jointVelocity[1] = fbSteerVel;
      }
      else
      {
        const float separation = 0.4f;
        float linearPer = linear;
        float angularPer = angular * (separation * 0.5f);
        g_jointVelocity[0] = linearPer + angularPer;
        g_jointVelocity[1] = linearPer - angularPer;
        g_jointPosition[0] += g_jointVelocity[0] * dt;
        g_jointPosition[1] += g_jointVelocity[1] * dt;
      }

      char jointMsg[128];
      int jointLen = snprintf(jointMsg, sizeof(jointMsg),
                  "JOINT %.6f %.6f %.6f %.6f\r\n",
                  g_jointPosition[0],
                  g_jointVelocity[0],
                  g_jointPosition[1],
                  g_jointVelocity[1]);
      if (jointLen > 0)
      {
        USB_CDC_SendBuffer((uint8_t *)jointMsg, (uint16_t)jointLen);
      }
	  }
	  g_lastOdomPublishMs = elapsedMs;
	}
  }
}

/* USER CODE BEGIN 1 */
static void UsbCdc_HandleLine(char *line)
{
  if (line == NULL)
  {
    return;
  }

  while ((*line == ' ') || (*line == '\t'))
  {
    line++;
  }

  size_t len = strlen(line);
  while (len > 0U && (line[len - 1U] == '\r' || line[len - 1U] == '\n'))
  {
    line[--len] = '\0';
  }

  if (len == 0U)
    return;

  if (strncmp(line, "CMD ", 4) == 0)
  {
    float lx = 0.0f, ly = 0.0f, lz = 0.0f;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    if (sscanf(line + 4, "%f %f %f %f %f %f", &lx, &ly, &lz, &ax, &ay, &az) == 6)
    {
      g_cmdState.cmdVelLinearX = lx;
      g_cmdState.cmdVelAngularZ = az;
      g_cmdState.cmdVelValid = true;
      g_cmdState.cmdVelTimestampMs = api_getCurtMs();
    }
    return;
  }

  /* Anything that is not a cmd_vel goes to the L2DB test console, so the
   * drive can be driven over the USB CDC link (/dev/ttyACM0) as well.
   * Responses still come out on USART1.                                    */
  L2DB_ConsoleExecLine(line);
}

void UsbCdc_GetCommandState(UsbCdcCommandState_t *state)
{
  if (state == NULL)
  {
    return;
  }

  *state = g_cmdState;
}

void UsbCdc_ClearCommandFlags(void)
{
  g_cmdState.cmdVelValid = false;
  g_cmdState.walkValid = false;
  g_cmdState.steerValid = false;
}

static float clampf(float value, float minVal, float maxVal)
{
  if (value < minVal)
  {
    return minVal;
  }
  if (value > maxVal)
  {
    return maxVal;
  }
  return value;
}
/* USER CODE END 1 */
