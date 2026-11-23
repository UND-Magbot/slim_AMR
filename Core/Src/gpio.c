/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include <stdio.h>
#include "main.h"
#include "gpio.h"
#include "usbcdc.h"
#include "usart.h"
/* USER CODE BEGIN 0 */

// Joystick linear velocity: 0.5 m/s base, +0.5 m/s each 1 s, max 5 m/s
#define JOY_LINEAR_STEP              0.05f
#define JOY_LINEAR_MAX               0.25f
#define JOY_LINEAR_STEP_PERIOD_MS    1500U

// Joystick angular velocity: 0.3 rad/s base, +0.3 rad/s each 0.5 s, max 2 rad/s
#define JOY_ANGULAR_STEP             0.1f
#define JOY_ANGULAR_MAX              1.0f
#define JOY_ANGULAR_STEP_PERIOD_MS   500U

// After joystick activity, block remote USB commands for 3 seconds
#define JOY_REMOTE_BLOCK_MS          3000U
#define REMOTE_CMD_TIMEOUT_MS        500U
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
void MX_GPIO_Default_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/*Configure GPIO pins : PC13 PC14 PC15 PC0
						   PC1 PC2 PC3 PC4
						   PC5 PC6 PC7 PC8
						   PC9 PC10 PC11 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
						  |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
						  |GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8
						  |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PH0 PH1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 PA2 PA3
						   PA4 PA5 PA6 PA7
						   PA8 PA13 PA14 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
						  |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB10
						   PB11 PB12 PB13 PB14
						   PB15 PB3 PB4 PB5
						   PB6 PB7 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
						  |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
						  |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
						  |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PD9 PD11 PD12 PD13
						   PD14 PD15 PD0 PD1
						   PD2 PD3 PD4 */
	GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
						  |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
						  |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : PE2 PE3 PE7 PE8
						   PE9 PE10 PE11 PE12
						   PE13 PE14 PE15 PE0
						   PE1 */
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_8
						  |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
						  |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
						  |GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);



}
/* USER CODE END 1 */

/** Configure pins
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  MX_GPIO_Default_Init();

  /*Configure GPIO pins :   J_RIGHT_Pin */
  GPIO_InitStruct.Pin = J_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(J_RIGHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E_STOP_Pin J_DOWN_Pin J_UP_Pin */
  GPIO_InitStruct.Pin = E_STOP_Pin|J_DOWN_Pin|J_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : J_LEFT_Pin  */
  GPIO_InitStruct.Pin = J_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(J_LEFT_GPIO_Port, &GPIO_InitStruct);


  /*Configure GPIO pin : USART2_DE_Pin */
  GPIO_InitStruct.Pin = USART2_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USART2_DE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
// Joystick-based velocity state
static float    s_joyLinear           = 0.0f;
static float    s_joyAngular          = 0.0f;
static uint32_t s_lastLinearUpdateMs  = 0U;
static uint32_t s_lastAngularUpdateMs = 0U;
static int8_t   s_lastJoyWalkDir      = 0;
static int8_t   s_lastJoySteerDir     = 0;

static uint32_t s_lastJoyActiveMs     = 0U;

static bool IsTimestampFresh(uint32_t nowMs, uint32_t stampMs)
{
  uint32_t diff = (nowMs >= stampMs) ? (nowMs - stampMs) : (UINT32_MAX - stampMs + nowMs + 1U);
  return (diff <= REMOTE_CMD_TIMEOUT_MS);
}

float CheckMinMax(float val, float Min, float Max)
{
	if(val < Min)
		val = Min;
	else if(val > Max)
		val = Max;

	return val;
}

bool IsGpioActiveLow(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
}

void Joystick_Comd(int8_t *steerCmd, int8_t *walkCmd)
{
  if ((steerCmd == NULL) || (walkCmd == NULL))
  {
    return;
  }

  *steerCmd = 0;
  *walkCmd = 0;

  if (IsGpioActiveLow(J_LEFT_GPIO_Port, J_LEFT_Pin))
  {
    *steerCmd = -1;
  }
  else if (IsGpioActiveLow(J_RIGHT_GPIO_Port, J_RIGHT_Pin))
  {
    *steerCmd = 1;
  }


  if (IsGpioActiveLow(J_UP_GPIO_Port, J_UP_Pin))
  {
    *walkCmd = 1;
  }
  else if (IsGpioActiveLow(J_DOWN_GPIO_Port, J_DOWN_Pin))
  {
    *walkCmd = -1;
  }
}

void Joystick_Active(int8_t joySteer, int8_t joyWalk)
{
  
    /* 2) Update joystick linear velocity (m/s) */
    if (joyWalk == 0)
    {
      s_joyLinear = 0.0f;
      s_lastJoyWalkDir = 0;
      s_lastLinearUpdateMs = api_getCurtMs();
    }
    else
    {
      if (s_lastJoyWalkDir != joyWalk)
      {
        s_joyLinear = JOY_LINEAR_STEP * (float)joyWalk;
        s_lastJoyWalkDir = joyWalk;
        s_lastLinearUpdateMs = api_getCurtMs();
      }
      else
      {
        uint32_t diff = HAL_TimDiff(api_getCurtMs(), s_lastLinearUpdateMs);
        if (diff >= JOY_LINEAR_STEP_PERIOD_MS)
        {
          uint32_t steps = diff / JOY_LINEAR_STEP_PERIOD_MS;
          float newLinear = s_joyLinear + (float)joyWalk * JOY_LINEAR_STEP * (float)steps;
          s_joyLinear = CheckMinMax(newLinear,-JOY_LINEAR_MAX, JOY_LINEAR_MAX);
          s_lastLinearUpdateMs += JOY_LINEAR_STEP_PERIOD_MS * steps;
        }
      }
    }

    /* 3) Update joystick angular velocity (rad/s) */
    if (joySteer == 0)
    {
      s_joyAngular = 0.0f;
      s_lastJoySteerDir = 0;
      s_lastAngularUpdateMs = api_getCurtMs();
    }
    else
    {
      if (s_lastJoySteerDir != joySteer)
      {
        s_joyAngular = JOY_ANGULAR_STEP * (float)joySteer;
        s_lastJoySteerDir = joySteer;
        s_lastAngularUpdateMs = api_getCurtMs();
      }
      else
      {
        uint32_t diff = HAL_TimDiff(api_getCurtMs(), s_lastAngularUpdateMs);
        if (diff >= JOY_ANGULAR_STEP_PERIOD_MS)
        {
          uint32_t steps = diff / JOY_ANGULAR_STEP_PERIOD_MS;
          float newAngular = s_joyAngular + (float)joySteer * JOY_ANGULAR_STEP * (float)steps;
          s_joyAngular = CheckMinMax(newAngular,-JOY_ANGULAR_MAX, JOY_ANGULAR_MAX);
          s_lastAngularUpdateMs += JOY_ANGULAR_STEP_PERIOD_MS * steps;
        }
      }
	  }
}

static void UsbCdc_Comd(float *linearCmd, float *angularCmd, uint32_t elapsedMs, bool *hasCmd)
{
  if ((linearCmd == NULL) || (angularCmd == NULL) || (hasCmd == NULL))
  {
    return;
  }

  *linearCmd = 0.0f;
  *angularCmd = 0.0f;
  *hasCmd = false;

  UsbCdcCommandState_t usbState;
  UsbCdc_GetCommandState(&usbState);

  if (usbState.cmdVelValid && IsTimestampFresh(elapsedMs, usbState.cmdVelTimestampMs))
  {
    *linearCmd  = usbState.cmdVelLinearX;
    *angularCmd = usbState.cmdVelAngularZ;
    *hasCmd = true;
  }
}


void App_KeyHandler()
{

  /* 1) Read joystick directions */
  int8_t joySteer = 0;
  int8_t joyWalk  = 0;
  float steerCmd  = 0.0;
  float walkCmd	  = 0.0;
  bool blockRemote = false;
  bool estopActive = IsGpioActiveLow(E_STOP_GPIO_Port, E_STOP_Pin);

  Joystick_Comd(&joySteer, &joyWalk);

  bool joyActive = ((joySteer != 0) || (joyWalk != 0));

  if (joyActive)
  {
	  blockRemote = true;
    s_lastJoyActiveMs = api_getCurtMs();
    Joystick_Active(joySteer, joyWalk);
  }

  /* key 입력후 5초 동안 usbcmd 안먹음 */
  if (s_lastJoyActiveMs != 0U)
  {
    uint32_t diff = HAL_TimDiff(api_getCurtMs(), s_lastJoyActiveMs);
    if (diff < JOY_REMOTE_BLOCK_MS)
    {
      blockRemote = true;
    }
  }

  /* 4) Read USB CMD (/cmd_vel) */
  float usbLinear  = 0.0f;
  float usbAngular = 0.0f;
  bool  usbHasCmd  = false;
  UsbCdc_Comd(&usbLinear, &usbAngular, api_getCurtMs(), &usbHasCmd);

  /* 5) Joystick has priority; block USB for JOY_REMOTE_BLOCK_MS after last joystick activity */
  if (blockRemote)
  {
    if (joyActive)
    {
    	walkCmd  = s_joyLinear;
    	steerCmd = s_joyAngular;
    }
  }
  else
  {
    if (usbHasCmd)
    {
    	walkCmd  = usbLinear;
    	steerCmd = usbAngular;
    }
  }

  printf("input cmd: walkCmd=%f, steerCmd=%f\r\n", walkCmd, steerCmd);
  Api_CheckBelanced(&steerCmd, &walkCmd);
  Api_ChangeRadsToRad(&steerCmd, &walkCmd);
  printf("balanced cmd: walkCmd=%f, steerCmd=%f\r\n", walkCmd, steerCmd);
  
  /* 7) E-STOP and motor update */

  if (estopActive)
  {
    KincoFD125_EStop();
  }
  else
  {
    KincoFD125_Update(steerCmd, walkCmd);
  }

}

/* USER CODE END 2 */
