/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "can2.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "rs485.h"
#include "lift.h"
#include "system.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbcdc.h"
#include "usbd_cdc_if.h"
#include "can.h"
#include "l2db4830.h"

/* Migration switches: Kinco FD125 (RS485/Modbus) -> L2DB4830 (CAN/CANopen SDO).
 * Set APP_USE_KINCO_RS485 back to 1 to fall back to the old motor path.      */
#define APP_USE_KINCO_RS485   0
#define APP_CAN_BRINGUP       1
/* USER CODE END Includes */

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

/* USER CODE BEGIN PV */
static volatile uint32_t g_LoopCnt = 0U;
static volatile uint32_t g_1msCnt = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/


/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

USBD_HandleTypeDef USBD_Device;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  Lift_Init();                 /* PC0 up / PE1 down lifting outputs, both LOW */
  /* MX_USART2_UART_Init(); */
  RS485_Init(115200);
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_Delay(50);
#if APP_CAN_BRINGUP
  L2DB_UartRxInit();
#endif
#if APP_USE_KINCO_RS485
  KincoFD125_Init();
#endif
  printf("System initialized\r\n");
#if APP_CAN_BRINGUP
  CAN_BringUp();
  (void)CAN2_BatteryInit();
  L2DB_UartReport();
  L2DB_ConsoleHelp();
  printf("> ");
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastLoopCnt = 0U;
  while (1)
  {

	if (g_LoopCnt == lastLoopCnt)
    {
      /*임시 주석 처리  __WFI();  */
      continue;
    }
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_4);
    lastLoopCnt = g_LoopCnt;

#if APP_USE_KINCO_RS485
    App_KeyHandler();
    KincoFD125_PollFeedback();
#endif
#if APP_CAN_BRINGUP
    /* Note: never run CAN_SnifferPoll() here - it drains RX FIFO0 and would
     * swallow the SDO responses the driver is waiting for.                  */
    L2DB_ConsolePoll();
    L2DB_PollFeedback();
    CAN_BusWatch();              /* FDCAN1 bus-off -> rejoin (2026-10-26) */
    CAN2_Poll();
#endif
    App_UsbCdc_Handler();
    Lift_Poll();                 /* ends a timed up / down move */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) /* 10us timer */
{
  if (htim->Instance == TIM6)
  {
    static uint32_t tickDiv10ms = 0U;
    static uint32_t tickDiv1ms = 0U;

    tickDiv1ms++;
    if (tickDiv1ms >= MAIN_LOOP_1MS_COUNT)		/* 1ms */
    {
      tickDiv1ms = 0U;
      if( UINT32_MAX <= g_1msCnt++) g_1msCnt = 0;
    }

    tickDiv10ms++;
    if (tickDiv10ms >= MAIN_LOOP_TICK_COUNT)		/* 10ms */
    {
      tickDiv10ms = 0U;
      if( UINT32_MAX <= g_LoopCnt++) g_LoopCnt = 0;
    }
  }
}

uint32_t api_getCurtMs(void)
{
	return g_1msCnt;
}


/* printf -> USART1. Bytes go into a ring buffer and the USART1 interrupt
 * sends them (L2DB_ConsolePutc / L2DB_ConsoleTxIsr).
 * 2026-09-30: this used HAL_UART_Transmit one byte at a time. newlib's stdio
 * lock (STM32_THREAD_SAFE_STRATEGY=2) disables ALL interrupts while printf
 * runs, so an 80-char status line blocked every interrupt for ~7 ms at
 * 115200 - the USART1 RX FIFO (16 bytes = 1.4 ms) overflowed and command
 * lines were lost (probe: TIM6 gap 10 ms, RX burst 16-18 bytes). */
int __io_putchar(int ch)
{
  L2DB_ConsolePutc((uint8_t)ch);
  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
