#include "isr_probe.h"
#include "rs485.h"

UART_HandleTypeDef USART2_RS485Handler;

static volatile uint8_t s_uartRxRing[RS485_REC_BUFF_SIZE];
static volatile uint16_t s_uartRxHead = 0U;
static volatile uint16_t s_uartRxTail = 0U;
static uint8_t s_uartRxByte = 0U;

static void RS485_RingPush(uint8_t byte)
{
  uint16_t next = (uint16_t)((s_uartRxHead + 1U) % RS485_REC_BUFF_SIZE);
  if (next == s_uartRxTail)
  {
    s_uartRxTail = (uint16_t)((s_uartRxTail + 1U) % RS485_REC_BUFF_SIZE);
  }
  s_uartRxRing[s_uartRxHead] = byte;
  s_uartRxHead = next;
}

void USART2_IRQHandler(void)
{
  uint32_t probeT0 = probe_now();
  HAL_UART_IRQHandler(&USART2_RS485Handler);
  probe_max(&g_probeU2Max, probe_now() - probeT0);
}

void RS485_RxBuffer_Clear(void)
{
  __disable_irq();
  s_uartRxHead = 0U;
  s_uartRxTail = 0U;
  __enable_irq();
}

uint16_t RS485_RxBuffer_Count(void)
{
  uint16_t head;
  uint16_t tail;
  __disable_irq();
  head = s_uartRxHead;
  tail = s_uartRxTail;
  __enable_irq();

  if (head >= tail)
  {
    return (uint16_t)(head - tail);
  }
  return (uint16_t)(RS485_REC_BUFF_SIZE - tail + head);
}

uint16_t RS485_RxBuffer_Read(uint8_t *dest, uint16_t len)
{
  if (len == 0U)
  {
    return 0U;
  }

  uint16_t read = 0U;
  __disable_irq();
  while ((s_uartRxTail != s_uartRxHead) && (read < len))
  {
    uint8_t byte = s_uartRxRing[s_uartRxTail];
    s_uartRxTail = (uint16_t)((s_uartRxTail + 1U) % RS485_REC_BUFF_SIZE);
    if (dest != NULL)
    {
      dest[read] = byte;
    }
    read++;
  }
  __enable_irq();
  return read;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    RS485_RingPush(s_uartRxByte);
    HAL_UART_Receive_IT(&USART2_RS485Handler, &s_uartRxByte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
    HAL_UART_Receive_IT(&USART2_RS485Handler, &s_uartRxByte, 1U);
  }
}

void RS485_Init(uint32_t bound)
{
  GPIO_InitTypeDef GPIO_Initure;
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  GPIO_Initure.Pin = GPIO_PIN_5;
  GPIO_Initure.Mode = GPIO_MODE_AF_PP;
  GPIO_Initure.Pull = GPIO_PULLUP;
  GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_Initure.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_Initure);

  GPIO_Initure.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOD, &GPIO_Initure);

  GPIO_Initure.Pin = GPIO_PIN_7;
  GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_Initure.Pull = GPIO_PULLUP;
  GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_Initure);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

  USART2_RS485Handler.Instance = USART2;
  USART2_RS485Handler.Init.BaudRate = bound;
  USART2_RS485Handler.Init.WordLength = UART_WORDLENGTH_8B;
  USART2_RS485Handler.Init.StopBits = UART_STOPBITS_1;
  USART2_RS485Handler.Init.Parity = UART_PARITY_NONE;
  USART2_RS485Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  USART2_RS485Handler.Init.Mode = UART_MODE_TX_RX;
  HAL_UART_Init(&USART2_RS485Handler);

  __HAL_UART_CLEAR_IT(&USART2_RS485Handler, UART_CLEAR_TCF);

  HAL_NVIC_SetPriority(USART2_IRQn, 3, 3);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  RS485_TX_Set(0);
  RS485_RxBuffer_Clear();
  HAL_UART_Receive_IT(&USART2_RS485Handler, &s_uartRxByte, 1U);
}

void RS485_Send_Data(uint8_t *buf, uint8_t len)
{
  RS485_TX_Set(1);
  HAL_UART_Transmit(&USART2_RS485Handler, buf, len, 1000);
  RS485_TX_Set(0);
}

void RS485_Receive_Data(uint8_t *buf, uint8_t *len)
{
  if ((buf == NULL) || (len == NULL))
  {
    return;
  }
  uint16_t available = RS485_RxBuffer_Count();
  if (available > UINT8_MAX)
  {
    available = UINT8_MAX;
  }
  uint16_t read = RS485_RxBuffer_Read(buf, available);
  *len = (uint8_t)read;
}

void RS485_TX_Set(uint8_t en)
{
  if (en == 0U)
  {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
  }
}
