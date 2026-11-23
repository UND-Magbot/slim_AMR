#ifndef __RS485_H
#define __RS485_H

#include "main.h"

#define RS485_REC_BUFF_SIZE 256U

extern UART_HandleTypeDef USART2_RS485Handler;

void RS485_Init(uint32_t bound);
void RS485_Send_Data(uint8_t *buf, uint8_t len);
void RS485_Receive_Data(uint8_t *buf, uint8_t *len);
void RS485_TX_Set(uint8_t en);
void RS485_RxBuffer_Clear(void);
uint16_t RS485_RxBuffer_Count(void);
uint16_t RS485_RxBuffer_Read(uint8_t *dest, uint16_t len);

#endif /* __RS485_H */
