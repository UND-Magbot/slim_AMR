/**
  ******************************************************************************
  * @file    can2.h
  * @brief   FDCAN2 receive-only bus probe (battery / BMS bring-up).
  *
  *          Receives every standard and extended frame and keeps a small
  *          per-id table. It never transmits. In monitor mode it does not even
  *          ACK; in ack mode it ACKs like a normal node (needed when the BMS is
  *          the only other node - an un-ACKed frame is aborted before it
  *          becomes valid, so a pure listener sees nothing).
  *
  *          Pins (AF9) : set 0 = PB12 RX / PB13 TX, set 1 = PB5 RX / PB6 TX.
  *          (2026-09-29: board silk says CAN2 H/L; which pin pair feeds its
  *           transceiver is not documented, so both can be tried.)
  ******************************************************************************
  */
#ifndef __CAN2_H__
#define __CAN2_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

bool CAN2_Start(uint32_t kbps, uint8_t pinSet, bool ack);
void CAN2_Stop(void);
void CAN2_Poll(void);          /* call from the 100 Hz main loop */
void CAN2_PrintTable(void);
void CAN2_ConsoleCmd(char *args);
bool CAN2_BatteryInit(void);   /* 500k, PB5/6, ACK, poll the Tabos BMS */

#ifdef __cplusplus
}
#endif

#endif /* __CAN2_H__ */
