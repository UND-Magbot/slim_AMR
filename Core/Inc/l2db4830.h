/**
  ******************************************************************************
  * @file    l2db4830.h
  * @brief   L2DB4830-CAFC servo controller driver (CANopen SDO over FDCAN1).
  *
  *   axis 1 -> CAN node 1  (M1 / ENC1)
  *   axis 2 -> CAN node 2  (M2 / ENC2)
  *
  *   Both axes run in speed mode (mode 3, trapezoidal ramp) for the
  *   two-wheel differential drive.
  ******************************************************************************
  */
#ifndef __L2DB4830_H__
#define __L2DB4830_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
  L2DB_AXIS_1 = 0,
  L2DB_AXIS_2,
  L2DB_AXIS_COUNT
} L2dbAxis_t;

/* Confirmed by bring-up probe: object 0x6410:03 reads 4096 on both axes. */
#define L2DB_ENCODER_RESOLUTION   4096U

/* Safety limits applied to every speed command. */
#define L2DB_MAX_RPM              400
#define L2DB_DEFAULT_ACCEL_RPS2   1.0f
#define L2DB_DEFAULT_DECEL_RPS2   1.0f
/* Speed loop gains written at every init (RAM only; the drive's saved
 * values are kp 35 / ki 8). 2026-09-29: ki 64 was the setting for all
 * docking/caster tests since then; 'k 0 35 8' goes back at runtime. */
#define L2DB_SPEED_KP             35U
#define L2DB_SPEED_KI             64U

typedef struct
{
  bool     online;          /* last SDO exchange succeeded            */
  bool     enabled;         /* control word 0x0F has been written     */
  int32_t  commandRpm;      /* last rpm we asked for                  */
  int16_t  actualRpm;       /* 0x60F9:18                              */
  int32_t  actualPosition;  /* 0x6063:00, encoder increments          */
  uint16_t statusWord;      /* 0x6041:00, 0x0008 = alarm              */
  uint16_t errorCode;       /* 0x2601:00, bit field                   */
  int16_t  busVoltage;      /* 0x60F7:12, volts                       */
  int16_t  iqRaw;           /* 0x6078:00, DEC = Arms*1.414*2048/50    */
  uint32_t lastOkMs;
} L2dbAxisState_t;

bool L2DB_AxisInit(L2dbAxis_t axis);
bool L2DB_InitAll(void);

bool L2DB_SetSpeedRpm(L2dbAxis_t axis, int32_t rpm);
bool L2DB_SetSpeedCentiRpm(L2dbAxis_t axis, int32_t centiRpm);
void L2DB_UartStats(void);
void L2DB_PrintDriveLimits(void);
bool L2DB_Enable(L2dbAxis_t axis, bool on);
bool L2DB_ClearAlarm(L2dbAxis_t axis);
bool L2DB_SetRamp(L2dbAxis_t axis, float accelRps2, float decelRps2);
void L2DB_StopAll(void);          /* speed 0 on every axis, keeps axes locked */
void L2DB_ReleaseAll(void);       /* control word 0x06, shafts free           */

/* One-shot configuration: make the drive cut the motor when the CAN link
 * dies. Factory default is DISABLED. Writes to the S1 group and saves it,
 * so it survives a power cycle. Requires a drive reboot to take effect.    */
bool L2DB_ConfigCommLossStop(uint32_t delayMs);

void L2DB_PollFeedback(void);     /* round-robin, one object per call */
void L2DB_PrintFeedback(void);
void L2DB_DecodeError(uint16_t errorCode);
const L2dbAxisState_t *L2DB_GetState(L2dbAxis_t axis);

/* Interactive test console on USART1 (115200). Call from the main loop. */
void L2DB_ConsolePoll(void);
void L2DB_ConsoleHelp(void);
void L2DB_ConsoleExecLine(char *line);   /* also reachable from the USB CDC link */
void L2DB_JumpToBootloader(void);        /* software DFU entry, no BOOT0 jumper */
void L2DB_UartRxInit(void);              /* USART1 RXNE interrupt + ring buffer */
void L2DB_UartReport(void);              /* USART1 register dump for RX debugging */

/* Non-blocking console output (printf goes through here). */
void L2DB_ConsolePutc(uint8_t c);
void L2DB_ConsoleFlush(uint32_t timeoutMs);

#ifdef __cplusplus
}
#endif
#endif /* __L2DB4830_H__ */
