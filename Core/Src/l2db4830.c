/**
  ******************************************************************************
  * @file    l2db4830.c
  * @brief   L2DB4830-CAFC servo driver + interactive test console.
  ******************************************************************************
  */

#include "l2db4830.h"
#include "isr_probe.h"
#include "can.h"
#include "can2.h"
#include "lift.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_core.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* usb_device.c defines this but its header does not export it. */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* axis -> CAN node id.
 * 2026-10-26: after the drive wiring was redone both drives answered as node
 * 16 / 17 instead of 1 / 2 (console 'p' scan). Kept in the old order:
 * axis1 = 16 (was 1), axis2 = 17 (was 2). */
static const uint8_t s_nodeId[L2DB_AXIS_COUNT] = { 16U, 17U };

/* Wheel mounting sign. The two drive wheels face each other, so the same
 * motor direction moves the robot opposite ways. With this table every
 * command and every feedback value is expressed in robot convention:
 *   positive rpm = that wheel drives the robot FORWARD.
 * Adjustable at runtime with the console 'd' command.                      */
/* 2026-08-25 조립 완료 후 실측: 이 부호로 b/s 지령이 사용자 기준 전진.
 * axis1 = 좌륜, axis2 = 우륜 (axis1 단독 구동 시 우회전으로 확인).      */
static int8_t s_dir[L2DB_AXIS_COUNT] = { -1, +1 };

static L2dbAxisState_t s_axis[L2DB_AXIS_COUNT];

/* -------------------------------------------------------------------------- */
/* SDO helpers                                                                */
/* -------------------------------------------------------------------------- */

static bool L2DB_Write(L2dbAxis_t axis, uint16_t index, uint8_t sub,
                       uint32_t value, uint8_t size)
{
  if (axis >= L2DB_AXIS_COUNT)
  {
    return false;
  }

  SdoResult_t r = CAN_SdoWrite(s_nodeId[axis], index, sub, value, size);
  if (!r.ok)
  {
    if (r.cmd == SDO_RSP_ABORT)
    {
      printf("[L2DB] axis%u write 0x%04X:%02X abort 0x%08lX\r\n",
             (unsigned)(axis + 1U), index, sub, (unsigned long)r.abortCode);
    }
    else
    {
      printf("[L2DB] axis%u write 0x%04X:%02X no response\r\n",
             (unsigned)(axis + 1U), index, sub);
    }
    s_axis[axis].online = false;
    return false;
  }

  s_axis[axis].online   = true;
  s_axis[axis].lastOkMs = api_getCurtMs();
  return true;
}

static bool L2DB_Read(L2dbAxis_t axis, uint16_t index, uint8_t sub, uint32_t *out)
{
  if ((axis >= L2DB_AXIS_COUNT) || (out == NULL))
  {
    return false;
  }

  SdoResult_t r = CAN_SdoRead(s_nodeId[axis], index, sub);
  if (!r.ok)
  {
    s_axis[axis].online = false;
    return false;
  }

  *out = r.value;
  s_axis[axis].online   = true;
  s_axis[axis].lastOkMs = api_getCurtMs();
  return true;
}

/* The manual gives conflicting widths for 0x2FF0:09 (16S in the speed-mode
 * table, 32S in table F1-2). Try 32 bit first and fall back to 16 bit.      */
static bool L2DB_WriteTargetSpeed(L2dbAxis_t axis, int32_t rpm)
{
  SdoResult_t r = CAN_SdoWrite(s_nodeId[axis], OBJ_TARGET_SPEED_RPM,
                               (uint32_t)rpm, 4U);
  if (r.ok)
  {
    return true;
  }
  if (r.cmd == SDO_RSP_ABORT)
  {
    r = CAN_SdoWrite(s_nodeId[axis], OBJ_TARGET_SPEED_RPM,
                     (uint32_t)(int32_t)(int16_t)rpm, 2U);
    if (r.ok)
    {
      return true;
    }
  }

  printf("[L2DB] axis%u target speed write failed (cmd=0x%02X abort=0x%08lX)\r\n",
         (unsigned)(axis + 1U), r.cmd, (unsigned long)r.abortCode);
  s_axis[axis].online = false;
  return false;
}

/* -------------------------------------------------------------------------- */
/* Control                                                                    */
/* -------------------------------------------------------------------------- */

bool L2DB_ClearAlarm(L2dbAxis_t axis)
{
  return L2DB_Write(axis, OBJ_CONTROL_WORD, L2DB_CTRL_CLEAR_ALARM, 2U);
}

bool L2DB_Enable(L2dbAxis_t axis, bool on)
{
  if (axis >= L2DB_AXIS_COUNT)
  {
    return false;
  }

  bool ok = L2DB_Write(axis, OBJ_CONTROL_WORD,
                       on ? L2DB_CTRL_ENABLE : L2DB_CTRL_FREE, 2U);
  if (ok)
  {
    s_axis[axis].enabled = on;
    if (!on)
    {
      s_axis[axis].commandRpm = 0;
    }
  }
  return ok;
}

bool L2DB_SetRamp(L2dbAxis_t axis, float accelRps2, float decelRps2)
{
  uint32_t accDec = CAN_RpsPerSecToDec(accelRps2, L2DB_ENCODER_RESOLUTION);
  uint32_t decDec = CAN_RpsPerSecToDec(decelRps2, L2DB_ENCODER_RESOLUTION);

  bool ok = L2DB_Write(axis, OBJ_PROFILE_ACCEL, accDec, 4U);
  ok = L2DB_Write(axis, OBJ_PROFILE_DECEL, decDec, 4U) && ok;
  return ok;
}

bool L2DB_SetSpeedRpm(L2dbAxis_t axis, int32_t rpm)
{
  if (axis >= L2DB_AXIS_COUNT)
  {
    return false;
  }

  if (rpm >  L2DB_MAX_RPM) { rpm =  L2DB_MAX_RPM; }
  if (rpm < -L2DB_MAX_RPM) { rpm = -L2DB_MAX_RPM; }

  if (!s_axis[axis].enabled && (rpm != 0))
  {
    printf("[L2DB] axis%u not enabled - run 'i' first\r\n", (unsigned)(axis + 1U));
    return false;
  }

  /* rpm is in robot convention; the drive wants its own motor convention. */
  if (!L2DB_WriteTargetSpeed(axis, rpm * s_dir[axis]))
  {
    return false;
  }

  s_axis[axis].commandRpm = rpm;
  return true;
}

/* 2026-09-28: fractional speed through 0x60FF. The integer-rpm object left
 * steering at 1 rpm steps (~0.1 rad/s of yaw) and the PC had to dither,
 * which made the robot weave +-2-3 deg between rack legs. */
bool L2DB_SetSpeedCentiRpm(L2dbAxis_t axis, int32_t centiRpm)
{
  if (axis >= L2DB_AXIS_COUNT)
  {
    return false;
  }

  const int32_t lim = (int32_t)L2DB_MAX_RPM * 100;
  if (centiRpm >  lim) { centiRpm =  lim; }
  if (centiRpm < -lim) { centiRpm = -lim; }

  if (!s_axis[axis].enabled && (centiRpm != 0))
  {
    printf("[L2DB] axis%u not enabled - run 'i' first\r\n", (unsigned)(axis + 1U));
    return false;
  }

  int64_t dec = ((int64_t)centiRpm * s_dir[axis] * 512
                 * (int64_t)L2DB_ENCODER_RESOLUTION) / (1875 * 100);
  SdoResult_t r = CAN_SdoWrite(s_nodeId[axis], OBJ_TARGET_VELOCITY,
                               (uint32_t)(int32_t)dec, 4U);
  if (!r.ok)
  {
    printf("[L2DB] axis%u target velocity write failed (cmd=0x%02X abort=0x%08lX)\r\n",
           (unsigned)(axis + 1U), r.cmd, (unsigned long)r.abortCode);
    s_axis[axis].online = false;
    return false;
  }

  /* status line reports whole rpm; round half away from zero */
  s_axis[axis].commandRpm = (centiRpm >= 0) ? ((centiRpm + 50) / 100)
                                            : -((-centiRpm + 50) / 100);
  return true;
}

bool L2DB_AxisInit(L2dbAxis_t axis)
{
  if (axis >= L2DB_AXIS_COUNT)
  {
    return false;
  }

  printf("[L2DB] axis%u (node %u) init...\r\n",
         (unsigned)(axis + 1U), (unsigned)s_nodeId[axis]);

  uint32_t probe = 0U;
  if (!L2DB_Read(axis, OBJ_STATUS_WORD, &probe))
  {
    printf("[L2DB] axis%u offline\r\n", (unsigned)(axis + 1U));
    return false;
  }

  /* An alarm latches the drive; clear it before anything else. */
  if ((probe & 0x0008U) != 0U)
  {
    uint32_t err = 0U;
    (void)L2DB_Read(axis, OBJ_ERROR_CODE, &err);
    printf("[L2DB] axis%u alarm 0x%04lX, clearing\r\n",
           (unsigned)(axis + 1U), (unsigned long)(err & 0xFFFFU));
    L2DB_DecodeError((uint16_t)err);
    (void)L2DB_ClearAlarm(axis);
    HAL_Delay(10);
  }

  bool ok = true;
  /* Step 1: ramps.  Step 2: mode 3.  Step 3: target 0.  Step 4: enable. */
  ok = L2DB_SetRamp(axis, L2DB_DEFAULT_ACCEL_RPS2, L2DB_DEFAULT_DECEL_RPS2) && ok;
  ok = L2DB_Write(axis, OBJ_MODE_OF_OPERATION, (uint32_t)L2DB_MODE_SPEED_RAMP, 1U) && ok;
  ok = L2DB_WriteTargetSpeed(axis, 0) && ok;
  if (!(L2DB_Write(axis, OBJ_SPEED_LOOP_KP, L2DB_SPEED_KP, 2U)
        && L2DB_Write(axis, OBJ_SPEED_LOOP_KI, L2DB_SPEED_KI, 2U)))
  {
    /* Not fatal: the drive keeps its saved gains and still runs. */
    printf("[L2DB] axis%u speed loop gains WRITE-FAIL\r\n", (unsigned)(axis + 1U));
  }
  ok = L2DB_Enable(axis, true) && ok;

  uint32_t mode = 0U;
  if (L2DB_Read(axis, OBJ_MODE_DISPLAY, &mode))
  {
    printf("[L2DB] axis%u active mode = %d %s\r\n",
           (unsigned)(axis + 1U), (int)(int8_t)mode,
           ((int8_t)mode == L2DB_MODE_SPEED_RAMP) ? "(speed)" : "(unexpected)");
  }

  printf("[L2DB] axis%u init %s\r\n",
         (unsigned)(axis + 1U), ok ? "OK" : "FAILED");
  return ok;
}

bool L2DB_InitAll(void)
{
  bool ok = true;
  for (uint8_t a = 0U; a < L2DB_AXIS_COUNT; ++a)
  {
    ok = L2DB_AxisInit((L2dbAxis_t)a) && ok;
  }
  return ok;
}

void L2DB_StopAll(void)
{
  Lift_Stop();                     /* x, z and boot also stop the lift */
  for (uint8_t a = 0U; a < L2DB_AXIS_COUNT; ++a)
  {
    (void)L2DB_WriteTargetSpeed((L2dbAxis_t)a, 0);
    s_axis[a].commandRpm = 0;
  }
  printf("[L2DB] all axes -> 0 rpm\r\n");
}

void L2DB_ReleaseAll(void)
{
  L2DB_StopAll();
  for (uint8_t a = 0U; a < L2DB_AXIS_COUNT; ++a)
  {
    (void)L2DB_Enable((L2dbAxis_t)a, false);
  }
  printf("[L2DB] all axes released (shaft free)\r\n");
}

bool L2DB_ConfigCommLossStop(uint32_t delayMs)
{
  printf("[L2DB] enabling comm-loss shutdown (%lu ms) on both axes\r\n",
         (unsigned long)delayMs);

  bool ok = true;
  for (uint8_t a = 0U; a < L2DB_AXIS_COUNT; ++a)
  {
    L2dbAxis_t ax = (L2dbAxis_t)a;
    ok = L2DB_Write(ax, OBJ_COMMLOSS_STOP_EN, 1U, 1U) && ok;
    ok = L2DB_Write(ax, OBJ_COMMLOSS_STOP_DELAY, delayMs, 4U) && ok;
    /* 0x4100:xx belongs to parameter group S1 */
    ok = L2DB_Write(ax, OBJ_SAVE_GROUP_S1, 1U, 1U) && ok;
    HAL_Delay(50);
  }

  printf("[L2DB] comm-loss config %s - power cycle the drive to apply\r\n",
         ok ? "saved" : "FAILED");
  return ok;
}

/* -------------------------------------------------------------------------- */
/* Feedback                                                                   */
/* -------------------------------------------------------------------------- */

void L2DB_PollFeedback(void)
{
  static uint8_t step = 0U;
  uint32_t v = 0U;

  /* Positions: both axes every call, back to back.
   *
   * Odometry derives heading from the DIFFERENCE of the two positions, so if
   * they come from different instants that time skew turns into phantom
   * rotation. The old 4-object round robin read axis1 position at step 1 and
   * axis2 at step 5 - always 40 ms apart - and refreshed each axis only every
   * 80 ms, while ROS polls every 50 ms. At 0.15 m/s that 40 ms skew is 6 mm,
   * which over the 179 mm track is 1.9 deg, and the 80 ms staleness adds up to
   * 3.8 deg more. Measured on the ROS side: wheel heading jittered at rms
   * 5 deg while the gyro showed 0.2 deg (2026-09-11, tools/odom_smooth.py).
   *
   * An offline axis is skipped so one dead drive cannot stall the 10 ms main
   * loop with repeated SDO timeouts; the round robin below still probes it and
   * L2DB_Read flips it back online as soon as it answers.                   */
  for (uint8_t a = 0U; a < L2DB_AXIS_COUNT; ++a)
  {
    if (!s_axis[a].online)
    {
      continue;
    }
    if (L2DB_Read((L2dbAxis_t)a, OBJ_ACTUAL_POSITION, &v))
    {
      s_axis[a].actualPosition = (int32_t)v * s_dir[a];
    }
  }

  /* Everything else may lag. 4 objects x 2 axes = 80 ms per axis. */
  L2dbAxis_t axis = (L2dbAxis_t)(step / 4U);
  uint8_t    what = step % 4U;

  step = (uint8_t)((step + 1U) % (4U * L2DB_AXIS_COUNT));

  switch (what)
  {
    case 0U:
      if (L2DB_Read(axis, OBJ_ACTUAL_SPEED_RPM, &v))
      {
        s_axis[axis].actualRpm = (int16_t)((int16_t)v * s_dir[axis]);
      }
      break;
    case 1U:
      if (L2DB_Read(axis, OBJ_STATUS_WORD, &v))
      {
        s_axis[axis].statusWord = (uint16_t)v;
      }
      break;
    case 2U:
      /* 2026-09-29: motor current, to see what a caster kick costs the drive */
      if (L2DB_Read(axis, OBJ_ACTUAL_CURRENT_IQ, &v))
      {
        s_axis[axis].iqRaw = (int16_t)((int16_t)v * s_dir[axis]);
      }
      break;
    default:
      if (L2DB_Read(axis, OBJ_ERROR_CODE, &v))
      {
        s_axis[axis].errorCode = (uint16_t)v;
      }
      break;
  }
}

static const char *const s_errBit[16] =
{
  "internal error",
  "encoder ABZ signal",
  "encoder UVW signal",
  "encoder count",
  "driver over-temperature",
  "bus over-voltage",
  "bus under-voltage",
  "output short circuit",
  "brake resistor fault",
  "following error too large",
  NULL,
  "I2T overload (driver or motor)",
  NULL,
  NULL,
  "motor search failed (serial encoder / motor not connected)",
  NULL
};

void L2DB_DecodeError(uint16_t errorCode)
{
  if (errorCode == 0U)
  {
    return;
  }
  for (uint8_t b = 0U; b < 16U; ++b)
  {
    if (((errorCode >> b) & 1U) != 0U)
    {
      printf("           bit%-2u : %s\r\n", (unsigned)b,
             (s_errBit[b] != NULL) ? s_errBit[b] : "reserved");
    }
  }
}

/* 'q': current limits and speed-loop gains as configured in each drive. */
void L2DB_PrintDriveLimits(void)
{
  for (uint8_t a = 0U; a < L2DB_AXIS_COUNT; ++a)
  {
    uint32_t lim = 0U, ovl = 0U, mx = 0U, kp = 0U, ki = 0U, iq = 0U;
    bool ok = L2DB_Read((L2dbAxis_t)a, OBJ_CURRENT_LIMIT, &lim)
            & L2DB_Read((L2dbAxis_t)a, OBJ_MOTOR_OVERLOAD_A, &ovl)
            & L2DB_Read((L2dbAxis_t)a, OBJ_MOTOR_MAX_A, &mx)
            & L2DB_Read((L2dbAxis_t)a, OBJ_SPEED_LOOP_KP, &kp)
            & L2DB_Read((L2dbAxis_t)a, OBJ_SPEED_LOOP_KI, &ki)
            & L2DB_Read((L2dbAxis_t)a, OBJ_ACTUAL_CURRENT_IQ, &iq);
    const float k = 50.0f / (1.414f * 2048.0f);
    printf("[DRIVE] axis%u %s limit=%.1fArms(raw %lu) motor_max=%.1fA "
           "overload=%.1fA speed_kp=%lu speed_ki=%lu iq=%+.2fA\r\n",
           (unsigned)(a + 1U), ok ? "ok" : "READ-FAIL",
           (double)((float)(uint16_t)lim * k), (unsigned long)(uint16_t)lim,
           (double)((float)(uint16_t)mx / 10.0f),
           (double)((float)(uint16_t)ovl / 10.0f),
           (unsigned long)(uint16_t)kp, (unsigned long)(uint16_t)ki,
           (double)((float)(int16_t)iq * k));
    uint32_t pkp = 0U, pff = 0U, few = 0U;
    bool ok2 = L2DB_Read((L2dbAxis_t)a, OBJ_POS_LOOP_KP, &pkp)
             & L2DB_Read((L2dbAxis_t)a, OBJ_POS_LOOP_FF, &pff)
             & L2DB_Read((L2dbAxis_t)a, OBJ_FOLLOW_ERR_WINDOW, &few);
    printf("[DRIVE] axis%u %s pos_kp=%d pos_ff=%d follow_win=%lu\r\n",
           (unsigned)(a + 1U), ok2 ? "ok" : "READ-FAIL",
           (int)(int16_t)pkp, (int)(int16_t)pff, (unsigned long)few);
  }
}

void L2DB_PrintFeedback(void)
{
  for (uint8_t a = 0U; a < L2DB_AXIS_COUNT; ++a)
  {
    uint32_t volt = 0U;
    if (L2DB_Read((L2dbAxis_t)a, OBJ_BUS_VOLTAGE, &volt))
    {
      s_axis[a].busVoltage = (int16_t)volt;
    }

    printf("axis%u node%u %-7s %-8s cmd=%4ld rpm  act=%4d rpm  pos=%10ld  "
           "status=0x%04X err=0x%04X  %dV iq=%+.2fA\r\n",
           (unsigned)(a + 1U), (unsigned)s_nodeId[a],
           s_axis[a].online  ? "online"  : "OFFLINE",
           s_axis[a].enabled ? "enabled" : "free",
           (long)s_axis[a].commandRpm,
           (int)s_axis[a].actualRpm,
           (long)s_axis[a].actualPosition,
           (unsigned)s_axis[a].statusWord,
           (unsigned)s_axis[a].errorCode,
           (int)s_axis[a].busVoltage,
           (double)((float)s_axis[a].iqRaw * 50.0f / (1.414f * 2048.0f)));
    L2DB_DecodeError(s_axis[a].errorCode);
  }
}

const L2dbAxisState_t *L2DB_GetState(L2dbAxis_t axis)
{
  return (axis < L2DB_AXIS_COUNT) ? &s_axis[axis] : NULL;
}

/* -------------------------------------------------------------------------- */
/* Test console on USART1                                                     */
/* -------------------------------------------------------------------------- */

void L2DB_ConsoleHelp(void)
{
  printf(
    "\r\n"
    "  ---- L2DB4830 test console -------------------------------------\r\n"
    "   i            initialise both axes (clear alarm, mode 3, enable)\r\n"
    "   i <n>        initialise axis n only            (n = 1 or 2)\r\n"
    "   s <n> <rpm>  set axis n speed, e.g.  s 1 60    (range +/-%d)\r\n"
    "   w <c1> <c2>  both axes at once, centi-rpm (motion cmds need #sum or %%crc16)\r\n"
    "   b <rpm>      both axes same speed  (straight)\r\n"
    "   t <rpm>      turn in place: axis1 +rpm, axis2 -rpm\r\n"
    "   x            STOP - both axes to 0 rpm\r\n"
    "   z            release - shafts free (control word 0x06)\r\n"
    "   f            print feedback\r\n"
    "   a            clear alarms on both axes\r\n"
    "   r <acc> <dec>  ramp in rps/s, e.g.  r 1.0 1.0\r\n"
    "   g <n> <kp> <ff>  position loop in speed mode (RAM), n 0=both\r\n"
    "   d            show wheel direction signs\r\n"
    "   d <n> <+-1>  set axis n direction (mirror-mounted wheels)\r\n"
    "   c            enable comm-loss shutdown + save (reboot drive after)\r\n"
    "   l up|down [ms]  lift: PC0 / PE1 HIGH, LOW after travel time; l stop; l t <up> <down>\r\n"
    "   n <kbps> [p] [a]  CAN2 probe: p 0=PB12/13 1=PB5/6, a=ACK; n=table, n 0=off\r\n"
    "   boot         enter DFU bootloader (no BOOT0 jumper needed)\r\n"
    "   p            scan CAN1 node ids 1..127 (drives not answering?)\r\n"
    "   h            this help\r\n"
    "  ----------------------------------------------------------------\r\n",
    L2DB_MAX_RPM);
}

/* Jump into the STM32H743 system bootloader so the board shows up as
 * 0483:df11 without moving the BOOT0 jumper. Best effort - if the host does
 * not enumerate DFU within a few seconds, BOOT0 = 1 + RESET still works.   */
void L2DB_JumpToBootloader(void)
{
  const uint32_t bootAddr = 0x1FF09800UL;   /* AN2606, STM32H74x/75x */

  printf("[SYS] entering system bootloader (DFU)...\r\n");
  printf("[SYS] run 'make dfu', then RESET to come back.\r\n");
  HAL_Delay(80);                            /* let the UART drain */

  /* Force a USB detach so the host re-enumerates the bootloader. */
  USBD_Stop(&hUsbDeviceFS);
  USBD_DeInit(&hUsbDeviceFS);
  HAL_Delay(120);

  __disable_irq();
  HAL_RCC_DeInit();
  HAL_DeInit();

  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL  = 0U;
  for (uint8_t i = 0U; i < 8U; ++i)
  {
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }

  SCB->VTOR = bootAddr;
  __set_MSP(*(volatile uint32_t *)bootAddr);
  __enable_irq();

  ((void (*)(void))(*(volatile uint32_t *)(bootAddr + 4U)))();

  while (1) { }                             /* not reached */
}

/* Optional line checksum: "<payload> #<sum>", sum = byte sum of <payload>
 * (trailing spaces excluded) mod 256. Lines without '#' are accepted as
 * before so the console stays usable by hand.
 *
 * 2026-09-28: lost bytes merged two speed lines into one ("s 1 -13" +
 * "14 #211" -> "s 1 -1314"). The merged line carries the second line's sum,
 * which does not match its payload, so it is dropped instead of spinning a
 * wheel at 400 rpm. No speed limit is imposed. */
static uint32_t s_lineRejected = 0U;

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over [p, end). */
static uint16_t L2DB_Crc16(const char *p, const char *end)
{
  uint16_t crc = 0xFFFFU;
  for (; p < end; p++)
  {
    crc ^= (uint16_t)((uint8_t)*p << 8);
    for (int b = 0; b < 8; b++)
    {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/* Set by L2DB_LineChecksumOk: the line carried a check value and it matched.
 * Lines typed by hand carry none and are still accepted, except motion
 * commands (see L2DB_ConsoleExecLine). */
static bool s_lineVerified;

/* Accepts "<cmd> %XXXX" (CRC-16, hex) or the older "<cmd> #<byte sum>".
 * 2026-09-30: the 1-byte sum let about 1 in 256 corrupted lines through, and
 * with ~80 dropped lines a minute a corrupted "v 2 ..." set the right drive
 * to -262 rpm for a moment. The CRC misses about 1 in 65536. */
static bool L2DB_LineChecksumOk(char *line)
{
  s_lineVerified = false;
  char *pct = strrchr(line, '%');
  if (pct != NULL)
  {
    char *end = pct;
    while ((end > line) && ((end[-1] == ' ') || (end[-1] == '\t')))
    {
      end--;
    }
    char *stop = NULL;
    unsigned long want = strtoul(pct + 1, &stop, 16);
    if ((stop != pct + 5) || (want != L2DB_Crc16(line, end)))
    {
      s_lineRejected++;
      printf("[L2DB] checksum mismatch - line dropped (%lu)\r\n",
             (unsigned long)s_lineRejected);
      return false;
    }
    *end = '\0';
    s_lineVerified = true;
    return true;
  }
  char *hash = strchr(line, '#');
  if (hash == NULL)
  {
    return true;
  }
  char *end = hash;
  while ((end > line) && ((end[-1] == ' ') || (end[-1] == '\t')))
  {
    end--;
  }
  unsigned int sum = 0U;
  for (const char *p = line; p < end; p++)
  {
    sum += (unsigned char)*p;
  }
  char *stop = NULL;
  long want = strtol(hash + 1, &stop, 10);
  if ((stop == hash + 1) || (want != (long)(sum & 0xFFU)))
  {
    s_lineRejected++;
    printf("[L2DB] checksum mismatch - line dropped (%lu)\r\n",
           (unsigned long)s_lineRejected);
    return false;
  }
  *end = '\0';
  s_lineVerified = true;
  return true;
}

void L2DB_ConsoleExecLine(char *line)
{
  while ((*line == ' ') || (*line == '\t'))
  {
    line++;
  }
  if (*line == '\0')
  {
    return;
  }
  if (!L2DB_LineChecksumOk(line))
  {
    return;
  }
  /* Motion commands must carry a check value. A corrupted line can lose its
   * whole " #.." / " %...." tail and would otherwise run unchecked. Stop (x)
   * and release (z) stay accepted without one. */
  if ((strchr("vswbtl", *line) != NULL) && (*line != '\0') && !s_lineVerified
      && (strncmp(line, "boot", 4) != 0))
  {
    s_lineRejected++;
    printf("[L2DB] unverified motion line dropped - checksum mismatch (%lu)\r\n",
           (unsigned long)s_lineRejected);
    return;
  }

  /* Word commands are checked before the single-letter dispatch below. */
  if (strncmp(line, "boot", 4) == 0)
  {
    L2DB_StopAll();
    L2DB_JumpToBootloader();
    return;
  }

  const char cmd = *line;
  char *args = line + 1;

  switch (cmd)
  {
    case 'h':
    case '?':
      L2DB_ConsoleHelp();
      break;

    case 'l':                /* l | l up|down [ms] | l stop | l t <up> <down> : lift */
      Lift_ConsoleCmd(args);
      break;

    case 'n':                /* n | n <kbps> [pins 0|1] [a] | n 0 : CAN2 probe */
      CAN2_ConsoleCmd(args);
      break;

    case 'u':
      L2DB_UartStats();
      break;

    case 'q':
      L2DB_PrintDriveLimits();
      CAN_PrintDriveBusLine();
      break;

    case 'p':                /* p : scan CANopen node ids 1..127 (robot idle) */
      CAN_ScanNodes();
      CAN_PrintDriveBusLine();
      break;

    case 'k':                /* k <axis 0=both|1|2> <speed kp> <speed ki> */
    {
      /* 2026-09-29: the drives run the speed loop at kp 35 / ki 8 (factory
       * 10 / 64). A weak integral lets a caster pushing one wheel sag its
       * speed and turn the robot. Written to RAM only; init writes
       * L2DB_SPEED_KP/KI on every boot, 'k 0 35 8' restores the drive's own. */
      int n = -1, kp = -1, ki = -1;
      if ((sscanf(args, "%d %d %d", &n, &kp, &ki) == 3) && (n >= 0) && (n <= 2)
          && (kp >= 0) && (kp <= 1000) && (ki >= 0) && (ki <= 1000))
      {
        for (uint8_t a = 0U; a < L2DB_AXIS_COUNT; ++a)
        {
          if ((n != 0) && (a != (uint8_t)(n - 1)))
          {
            continue;
          }
          bool ok = L2DB_Write((L2dbAxis_t)a, OBJ_SPEED_LOOP_KP, (uint32_t)kp, 2U)
                  & L2DB_Write((L2dbAxis_t)a, OBJ_SPEED_LOOP_KI, (uint32_t)ki, 2U);
          printf("[DRIVE] axis%u speed loop set kp=%d ki=%d %s\r\n",
                 (unsigned)(a + 1U), kp, ki, ok ? "ok" : "WRITE-FAIL");
        }
        L2DB_PrintDriveLimits();
      }
      else
      {
        printf("[L2DB] usage: k <0|1|2> <kp 0..1000> <ki 0..1000>\r\n");
      }
      break;
    }

    case 'g':                /* g <axis 0=both|1|2> <pos kp> <pos ff 0..256> */
    {
      /* 2026-11-16: speed mode 3 still runs a position loop (manual 6.2). The
       * ~1 Hz wheel-speed pulsation did not move with speed kp/ki (35..150 /
       * 16..128), so try this loop. RAM only - reboot restores the drive's own. */
      int n = -1, pkp = -1, pff = -1;
      if ((sscanf(args, "%d %d %d", &n, &pkp, &pff) == 3) && (n >= 0) && (n <= 2)
          && (pkp >= 0) && (pkp <= 1000) && (pff >= 0) && (pff <= 256))
      {
        for (uint8_t a = 0U; a < L2DB_AXIS_COUNT; ++a)
        {
          if ((n != 0) && (a != (uint8_t)(n - 1)))
          {
            continue;
          }
          bool ok = L2DB_Write((L2dbAxis_t)a, OBJ_POS_LOOP_KP, (uint32_t)pkp, 2U)
                  & L2DB_Write((L2dbAxis_t)a, OBJ_POS_LOOP_FF, (uint32_t)pff, 2U);
          printf("[DRIVE] axis%u position loop set kp=%d ff=%d %s\r\n",
                 (unsigned)(a + 1U), pkp, pff, ok ? "ok" : "WRITE-FAIL");
        }
        L2DB_PrintDriveLimits();
      }
      else
      {
        printf("[L2DB] usage: g <0|1|2> <pos kp 0..1000> <pos ff 0..256>\r\n");
      }
      break;
    }

    case 'w':              /* w <axis1 centi-rpm> <axis2 centi-rpm> : both at once */
    {
      /* 2026-09-30: sending the two wheels as two lines let one line (mostly
       * axis 2, which arrives while axis 1 is being written) drop alone, so
       * one wheel kept its old speed for up to 0.2 s. One line = both or neither. */
      int c1 = 0, c2 = 0;
      if (sscanf(args, "%d %d", &c1, &c2) == 2)
      {
        (void)L2DB_SetSpeedCentiRpm(L2DB_AXIS_1, c1);
        (void)L2DB_SetSpeedCentiRpm(L2DB_AXIS_2, c2);
      }
      else
      {
        printf("[L2DB] usage: w <axis1 centi-rpm> <axis2 centi-rpm>\r\n");
      }
      break;
    }

    case 'v':                       /* v <axis> <centi-rpm>, e.g. v 1 -1325 */
    {
      int n = 0, crpm = 0;
      if ((sscanf(args, "%d %d", &n, &crpm) == 2) && ((n == 1) || (n == 2)))
      {
        (void)L2DB_SetSpeedCentiRpm((L2dbAxis_t)(n - 1), crpm);
      }
      else
      {
        printf("[L2DB] usage: v <1|2> <centi-rpm>\r\n");
      }
      break;
    }

    case 'i':
    {
      int n = atoi(args);
      if ((n == 1) || (n == 2))
      {
        (void)L2DB_AxisInit((L2dbAxis_t)(n - 1));
      }
      else
      {
        (void)L2DB_InitAll();
      }
      break;
    }

    case 's':
    {
      int n = 0, rpm = 0;
      if (sscanf(args, "%d %d", &n, &rpm) == 2)
      {
        if ((n == 1) || (n == 2))
        {
          if (L2DB_SetSpeedRpm((L2dbAxis_t)(n - 1), rpm))
          {
            printf("[L2DB] axis%d -> %d rpm\r\n", n, rpm);
          }
        }
        else
        {
          printf("[L2DB] axis must be 1 or 2\r\n");
        }
      }
      else
      {
        printf("[L2DB] usage: s <axis> <rpm>\r\n");
      }
      break;
    }

    case 'b':
    {
      int rpm = atoi(args);
      (void)L2DB_SetSpeedRpm(L2DB_AXIS_1, rpm);
      (void)L2DB_SetSpeedRpm(L2DB_AXIS_2, rpm);
      printf("[L2DB] both -> %d rpm\r\n", rpm);
      break;
    }

    case 't':
    {
      int rpm = atoi(args);
      (void)L2DB_SetSpeedRpm(L2DB_AXIS_1,  rpm);
      (void)L2DB_SetSpeedRpm(L2DB_AXIS_2, -rpm);
      printf("[L2DB] spin: axis1 %+d, axis2 %+d rpm\r\n", rpm, -rpm);
      break;
    }

    case 'x':
      L2DB_StopAll();
      break;

    case 'z':
      L2DB_ReleaseAll();
      break;

    case 'f':
      L2DB_PrintFeedback();
      break;

    case 'a':
      for (uint8_t a = 0U; a < L2DB_AXIS_COUNT; ++a)
      {
        (void)L2DB_ClearAlarm((L2dbAxis_t)a);
      }
      printf("[L2DB] alarms cleared\r\n");
      break;

    case 'r':
    {
      float acc = 0.0f, dec = 0.0f;
      if (sscanf(args, "%f %f", &acc, &dec) == 2)
      {
        for (uint8_t a = 0U; a < L2DB_AXIS_COUNT; ++a)
        {
          (void)L2DB_SetRamp((L2dbAxis_t)a, acc, dec);
        }
        printf("[L2DB] ramp %.2f / %.2f rps/s\r\n", acc, dec);
      }
      else
      {
        printf("[L2DB] usage: r <accel> <decel>\r\n");
      }
      break;
    }

    case 'd':
    {
      int n = 0, d = 0;
      if (sscanf(args, "%d %d", &n, &d) == 2)
      {
        if (((n == 1) || (n == 2)) && ((d == 1) || (d == -1)))
        {
          s_dir[n - 1] = (int8_t)d;
          printf("[L2DB] axis%d direction = %+d\r\n", n, d);
        }
        else
        {
          printf("[L2DB] usage: d <axis 1|2> <dir 1|-1>\r\n");
        }
      }
      else
      {
        printf("[L2DB] direction: axis1 %+d, axis2 %+d\r\n",
               (int)s_dir[0], (int)s_dir[1]);
      }
      break;
    }

    case 'c':
      (void)L2DB_ConfigCommLossStop(300U);
      break;

    default:
      printf("[L2DB] unknown '%c' - press h for help\r\n", cmd);
      break;
  }
}

/* ---- USART1 receive, interrupt driven -----------------------------------
 *
 * This used to poll RDR once per 10 ms main-loop tick. USART1 runs with its
 * FIFO disabled, so exactly ONE byte fits in the peripheral between polls -
 * but at 115200 a byte lands every 87 us. Hand typing survived that; any
 * program sending "f\r\n" as one burst lost two of the three bytes, so
 * commands piled up as "ffffff..." with the terminators missing.
 *
 * An RXNE interrupt with a ring buffer removes the constraint entirely.    */
#define UART_RX_RING_SIZE   256U

static volatile uint8_t  s_rxRing[UART_RX_RING_SIZE];
static volatile uint16_t s_rxHead    = 0U;
static volatile uint16_t s_rxTail    = 0U;
static volatile uint32_t s_rxDropped = 0U;
static volatile uint32_t s_rxOverrun = 0U;   /* HW overrun: bytes lost before the ISR ran */
static volatile uint32_t s_rxFraming = 0U;   /* framing error: bad stop bit (noise/baud) */
static volatile uint32_t s_rxNoise   = 0U;   /* noise flag on a sampled bit */
static volatile uint32_t s_rxParity  = 0U;
static volatile uint32_t s_rxBytes   = 0U;

volatile uint32_t g_probeTim6Last = 0U;
volatile uint32_t g_probeTim6GapMax = 0U;
volatile uint32_t g_probeOtgMax = 0U;
volatile uint32_t g_probeU2Max = 0U;
volatile uint32_t g_probeTickMax = 0U;
volatile uint32_t g_probeU1Max = 0U;
volatile uint32_t g_probeU1BurstMax = 0U;

void ISR_ProbeInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = 0xC5ACCE55U;             /* Cortex-M7 needs the lock opened */
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ---- console TX ring (printf output) ---------------------------------- */
#define CON_TX_SIZE 2048U
static volatile uint8_t  s_txRing[CON_TX_SIZE];
static volatile uint16_t s_txHead;      /* written by printf (main)        */
static volatile uint16_t s_txTail;      /* advanced by the USART1 interrupt */
static volatile uint32_t s_txFullWaits; /* times the ring was full          */

static void L2DB_TxPushOne(void)
{
  /* One byte from the ring into the TX FIFO, if there is room. */
  if ((s_txTail != s_txHead) && ((USART1->ISR & USART_ISR_TXE_TXFNF) != 0U))
  {
    USART1->TDR = s_txRing[s_txTail];
    s_txTail = (uint16_t)((s_txTail + 1U) % CON_TX_SIZE);
  }
}

void L2DB_ConsolePutc(uint8_t c)
{
  uint16_t next = (uint16_t)((s_txHead + 1U) % CON_TX_SIZE);
  if (next == s_txTail)
  {
    /* Full (long help text). printf may hold the stdio lock with interrupts
     * off, so the TX interrupt cannot drain it - push bytes out by polling. */
    s_txFullWaits++;
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    while (next == s_txTail)
    {
      L2DB_TxPushOne();
    }
    __set_PRIMASK(prim);
  }
  s_txRing[s_txHead] = c;
  s_txHead = next;
  ATOMIC_SET_BIT(USART1->CR1, USART_CR1_TXEIE_TXFNFIE);
}

void L2DB_ConsoleFlush(uint32_t timeoutMs)
{
  uint32_t t0 = HAL_GetTick();
  while ((s_txTail != s_txHead) && ((HAL_GetTick() - t0) < timeoutMs))
  {
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    L2DB_TxPushOne();
    __set_PRIMASK(prim);
  }
}

static void L2DB_ConsoleTxIsr(void)
{
  USART_TypeDef *u = USART1;
  if (((u->CR1 & USART_CR1_TXEIE_TXFNFIE) == 0U)
      || ((u->ISR & USART_ISR_TXE_TXFNF) == 0U))
  {
    return;
  }
  while ((s_txTail != s_txHead) && ((u->ISR & USART_ISR_TXE_TXFNF) != 0U))
  {
    u->TDR = s_txRing[s_txTail];
    s_txTail = (uint16_t)((s_txTail + 1U) % CON_TX_SIZE);
  }
  if (s_txTail == s_txHead)
  {
    ATOMIC_CLEAR_BIT(u->CR1, USART_CR1_TXEIE_TXFNFIE);
  }
}

void USART1_IRQHandler(void)
{
  USART_TypeDef *u = USART1;
  uint32_t probeT0 = probe_now();
  uint32_t burst = 0U;

  if ((u->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0U)
  {
    /* 2026-09-28: overrun used to be cleared silently. Bytes lost here merged
     * "s 1 -13" and "s 2 -14" into "s 1 -1314" (clamped to 400 rpm) and the
     * robot lurched. Count it so it shows up in L2DB_UartReport. */
    uint32_t isr = u->ISR;
    if ((isr & USART_ISR_ORE) != 0U) { s_rxOverrun++; }
    if ((isr & USART_ISR_FE)  != 0U) { s_rxFraming++; }
    if ((isr & USART_ISR_NE)  != 0U) { s_rxNoise++; }
    if ((isr & USART_ISR_PE)  != 0U) { s_rxParity++; }
    u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
  }

  while ((u->ISR & USART_ISR_RXNE_RXFNE) != 0U)
  {
    uint8_t  c    = (uint8_t)(u->RDR & 0xFFU);
    uint16_t next = (uint16_t)((s_rxHead + 1U) % UART_RX_RING_SIZE);

    if (next == s_rxTail)
    {
      s_rxDropped++;          /* ring full - drop the newest byte */
    }
    else
    {
      s_rxRing[s_rxHead] = c;
      s_rxHead = next;
      s_rxBytes++;
    }
    burst++;
  }
  L2DB_ConsoleTxIsr();
  probe_max(&g_probeU1BurstMax, burst);
  probe_max(&g_probeU1Max, probe_now() - probeT0);
}

void L2DB_UartRxInit(void)
{
  USART_TypeDef *u = USART1;

  s_rxHead = 0U;
  s_rxTail = 0U;
  s_rxDropped = 0U;
  s_rxOverrun = 0U;
  ISR_ProbeInit();

  /* 2026-09-28: enable the 16-byte RX FIFO. This IRQ sits below TIM6, so a
   * long tick overran the single-byte RX register and dropped 6-7 bytes.
   * The FIFO rides out ~1.4 ms at 115200 without touching priorities. */
  (void)HAL_UARTEx_EnableFifoMode(&huart1);

  (void)u->RDR;               /* discard whatever is sitting there */
  u->ICR  = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
  u->CR1 |= USART_CR1_RXNEIE_RXFNEIE;

  /* 2026-09-28: was (6,0), below TIM6 (5,0). TIM6 actually fires every 10 us
   * (100 kHz), so this IRQ kept getting pushed back and the 'u' stats showed
   * 315 overruns (framing/noise 0) during one docking run - byte loss, not
   * line noise. This handler only moves bytes into the ring, so running it
   * above TIM6 and USART2 (3) costs them next to nothing. USB stays at 0. */
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static bool L2DB_UartGetChar(uint8_t *c)
{
  if (s_rxTail == s_rxHead)
  {
    return false;
  }
  *c = s_rxRing[s_rxTail];
  s_rxTail = (uint16_t)((s_rxTail + 1U) % UART_RX_RING_SIZE);
  return true;
}

/* 2026-09-28: 'u' console command. ~2.4% of command lines failed the
 * checksum; this tells byte loss (overrun) apart from line noise (framing/
 * noise) without a reboot. */
void L2DB_UartStats(void)
{
  printf("[UART1] fifo=%lu bytes=%lu overrun=%lu framing=%lu noise=%lu parity=%lu "
         "ringfull=%lu cksum_drop=%lu\r\n",
         (unsigned long)((USART1->CR1 & USART_CR1_FIFOEN) != 0U),
         (unsigned long)s_rxBytes, (unsigned long)s_rxOverrun,
         (unsigned long)s_rxFraming, (unsigned long)s_rxNoise,
         (unsigned long)s_rxParity, (unsigned long)s_rxDropped,
         (unsigned long)s_lineRejected);
  /* ISR probe: microseconds, maxima since the last 'u'. tim6gap is the
   * longest stretch in which the 10 us tick could not run. */
  const uint32_t cpu = SystemCoreClock / 1000000U;
  printf("[UART1] txfull=%lu\r\n", (unsigned long)s_txFullWaits);
  printf("[UART1] probe tim6gap=%luus otg=%luus usart2=%luus systick=%luus "
         "usart1=%luus burst=%lu\r\n",
         (unsigned long)(g_probeTim6GapMax / cpu),
         (unsigned long)(g_probeOtgMax / cpu),
         (unsigned long)(g_probeU2Max / cpu),
         (unsigned long)(g_probeTickMax / cpu),
         (unsigned long)(g_probeU1Max / cpu),
         (unsigned long)g_probeU1BurstMax);
  g_probeTim6GapMax = 0U;
  g_probeOtgMax = 0U;
  g_probeU2Max = 0U;
  g_probeTickMax = 0U;
  g_probeU1Max = 0U;
  g_probeU1BurstMax = 0U;
}

void L2DB_UartReport(void)
{
  USART_TypeDef *u = huart1.Instance;
  printf("[UART1] CR1=0x%08lX ISR=0x%08lX  UE=%lu RE=%lu TE=%lu RXNEIE=%lu  dropped=%lu overrun=%lu\r\n",
         (unsigned long)u->CR1, (unsigned long)u->ISR,
         (unsigned long)((u->CR1 & USART_CR1_UE) != 0U),
         (unsigned long)((u->CR1 & USART_CR1_RE) != 0U),
         (unsigned long)((u->CR1 & USART_CR1_TE) != 0U),
         (unsigned long)((u->CR1 & USART_CR1_RXNEIE_RXFNEIE) != 0U),
         (unsigned long)s_rxDropped, (unsigned long)s_rxOverrun);
}

void L2DB_ConsolePoll(void)
{
  static char    buf[48];
  static uint8_t len = 0U;

  uint8_t ch = 0U;
  while (L2DB_UartGetChar(&ch))
  {
    if ((ch == '\r') || (ch == '\n'))
    {
      if (len > 0U)
      {
        buf[len] = '\0';
        printf("\r\n");
        L2DB_ConsoleExecLine(buf);
        len = 0U;
      }
      printf("> ");
      continue;
    }

    if (((ch == 0x08U) || (ch == 0x7FU)))     /* backspace */
    {
      if (len > 0U)
      {
        len--;
        printf("\b \b");
      }
      continue;
    }

    if ((ch >= 0x20U) && (ch < 0x7FU) && (len < (sizeof(buf) - 1U)))
    {
      buf[len++] = (char)ch;
      printf("%c", (char)ch);       /* local echo */
    }
  }
}
