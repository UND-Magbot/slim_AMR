/**
  ******************************************************************************
  * @file    can.c
  * @brief   FDCAN1 bring-up + CANopen SDO client for the L2DB4830 servo drive.
  *
  *          Bus    : 500 kbit/s classic CAN, sample point 87.5 %
  *          Pins   : PD0 = FDCAN1_RX, PD1 = FDCAN1_TX (AF9)
  *          Kernel : PLL2Q = 80 MHz (HSI/2 -> /4 -> x50 -> /5)
  ******************************************************************************
  */

#include "can.h"

#include <stdio.h>
#include <string.h>

/* Bit timing: ker_ck 80 MHz / prescaler 10 = 8 MHz tq
 *             1 (sync) + 13 (seg1) + 2 (seg2) = 16 tq -> 500 kbit/s
 *             sample point = 14/16 = 87.5 %                                  */
#define CAN_NOM_PRESCALER      10U
#define CAN_NOM_SEG1           13U
#define CAN_NOM_SEG2            2U
#define CAN_NOM_SJW             2U

#define SDO_TIMEOUT_MS         10U
#define SNIFFER_MAX_PER_POLL    8U

FDCAN_HandleTypeDef hfdcan1;

static bool     s_canReady    = false;
static uint32_t s_rxTotal     = 0U;
static uint32_t s_txTotal     = 0U;
static uint32_t s_unexpected  = 0U;

/* -------------------------------------------------------------------------- */
/* Low level                                                                  */
/* -------------------------------------------------------------------------- */

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan->Instance != FDCAN1)
  {
    return;
  }

  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_FDCAN_CLK_ENABLE();

  /* PD0 = FDCAN1_RX, PD1 = FDCAN1_TX */
  gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOD, &gpio);
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan->Instance != FDCAN1)
  {
    return;
  }
  __HAL_RCC_FDCAN_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);
}

/* PLL2Q -> 80 MHz for the FDCAN kernel clock.
 * PLL source is HSI (already RCC_HSI_DIV2 = 32 MHz in SystemClock_Config).
 * 32 MHz / M4 = 8 MHz ref -> x50 = 400 MHz VCO -> /Q5 = 80 MHz              */
static bool CAN_ConfigKernelClock(void)
{
  RCC_PeriphCLKInitTypeDef periph = {0};

  periph.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  periph.PLL2.PLL2M      = 4;
  periph.PLL2.PLL2N      = 50;
  periph.PLL2.PLL2P      = 2;
  periph.PLL2.PLL2Q      = 5;      /* 80 MHz */
  periph.PLL2.PLL2R      = 2;
  periph.PLL2.PLL2RGE    = RCC_PLL2VCIRANGE_3;   /* 8 - 16 MHz input */
  periph.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  periph.PLL2.PLL2FRACN  = 0;
  periph.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;

  return (HAL_RCCEx_PeriphCLKConfig(&periph) == HAL_OK);
}

bool CAN_Init(CanMode_t mode)
{
  if (!CAN_ConfigKernelClock())
  {
    printf("[CAN] kernel clock config FAILED\r\n");
    return false;
  }

  hfdcan1.Instance                  = FDCAN1;
  hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode                 = (mode == CAN_MODE_LOOPBACK)
                                        ? FDCAN_MODE_INTERNAL_LOOPBACK
                                        : FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission   = ENABLE;
  hfdcan1.Init.TransmitPause        = DISABLE;
  hfdcan1.Init.ProtocolException    = DISABLE;

  hfdcan1.Init.NominalPrescaler     = CAN_NOM_PRESCALER;
  hfdcan1.Init.NominalSyncJumpWidth = CAN_NOM_SJW;
  hfdcan1.Init.NominalTimeSeg1      = CAN_NOM_SEG1;
  hfdcan1.Init.NominalTimeSeg2      = CAN_NOM_SEG2;
  /* Data phase is unused in classic mode but must still be valid. */
  hfdcan1.Init.DataPrescaler        = CAN_NOM_PRESCALER;
  hfdcan1.Init.DataSyncJumpWidth    = CAN_NOM_SJW;
  hfdcan1.Init.DataTimeSeg1         = CAN_NOM_SEG1;
  hfdcan1.Init.DataTimeSeg2         = CAN_NOM_SEG2;

  hfdcan1.Init.MessageRAMOffset     = 0;
  hfdcan1.Init.StdFiltersNbr        = 1;
  hfdcan1.Init.ExtFiltersNbr        = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr      = 16;
  hfdcan1.Init.RxFifo0ElmtSize      = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr      = 0;
  hfdcan1.Init.RxFifo1ElmtSize      = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr         = 0;
  hfdcan1.Init.RxBufferSize         = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr          = 0;
  hfdcan1.Init.TxBuffersNbr         = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr  = 8;
  hfdcan1.Init.TxElmtSize           = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    printf("[CAN] HAL_FDCAN_Init FAILED\r\n");
    return false;
  }

  /* Accept every standard id into RX FIFO0 (mask 0 matches all). */
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType       = FDCAN_STANDARD_ID;
  filter.FilterIndex  = 0;
  filter.FilterType   = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1    = 0x000;
  filter.FilterID2    = 0x000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    printf("[CAN] ConfigFilter FAILED\r\n");
    return false;
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    printf("[CAN] ConfigGlobalFilter FAILED\r\n");
    return false;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    printf("[CAN] HAL_FDCAN_Start FAILED\r\n");
    return false;
  }

  s_canReady = true;
  printf("[CAN] init ok  mode=%s  500kbit/s  ker=80MHz  PD0/PD1\r\n",
         (mode == CAN_MODE_LOOPBACK) ? "LOOPBACK" : "NORMAL");
  return true;
}

void CAN_DeInit(void)
{
  if (s_canReady)
  {
    HAL_FDCAN_Stop(&hfdcan1);
    HAL_FDCAN_DeInit(&hfdcan1);
    s_canReady = false;
  }
}

/* -------------------------------------------------------------------------- */
/* Frame helpers                                                              */
/* -------------------------------------------------------------------------- */

static bool CAN_Send(uint32_t stdId, const uint8_t payload[8])
{
  if (!s_canReady)
  {
    return false;
  }

  FDCAN_TxHeaderTypeDef tx = {0};
  tx.Identifier          = stdId;
  tx.IdType              = FDCAN_STANDARD_ID;
  tx.TxFrameType         = FDCAN_DATA_FRAME;
  tx.DataLength          = FDCAN_DLC_BYTES_8;
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch       = FDCAN_BRS_OFF;
  tx.FDFormat            = FDCAN_CLASSIC_CAN;
  tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker       = 0;

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
  {
    printf("[CAN] TX fifo full\r\n");
    return false;
  }
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, (uint8_t *)payload) != HAL_OK)
  {
    printf("[CAN] TX failed\r\n");
    return false;
  }
  s_txTotal++;
  return true;
}

static bool CAN_Receive(uint32_t *stdId, uint8_t payload[8], uint8_t *dlc)
{
  if (!s_canReady)
  {
    return false;
  }
  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U)
  {
    return false;
  }

  FDCAN_RxHeaderTypeDef rx = {0};
  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, payload) != HAL_OK)
  {
    return false;
  }

  s_rxTotal++;
  *stdId = rx.Identifier;
  /* DataLength is the coded DLC; for classic frames it maps 1:1 up to 8. */
  *dlc = (uint8_t)((rx.DataLength <= 8U) ? rx.DataLength : 8U);
  return true;
}

static void CAN_DumpFrame(const char *tag, uint32_t id, const uint8_t d[8], uint8_t dlc)
{
  printf("%s 0x%03lX [%u]", tag, (unsigned long)id, (unsigned)dlc);
  for (uint8_t i = 0U; i < dlc; ++i)
  {
    printf(" %02X", d[i]);
  }
  printf("\r\n");
}

/* -------------------------------------------------------------------------- */
/* SDO client                                                                 */
/* -------------------------------------------------------------------------- */

static SdoResult_t CAN_SdoTransfer(uint8_t node, const uint8_t req[8])
{
  SdoResult_t res = {0};
  const uint32_t expectId = SDO_RX_COBID(node);

  if (!CAN_Send(SDO_TX_COBID(node), req))
  {
    return res;
  }

  const uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < SDO_TIMEOUT_MS)
  {
    uint32_t id  = 0U;
    uint8_t  dlc = 0U;
    uint8_t  d[8];

    if (!CAN_Receive(&id, d, &dlc))
    {
      continue;
    }

    if (id != expectId)
    {
      s_unexpected++;
      CAN_DumpFrame("[DRIVE] ?RX", id, d, dlc);   /* base_node 로그에 보이게 (2026-10-26) */
      continue;
    }

    res.cmd = d[0];
    res.value = (uint32_t)d[4]
              | ((uint32_t)d[5] << 8)
              | ((uint32_t)d[6] << 16)
              | ((uint32_t)d[7] << 24);

    switch (d[0])
    {
      case SDO_RSP_READ_1B:  res.len = 1U; res.ok = true; break;
      case SDO_RSP_READ_2B:  res.len = 2U; res.ok = true; break;
      case SDO_RSP_READ_4B:  res.len = 4U; res.ok = true; break;
      case SDO_RSP_WRITE_OK: res.len = 0U; res.ok = true; break;
      case SDO_RSP_ABORT:    res.abortCode = res.value;   break;
      default:                                            break;
    }
    return res;
  }

  return res;   /* timeout: ok == false, cmd == 0 */
}

SdoResult_t CAN_SdoRead(uint8_t node, uint16_t index, uint8_t sub)
{
  uint8_t req[8] = {0};
  req[0] = SDO_CMD_READ;
  req[1] = (uint8_t)(index & 0xFFU);
  req[2] = (uint8_t)(index >> 8);
  req[3] = sub;
  return CAN_SdoTransfer(node, req);
}

SdoResult_t CAN_SdoWrite(uint8_t node, uint16_t index, uint8_t sub,
                         uint32_t value, uint8_t size)
{
  uint8_t req[8] = {0};

  switch (size)
  {
    case 1U:  req[0] = SDO_CMD_WRITE_1B; break;
    case 2U:  req[0] = SDO_CMD_WRITE_2B; break;
    case 4U:  req[0] = SDO_CMD_WRITE_4B; break;
    default:  { SdoResult_t bad = {0}; return bad; }
  }

  req[1] = (uint8_t)(index & 0xFFU);
  req[2] = (uint8_t)(index >> 8);
  req[3] = sub;
  req[4] = (uint8_t)(value & 0xFFU);
  req[5] = (uint8_t)((value >> 8) & 0xFFU);
  req[6] = (uint8_t)((value >> 16) & 0xFFU);
  req[7] = (uint8_t)((value >> 24) & 0xFFU);

  return CAN_SdoTransfer(node, req);
}

/* -------------------------------------------------------------------------- */
/* Unit conversion                                                            */
/* -------------------------------------------------------------------------- */

uint32_t CAN_RpsPerSecToDec(float rpsPerSec, uint32_t encoderResolution)
{
  /* DEC = rps/s * 256 * resolution / 15625 */
  double dec = ((double)rpsPerSec * 256.0 * (double)encoderResolution) / 15625.0;
  if (dec < 0.0) dec = 0.0;
  return (uint32_t)dec;
}

int32_t CAN_RpmToDec(float rpm, uint32_t encoderResolution)
{
  /* DEC = rpm * 512 * resolution / 1875 */
  double dec = ((double)rpm * 512.0 * (double)encoderResolution) / 1875.0;
  return (int32_t)dec;
}

/* -------------------------------------------------------------------------- */
/* Bring-up helpers                                                           */
/* -------------------------------------------------------------------------- */

void CAN_ReportBusState(void)
{
  FDCAN_ProtocolStatusTypeDef ps = {0};
  FDCAN_ErrorCountersTypeDef  ec = {0};

  HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
  HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);

  printf("[CAN] state: busOff=%u errPassive=%u warning=%u  TEC=%u REC=%u  lastErr=%lu\r\n",
         (unsigned)ps.BusOff, (unsigned)ps.ErrorPassive, (unsigned)ps.Warning,
         (unsigned)ec.TxErrorCnt, (unsigned)ec.RxErrorCnt,
         (unsigned long)ps.LastErrorCode);
  printf("[CAN] counters: tx=%lu rx=%lu unexpected=%lu\r\n",
         (unsigned long)s_txTotal, (unsigned long)s_rxTotal,
         (unsigned long)s_unexpected);
}

/* 2026-10-26: after the drive wiring was redone the drives stopped answering
 * (READ-FAIL on every SDO) and nothing told us why. The bus state was only
 * printed once at boot, and base_node only forwards [DRIVE]/[L2DB] lines.
 * lastErr: 0 none, 1 stuff, 2 form, 3 ACK (nobody on the bus answers),
 *          4 bit1, 5 bit0, 6 CRC, 7 unchanged.                              */
void CAN_PrintDriveBusLine(void)
{
  static const char *const lec[] = {"none", "stuff", "form", "ACK-nobody-answers",
                                     "bit1", "bit0", "crc", "unchanged"};
  FDCAN_ProtocolStatusTypeDef ps = {0};
  FDCAN_ErrorCountersTypeDef  ec = {0};

  HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
  HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
  printf("[DRIVE] CAN1 busOff=%u passive=%u warn=%u TEC=%u REC=%u lastErr=%lu(%s) tx=%lu rx=%lu\r\n",
         (unsigned)ps.BusOff, (unsigned)ps.ErrorPassive, (unsigned)ps.Warning,
         (unsigned)ec.TxErrorCnt, (unsigned)ec.RxErrorCnt,
         (unsigned long)ps.LastErrorCode, lec[ps.LastErrorCode & 7U],
         (unsigned long)s_txTotal, (unsigned long)s_rxTotal);
}

/* 2026-10-26: with the drives unpowered the controller counted TX errors up to
 * bus-off and stayed there; only a board reset brought it back (CAN2 already
 * restarts itself). Bus-off leaves CCCR.INIT set - clearing it makes the core
 * wait 129 x 11 recessive bits and rejoin the bus.                          */
void CAN_BusWatch(void)
{
  static uint32_t lastMs = 0U;
  uint32_t now = HAL_GetTick();
  if ((now - lastMs) < 500U)
  {
    return;
  }
  lastMs = now;
  FDCAN_ProtocolStatusTypeDef ps = {0};
  HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
  if (ps.BusOff)
  {
    printf("[DRIVE] CAN1 bus-off - restarting\r\n");
    CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
  }
}

/* 2026-10-26: drives ACK every frame (TEC 0) but nodes 1/2 never answer an
 * SDO - find out which node ids are on the bus. Reads 0x1000 (device type)
 * from 1..127, 10 ms each (~1.3 s, blocks the loop - only with the robot idle). */
void CAN_ScanNodes(void)
{
  unsigned found = 0U;
  printf("[DRIVE] CAN1 node scan 1..127\r\n");
  for (uint8_t n = 1U; n <= 127U; ++n)
  {
    SdoResult_t r = CAN_SdoRead(n, 0x1000U, 0U);
    if (r.ok || r.cmd == 0x80U)
    {
      printf("[DRIVE] node %u answers: cmd=0x%02X value=0x%08lX abort=0x%08lX\r\n",
             (unsigned)n, (unsigned)r.cmd, (unsigned long)r.value, (unsigned long)r.abortCode);
      found++;
    }
  }
  printf("[DRIVE] CAN1 node scan done: %u node(s)\r\n", found);
}

bool CAN_SelfTest(void)
{
  const uint8_t pattern[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04};

  printf("[CAN] --- internal loopback self test ---\r\n");
  if (!CAN_Send(0x123, pattern))
  {
    return false;
  }

  const uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < 20U)
  {
    uint32_t id  = 0U;
    uint8_t  dlc = 0U;
    uint8_t  d[8];

    if (!CAN_Receive(&id, d, &dlc))
    {
      continue;
    }

    CAN_DumpFrame("[CAN]   RX", id, d, dlc);
    if ((id == 0x123) && (dlc == 8U) && (memcmp(d, pattern, 8) == 0))
    {
      printf("[CAN] self test PASS - peripheral, clock and message RAM are good\r\n");
      return true;
    }
  }

  printf("[CAN] self test FAIL - no loopback frame returned\r\n");
  return false;
}

/* Probe a node id range. Reads three objects that every axis must answer;
 * whichever ids reply tell us how the two channels (M1/M2) are addressed,
 * and the encoder resolution needed for the DEC unit conversions.           */
void CAN_ProbeNodes(uint8_t firstNode, uint8_t lastNode)
{
  printf("[CAN] --- node probe %u..%u ---\r\n",
         (unsigned)firstNode, (unsigned)lastNode);

  for (uint8_t node = firstNode; node <= lastNode; ++node)
  {
    SdoResult_t pos = CAN_SdoRead(node, OBJ_ACTUAL_POSITION);

    if (!pos.ok && (pos.cmd == 0U))
    {
      printf("[CAN] node %-2u : no response\r\n", (unsigned)node);
      continue;
    }
    if (pos.cmd == SDO_RSP_ABORT)
    {
      printf("[CAN] node %-2u : ALIVE, abort 0x%08lX on 0x6063:00\r\n",
             (unsigned)node, (unsigned long)pos.abortCode);
      continue;
    }

    SdoResult_t res    = CAN_SdoRead(node, OBJ_ENCODER_RESOLUTION);
    SdoResult_t status = CAN_SdoRead(node, OBJ_STATUS_WORD);
    SdoResult_t err    = CAN_SdoRead(node, OBJ_ERROR_CODE);
    SdoResult_t volt   = CAN_SdoRead(node, OBJ_BUS_VOLTAGE);
    SdoResult_t mode   = CAN_SdoRead(node, OBJ_MODE_DISPLAY);

    printf("[CAN] node %-2u : ALIVE\r\n", (unsigned)node);
    printf("        actual position 0x6063:00 = %ld inc\r\n",
           (long)(int32_t)pos.value);
    printf("        encoder res     0x6410:03 = %lu\r\n",
           (unsigned long)res.value);
    printf("        status word     0x6041:00 = 0x%04lX %s\r\n",
           (unsigned long)(status.value & 0xFFFFU),
           ((status.value & 0x0008U) != 0U) ? "(ALARM)" : "(ok)");
    printf("        error code      0x2601:00 = 0x%04lX\r\n",
           (unsigned long)(err.value & 0xFFFFU));
    printf("        bus voltage     0x60F7:12 = %d V\r\n",
           (int)(int16_t)volt.value);
    printf("        active mode     0x6061:00 = %d\r\n",
           (int)(int8_t)mode.value);
  }
}

void CAN_SnifferPoll(void)
{
  for (uint8_t i = 0U; i < SNIFFER_MAX_PER_POLL; ++i)
  {
    uint32_t id  = 0U;
    uint8_t  dlc = 0U;
    uint8_t  d[8];

    if (!CAN_Receive(&id, d, &dlc))
    {
      return;
    }
    CAN_DumpFrame("[CAN]  RX", id, d, dlc);
  }
}

void CAN_BringUp(void)
{
  printf("\r\n========== FDCAN1 bring-up ==========\r\n");

  /* 1) peripheral / clock / message RAM, independent of wiring */
  if (CAN_Init(CAN_MODE_LOOPBACK))
  {
    (void)CAN_SelfTest();
    CAN_ReportBusState();
  }
  CAN_DeInit();

  /* 2) real bus */
  if (!CAN_Init(CAN_MODE_NORMAL))
  {
    printf("[CAN] normal mode init failed - stopping bring-up\r\n");
    return;
  }

  CAN_ProbeNodes(1U, 4U);
  CAN_ReportBusState();
  printf("========== bring-up done ==========\r\n\r\n");
}
