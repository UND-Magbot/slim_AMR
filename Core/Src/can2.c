/**
  ******************************************************************************
  * @file    can2.c
  * @brief   FDCAN2 receive-only bus probe for the battery (BMS) link.
  *
  *   n                      status + id table
  *   n <kbps> [pins] [a]    start: kbps 125/250/500/1000, pins 0=PB12/13
  *                          1=PB5/6, 'a' = ACK frames (default: monitor only)
  *   n 0                    stop
 *   n r [idx] [addr]       ask the Tabos battery for status (needs 'a' mode):
 *                          idx 0 = index 1..3 at once (default), 0x10..0x40 one
 *   n p <0|1>              battery polling off/on (on at boot, see CAN2_BatteryInit)
 *
 * Tabos LV-series BMS (LV-25V25AH-DH255COM, protocol 2025.07.18): 500 kbit/s,
 * CAN 2.0A, id 0x460 + rotary address. It stays silent until asked:
 * request [0x60+addr, idx, 0...]; answer on the same id with D2 = index 1..4.
  *
  * While running it prints one "[CAN2]" summary line per second, or, when
  * battery polling is on, one "[BAT]" line per second for base_node to parse:
  *   [BAT] v=2791 i=-100 st=0000 soc=76 soh=98 rc=1890 ae=4788 t=247 ttf=0
  *         tte=1134 cyc=12 age=210        (raw units, see Can2Bat_t; age ms)
  * Polled from the main loop - no interrupt, so it cannot delay the USART1
  * receive ISR.
  ******************************************************************************
  */
#include "can2.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CAN2_KER_HZ        80000000UL   /* PLL2Q, set up by CAN_Init() */
#define CAN2_TQ_PER_BIT    16U          /* 1 + 13 + 2, sample point 87.5 % */
#define CAN2_RAM_OFFSET    256U         /* words; FDCAN1 uses < 100 at 0 */
#define CAN2_RX_ELEMENTS   64U
#define CAN2_MAX_IDS       16U
#define CAN2_REPORT_MS     1000U

typedef struct
{
  uint32_t id;
  bool     ext;
  uint8_t  dlc;
  uint8_t  data[8];
  uint32_t count;
  uint32_t lastMs;
} Can2Id_t;

static FDCAN_HandleTypeDef hfdcan2;
static bool     s_run;
static uint32_t s_kbps;
static uint8_t  s_pins;
static bool     s_ack;
static Can2Id_t s_ids[CAN2_MAX_IDS];
static uint32_t s_nIds;
static uint32_t s_total;
static uint32_t s_lastTotal;
static uint32_t s_idOverflow;
static uint32_t s_lastReport;

/* Latest Tabos battery values (index 1..3) */
typedef struct
{
  bool     have[4];
  uint16_t mv10;        /* V x100 */
  int16_t  ma10;        /* A x100, + charge / - discharge */
  uint16_t status;      /* bit0 OV 1 UV 2 chg OC 3 dis OC 4 hot 5 cold 6 BMU err */
  uint16_t ttf, tte;    /* min */
  uint8_t  soc, soh;    /* % */
  uint16_t rc;          /* Ah x100 */
  uint16_t ae;          /* Wh x10 */
  int16_t  temp;        /* degC x10 */
  uint16_t cyc;
  uint32_t lastMs;
} Can2Bat_t;
static Can2Bat_t s_bat;

#define TABOS_ID_BASE      0x460U
/* Request period. The protocol asks for >= 500 ms. Index 4 (cycle count) is
 * not part of the index-0 burst, so it is asked separately now and then. */
#define BAT_POLL_MS        1000U
#define BAT_CYC_EVERY      30U

static bool     s_poll;
static bool     s_wantBat;      /* 부팅 때 켰다. 실패해도 다시 시도한다 */
static uint32_t s_retryMs;
static uint32_t s_pollMs;
static uint32_t s_pollN;

static void CAN2_Request(uint8_t idx, uint8_t addr);

static void CAN2_Gpio(uint8_t pinSet, bool on)
{
  GPIO_TypeDef *port = GPIOB;
  uint16_t pins = (pinSet == 0U) ? (GPIO_PIN_12 | GPIO_PIN_13)
                                 : (GPIO_PIN_5 | GPIO_PIN_6);
  if (!on)
  {
    HAL_GPIO_DeInit(port, pins);
    return;
  }
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin       = pins;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF9_FDCAN2;
  HAL_GPIO_Init(port, &gpio);
}

void CAN2_Stop(void)
{
  if (!s_run)
  {
    return;
  }
  (void)HAL_FDCAN_Stop(&hfdcan2);
  /* HAL_FDCAN_MspDeInit() returns early for FDCAN2, so the kernel clock that
   * FDCAN1 shares stays on. */
  (void)HAL_FDCAN_DeInit(&hfdcan2);
  CAN2_Gpio(s_pins, false);
  s_run = false;
  s_poll = false;
  printf("[CAN2] stopped\r\n");
}

bool CAN2_Start(uint32_t kbps, uint8_t pinSet, bool ack)
{
  uint32_t presc;
  switch (kbps)
  {
    case 125U:  presc = 40U; break;
    case 250U:  presc = 20U; break;
    case 500U:  presc = 10U; break;
    case 1000U: presc = 5U;  break;
    default:
      printf("[CAN2] kbps must be 125/250/500/1000\r\n");
      return false;
  }
  if (pinSet > 1U)
  {
    printf("[CAN2] pins must be 0 (PB12/13) or 1 (PB5/6)\r\n");
    return false;
  }
  CAN2_Stop();

  CAN2_Gpio(pinSet, true);
  __HAL_RCC_FDCAN_CLK_ENABLE();

  memset(&hfdcan2, 0, sizeof(hfdcan2));
  hfdcan2.Instance                  = FDCAN2;
  hfdcan2.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode                 = ack ? FDCAN_MODE_NORMAL
                                          : FDCAN_MODE_BUS_MONITORING;
  hfdcan2.Init.AutoRetransmission   = DISABLE;
  hfdcan2.Init.TransmitPause        = DISABLE;
  hfdcan2.Init.ProtocolException    = DISABLE;
  hfdcan2.Init.NominalPrescaler     = presc;
  hfdcan2.Init.NominalSyncJumpWidth = 2U;
  hfdcan2.Init.NominalTimeSeg1      = 13U;
  hfdcan2.Init.NominalTimeSeg2      = 2U;
  hfdcan2.Init.DataPrescaler        = presc;
  hfdcan2.Init.DataSyncJumpWidth    = 2U;
  hfdcan2.Init.DataTimeSeg1         = 13U;
  hfdcan2.Init.DataTimeSeg2         = 2U;
  hfdcan2.Init.MessageRAMOffset     = CAN2_RAM_OFFSET;
  hfdcan2.Init.StdFiltersNbr        = 0U;
  hfdcan2.Init.ExtFiltersNbr        = 0U;
  hfdcan2.Init.RxFifo0ElmtsNbr      = CAN2_RX_ELEMENTS;
  hfdcan2.Init.RxFifo0ElmtSize      = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr      = 0U;
  hfdcan2.Init.RxFifo1ElmtSize      = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr         = 0U;
  hfdcan2.Init.RxBufferSize         = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr          = 0U;
  hfdcan2.Init.TxBuffersNbr         = 0U;
  hfdcan2.Init.TxFifoQueueElmtsNbr  = 1U;
  hfdcan2.Init.TxElmtSize           = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    printf("[CAN2] init FAILED\r\n");
    CAN2_Gpio(pinSet, false);
    return false;
  }
  /* No filter elements: everything goes through the global filter. */
  if ((HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_REJECT_REMOTE,
                                    FDCAN_REJECT_REMOTE) != HAL_OK)
      || (HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2, FDCAN_RX_FIFO0,
                                          FDCAN_RX_FIFO_OVERWRITE) != HAL_OK)
      || (HAL_FDCAN_Start(&hfdcan2) != HAL_OK))
  {
    printf("[CAN2] start FAILED\r\n");
    (void)HAL_FDCAN_DeInit(&hfdcan2);
    CAN2_Gpio(pinSet, false);
    return false;
  }

  memset(s_ids, 0, sizeof(s_ids));
  memset(&s_bat, 0, sizeof(s_bat));
  s_nIds = s_total = s_lastTotal = s_idOverflow = 0U;
  s_kbps = kbps;
  s_pins = pinSet;
  s_ack  = ack;
  s_run  = true;
  s_lastReport = HAL_GetTick();
  printf("[CAN2] started %lukbit/s pins=%s mode=%s\r\n", (unsigned long)kbps,
         pinSet ? "PB5/PB6" : "PB12/PB13", ack ? "ack" : "monitor");
  return true;
}

static void CAN2_Store(const FDCAN_RxHeaderTypeDef *h, const uint8_t *d)
{
  bool ext = (h->IdType == FDCAN_EXTENDED_ID);
  uint8_t dlc = (uint8_t)(h->DataLength >> 16);   /* classic: DLC code = bytes */
  if (dlc > 8U)
  {
    dlc = 8U;
  }
  s_total++;
  if (!ext && (h->Identifier >= TABOS_ID_BASE)
      && (h->Identifier <= TABOS_ID_BASE + 0x0FU) && (dlc == 8U))
  {
    uint8_t idx = d[1];
    uint16_t a = (uint16_t)(d[2] | (d[3] << 8));
    uint16_t b = (uint16_t)(d[4] | (d[5] << 8));
    uint16_t c = (uint16_t)(d[6] | (d[7] << 8));
    switch (idx)
    {
      case 1U: s_bat.mv10 = a; s_bat.ma10 = (int16_t)b; s_bat.status = c; break;
      case 2U: s_bat.ttf = a; s_bat.tte = b; s_bat.soc = d[6]; s_bat.soh = d[7]; break;
      case 3U: s_bat.rc = a; s_bat.ae = b; s_bat.temp = (int16_t)c; break;
      case 4U: s_bat.cyc = a; break;
      default: idx = 0U; break;
    }
    if ((idx >= 1U) && (idx <= 4U))
    {
      s_bat.have[idx - 1U] = true;
      s_bat.lastMs = HAL_GetTick();
    }
  }
  for (uint32_t i = 0U; i < s_nIds; ++i)
  {
    if ((s_ids[i].id == h->Identifier) && (s_ids[i].ext == ext))
    {
      s_ids[i].count++;
      s_ids[i].dlc = dlc;
      memcpy(s_ids[i].data, d, dlc);
      s_ids[i].lastMs = HAL_GetTick();
      return;
    }
  }
  if (s_nIds >= CAN2_MAX_IDS)
  {
    s_idOverflow++;
    return;
  }
  Can2Id_t *e = &s_ids[s_nIds++];
  e->id = h->Identifier;
  e->ext = ext;
  e->dlc = dlc;
  memcpy(e->data, d, dlc);
  e->count = 1U;
  e->lastMs = HAL_GetTick();
}

static void CAN2_PrintStatus(void)
{
  FDCAN_ProtocolStatusTypeDef ps = {0};
  FDCAN_ErrorCountersTypeDef ec = {0};
  (void)HAL_FDCAN_GetProtocolStatus(&hfdcan2, &ps);
  (void)HAL_FDCAN_GetErrorCounters(&hfdcan2, &ec);
  /* LEC: 0 none 1 stuff 2 form 3 ack 4 bit1 5 bit0 6 crc 7 no change */
  printf("[CAN2] %lukbit/s %s %s rx=+%lu total=%lu ids=%lu%s tec=%lu rec=%lu"
         " lec=%lu%s%s\r\n",
         (unsigned long)s_kbps, s_pins ? "PB5/6" : "PB12/13",
         s_ack ? "ack" : "mon",
         (unsigned long)(s_total - s_lastTotal), (unsigned long)s_total,
         (unsigned long)s_nIds, s_idOverflow ? "+" : "",
         (unsigned long)ec.TxErrorCnt, (unsigned long)ec.RxErrorCnt,
         (unsigned long)ps.LastErrorCode,
         ps.ErrorPassive ? " PASSIVE" : "", ps.BusOff ? " BUSOFF" : "");
  s_lastTotal = s_total;
  if (s_bat.have[0] || s_bat.have[1] || s_bat.have[2])
  {
    printf("[CAN2] bat V=%u.%02u I=%s%d.%02u st=0x%04X SOC=%u SOH=%u RC=%u.%02uAh"
           " T=%s%d.%u TTE=%umin age=%lums\r\n",
           s_bat.mv10 / 100U, s_bat.mv10 % 100U,
           (s_bat.ma10 < 0) ? "-" : "", abs(s_bat.ma10) / 100, abs(s_bat.ma10) % 100,
           s_bat.status, s_bat.soc, s_bat.soh, s_bat.rc / 100U, s_bat.rc % 100U,
           (s_bat.temp < 0) ? "-" : "", abs(s_bat.temp) / 10, abs(s_bat.temp) % 10,
           s_bat.tte, (unsigned long)(HAL_GetTick() - s_bat.lastMs));
  }
}

static void CAN2_Request(uint8_t idx, uint8_t addr)
{
  if (!s_run || !s_ack)
  {
    printf("[CAN2] request needs ack mode: n 500 <pins> a\r\n");
    return;
  }
  FDCAN_TxHeaderTypeDef tx = {0};
  tx.Identifier          = TABOS_ID_BASE + (addr & 0x0FU);
  tx.IdType              = FDCAN_STANDARD_ID;
  tx.TxFrameType         = FDCAN_DATA_FRAME;
  tx.DataLength          = FDCAN_DLC_BYTES_8;
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch       = FDCAN_BRS_OFF;
  tx.FDFormat            = FDCAN_CLASSIC_CAN;
  tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker       = 0U;
  uint8_t d[8] = {(uint8_t)(0x60U + (addr & 0x0FU)), idx, 0, 0, 0, 0, 0, 0};
  HAL_StatusTypeDef st = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx, d);
  if (s_poll && (st == HAL_OK))
  {
    return;
  }
  printf("[CAN2] request id 0x%03lX idx 0x%02X %s\r\n",
         (unsigned long)tx.Identifier, idx, (st == HAL_OK) ? "queued" : "FAILED");
}

void CAN2_Poll(void)
{
  if (!s_run)
  {
    /* Bus-off restart (or boot init) failed: keep retrying every 5 s. */
    if (s_wantBat && ((HAL_GetTick() - s_retryMs) >= 5000U))
    {
      s_retryMs = HAL_GetTick();
      (void)CAN2_BatteryInit();
    }
    return;
  }
  FDCAN_RxHeaderTypeDef h;
  uint8_t d[8];
  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0U)
  {
    if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &h, d) != HAL_OK)
    {
      break;
    }
    CAN2_Store(&h, d);
  }
  uint32_t now = HAL_GetTick();
  if (s_poll)
  {
    if ((now - s_pollMs) >= BAT_POLL_MS)
    {
      s_pollMs = now;
      FDCAN_ProtocolStatusTypeDef ps = {0};
      (void)HAL_FDCAN_GetProtocolStatus(&hfdcan2, &ps);
      if (ps.BusOff)
      {
        /* Bus-off leaves the core in INIT; start it again from scratch. */
        printf("[CAN2] bus-off - restarting\r\n");
        s_poll = CAN2_Start(s_kbps, s_pins, s_ack);
        s_retryMs = HAL_GetTick();
        return;
      }
      /* Report the answer to the previous request, then ask again. */
      /* Only when index 1..3 have all answered at least once; otherwise the
       * missing fields are still 0 from the memset and SOC would read 0 %. */
      if (s_bat.have[0] && s_bat.have[1] && s_bat.have[2])
      {
        printf("[BAT] v=%u i=%d st=%04X soc=%u soh=%u rc=%u ae=%u t=%d ttf=%u"
               " tte=%u cyc=%u age=%lu\r\n",
               s_bat.mv10, s_bat.ma10, s_bat.status, s_bat.soc, s_bat.soh,
               s_bat.rc, s_bat.ae, s_bat.temp, s_bat.ttf, s_bat.tte, s_bat.cyc,
               (unsigned long)(now - s_bat.lastMs));
      }
      else
      {
        printf("[BAT] none\r\n");
      }
      CAN2_Request((uint8_t)((++s_pollN % BAT_CYC_EVERY) == 0U ? 0x40U : 0x00U),
                   0U);
    }
    return;
  }
  if ((now - s_lastReport) >= CAN2_REPORT_MS)
  {
    s_lastReport = now;
    CAN2_PrintStatus();
  }
}

bool CAN2_BatteryInit(void)
{
  /* Board CAN2 H/L terminal = PB5/PB6 (found 2026-09-29: PB12/13 gave bit
   * errors, PB5/6 got the battery's answer). */
  s_wantBat = true;
  s_retryMs = HAL_GetTick();
  if (!CAN2_Start(500U, 1U, true))
  {
    return false;
  }
  s_poll = true;
  s_pollMs = HAL_GetTick();
  return true;
}

void CAN2_PrintTable(void)
{
  if (!s_run)
  {
    printf("[CAN2] not running (n <kbps> [0|1] [a])\r\n");
    return;
  }
  CAN2_PrintStatus();
  uint32_t now = HAL_GetTick();
  for (uint32_t i = 0U; i < s_nIds; ++i)
  {
    const Can2Id_t *e = &s_ids[i];
    char hex[3 * 8 + 1];
    hex[0] = '\0';
    for (uint8_t b = 0U; b < e->dlc; ++b)
    {
      snprintf(&hex[3U * b], 4, "%02X ", e->data[b]);
    }
    printf("[CAN2] id %s0x%0*lX n=%lu age=%lums dlc=%u %s\r\n",
           e->ext ? "x" : "", e->ext ? 8 : 3, (unsigned long)e->id,
           (unsigned long)e->count, (unsigned long)(now - e->lastMs),
           (unsigned)e->dlc, hex);
  }
}

void CAN2_ConsoleCmd(char *args)
{
  while (*args == ' ')
  {
    args++;
  }
  if (*args == 'p')
  {
    int on = 1;
    (void)sscanf(args + 1, "%d", &on);
    if (on && !s_run)
    {
      (void)CAN2_BatteryInit();
    }
    s_poll = (on != 0) && s_run && s_ack;
    s_pollMs = HAL_GetTick();
    printf("[CAN2] battery polling %s\r\n", s_poll ? "on" : "off");
    return;
  }
  if (*args == 'r')
  {
    unsigned idx = 0U, addr = 0U;
    (void)sscanf(args + 1, "%i %i", (int *)&idx, (int *)&addr);
    CAN2_Request((uint8_t)idx, (uint8_t)addr);
    return;
  }
  int kbps = -1, pins = 0;
  char mode = '\0';
  int n = sscanf(args, "%d %d %c", &kbps, &pins, &mode);
  if (n < 1)
  {
    CAN2_PrintTable();
    return;
  }
  s_wantBat = false;             /* manual control from here on */
  if (kbps == 0)
  {
    CAN2_Stop();
    return;
  }
  (void)CAN2_Start((uint32_t)kbps, (uint8_t)pins, (mode == 'a'));
}
