/**
  ******************************************************************************
  * @file    can.h
  * @brief   FDCAN1 bring-up + CANopen SDO client for L2DB4830 servo controller.
  ******************************************************************************
  */
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---------------------------------------------------------------------------
 * Bus configuration
 *   FDCAN1  RX = PD0, TX = PD1 (AF9)
 *   500 kbit/s  <- L2DB4830 SW2-2 = OFF
 *   Node id     <- L2DB4830 SW3 = 1
 * ------------------------------------------------------------------------- */
#define L2DB_NODE_ID_DEFAULT      1U

/* CANopen SDO COB-IDs */
#define SDO_TX_COBID(node)        (0x600U + (uint32_t)(node))   /* master -> drive */
#define SDO_RX_COBID(node)        (0x580U + (uint32_t)(node))   /* drive  -> master */

/* SDO command specifiers (manual 5.3) */
#define SDO_CMD_WRITE_1B          0x2FU
#define SDO_CMD_WRITE_2B          0x2BU
#define SDO_CMD_WRITE_4B          0x23U
#define SDO_CMD_READ              0x40U
#define SDO_RSP_WRITE_OK          0x60U
#define SDO_RSP_READ_1B           0x4FU
#define SDO_RSP_READ_2B           0x4BU
#define SDO_RSP_READ_4B           0x43U
#define SDO_RSP_ABORT             0x80U

/* ---------------------------------------------------------------------------
 * L2DB4830 object dictionary (LSDB manual, table F1-1 .. F1-6)
 * ------------------------------------------------------------------------- */
#define OBJ_MODE_OF_OPERATION     0x6060U, 0x00U   /*  8S  3=speed(ramp) 1=position */
#define OBJ_CONTROL_WORD          0x6040U, 0x00U   /* 16S  0x0F=enable 0x06=free 0x86=clr */
#define OBJ_MODE_DISPLAY          0x6061U, 0x00U   /*  8S  RO */
#define OBJ_STATUS_WORD           0x6041U, 0x00U   /* 16U  RO  0=ok 8=alarm */
#define OBJ_ERROR_CODE            0x2601U, 0x00U   /* 16U  RO  bitfield */
#define OBJ_QUICK_STOP            0x605AU, 0x11U   /*  8U  1=stop 0=release */
#define OBJ_QUICK_STOP_DEC        0x605AU, 0x01U   /* 32U */

#define OBJ_TARGET_SPEED_RPM      0x2FF0U, 0x09U   /* 32S  rpm, direct */
/* 32S  DEC = rpm*512*res/1875 (manual 6.2). ~1118 counts per rpm at res 4096,
 * so it takes fractional rpm - the rpm object above only takes integers. */
#define OBJ_TARGET_VELOCITY       0x60FFU, 0x00U
#define OBJ_TARGET_SPEED_DEC      0x60FFU, 0x00U   /* 32S  DEC */
#define OBJ_TARGET_POS_ABS        0x607AU, 0x00U   /* 32S  inc */
#define OBJ_TARGET_POS_REL        0x607BU, 0x00U   /* 32S  inc */
#define OBJ_PROFILE_SPEED_RPM     0x6082U, 0x00U   /* 16U  rpm (position mode) */
#define OBJ_PROFILE_ACCEL         0x6083U, 0x00U   /* 32U  DEC */
#define OBJ_PROFILE_DECEL         0x6084U, 0x00U   /* 32U  DEC */
#define OBJ_MAX_SPEED_RPM         0x6080U, 0x00U   /* 16U  rpm */

#define OBJ_ACTUAL_POSITION       0x6063U, 0x00U   /* 32S  RO  inc */
#define OBJ_ACTUAL_SPEED_RPM      0x60F9U, 0x18U   /* 16S  RO  rpm */
#define OBJ_ACTUAL_SPEED_DEC      0x606CU, 0x00U   /* 32S  RO  DEC */
#define OBJ_ACTUAL_CURRENT_IQ     0x6078U, 0x00U   /* 16S  RO  DEC = Arms*1.414*2048/50 */
/* 2026-09-29: read to tell whether caster kicks hit the current limit, a soft
 * speed loop, or wheel slip (manual 6.x object table). */
#define OBJ_CURRENT_LIMIT         0x6073U, 0x00U   /* 16U  DEC = Arms*1.414*2048/50 */
#define OBJ_MOTOR_OVERLOAD_A      0x6410U, 0x09U   /* 16U  0.1 A */
#define OBJ_MOTOR_MAX_A           0x6410U, 0x0BU   /* 16U  0.1 A */
#define OBJ_SPEED_LOOP_KP         0x60F9U, 0x01U   /* 16U  */
#define OBJ_SPEED_LOOP_KI         0x60F9U, 0x02U   /* 16U  */
/* 2026-11-16: in speed mode 3 the drive also closes a position loop on the
 * integrated target speed (manual 6.2: P gain 0x60FB01 factory 10, speed
 * feed-forward 0x60FB02 0..256 = 0..100%). */
#define OBJ_POS_LOOP_KP           0x60FBU, 0x01U   /* 16S  */
#define OBJ_POS_LOOP_FF           0x60FBU, 0x02U   /* 16S  0..256 */
#define OBJ_FOLLOW_ERR_WINDOW     0x6065U, 0x00U   /* 32U  DEC */
#define OBJ_BUS_VOLTAGE           0x60F7U, 0x12U   /* 16S  RO  V */
#define OBJ_ENCODER_RESOLUTION    0x6410U, 0x03U   /* 32U  feedback resolution */
#define OBJ_MOTOR_MODEL_CODE      0x6410U, 0x16U   /* 16U  RO */

/* Communication-loss shutdown - factory default is DISABLED. */
#define OBJ_COMMLOSS_STOP_EN      0x4100U, 0x10U   /*  8U  0=off 1=on   (group S1) */
#define OBJ_COMMLOSS_STOP_DELAY   0x4100U, 0x11U   /* 32U  ms, default 600 (S1) */
#define OBJ_SAVE_GROUP_S1         0x2FE5U, 0x01U   /*  8U  write 1 to store */
#define OBJ_SAVE_GROUP_S5         0x2FE5U, 0x05U   /*  8U  write 1 to store */

/* Values for OBJ_MODE_OF_OPERATION */
#define L2DB_MODE_POSITION        1
#define L2DB_MODE_SPEED_RAMP      3
#define L2DB_MODE_SPEED_IMMEDIATE (-3)
#define L2DB_MODE_TORQUE          4
#define L2DB_MODE_HOMING          6

/* Values for OBJ_CONTROL_WORD */
#define L2DB_CTRL_FREE            0x06U    /* de-energise, shaft free */
#define L2DB_CTRL_ENABLE          0x0FU    /* energise, shaft locked  */
#define L2DB_CTRL_CLEAR_ALARM     0x86U

typedef enum
{
  CAN_MODE_NORMAL = 0,
  CAN_MODE_LOOPBACK        /* internal loopback: no wiring / no drive needed */
} CanMode_t;

typedef struct
{
  bool     ok;             /* response received and not an abort */
  uint8_t  cmd;            /* raw response command specifier */
  uint8_t  len;            /* payload bytes reported by the drive (1/2/4) */
  uint32_t value;          /* raw little-endian payload */
  uint32_t abortCode;      /* valid when cmd == 0x80 */
} SdoResult_t;

/* Bring-up ---------------------------------------------------------------- */
bool     CAN_Init(CanMode_t mode);
void     CAN_DeInit(void);
bool     CAN_SelfTest(void);          /* internal loopback TX/RX round trip   */
void     CAN_ProbeNodes(uint8_t firstNode, uint8_t lastNode);
void     CAN_SnifferPoll(void);       /* dump every received frame to printf  */
void     CAN_ReportBusState(void);
void     CAN_PrintDriveBusLine(void);  /* one "[DRIVE] CAN1 ..." line (shows in base_node log) */
void     CAN_BusWatch(void);           /* main loop: restart FDCAN1 after bus-off */
void     CAN_ScanNodes(void);          /* console 'p': which CANopen node ids answer */
void     CAN_BringUp(void);           /* selftest -> normal -> probe          */

/* SDO client -------------------------------------------------------------- */
SdoResult_t CAN_SdoRead (uint8_t node, uint16_t index, uint8_t sub);
SdoResult_t CAN_SdoWrite(uint8_t node, uint16_t index, uint8_t sub,
                         uint32_t value, uint8_t size);

/* Unit conversion (manual: DEC = rps/s * 256 * resolution / 15625) --------- */
uint32_t CAN_RpsPerSecToDec(float rpsPerSec, uint32_t encoderResolution);
int32_t  CAN_RpmToDec(float rpm, uint32_t encoderResolution);

#ifdef __cplusplus
}
#endif
#endif /* __CAN_H__ */
