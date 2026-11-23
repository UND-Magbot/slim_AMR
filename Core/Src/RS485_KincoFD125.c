#include "RS485_KincoFD125.h"

#include "rs485.h"
#include "main.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define STEER_MOTOR_ID                  5U
#define WALK_MOTOR_ID                   1U

#define KINCO_REG_CONTROL_WORD          0x3100U
#define KINCO_REG_MODE_OF_OPERATION     0x3500U
#define KINCO_REG_TARGET_SPEED_SP       0x6F00U
#define KINCO_REG_TARGET_SPEED_PO     	0x4A00U
#define KINCO_REG_DIRECTION             0x4700U
#define KINCO_REG_TARGETPOS     		0x4000U
#define KINCO_REG_ACTUAL_SPEED          0x3B00U
#define KINCO_REG_ACTUAL_POSITION       0x3000U

#define KINCO_REG_HIGH_OFFSET           0x0010U

#define KINCO_CONTROL_SHUTDOWN          0x0006U
#define KINCO_CONTROL_SWITCH_ON         0x0007U
#define KINCO_CONTROL_Immed_Change		0x002FU
#define KINCO_CONTROL_Set_Point			0x003FU
#define KINCO_CONTROL_absolute			0x103FU
#define KINCO_CONTROL_ENABLE            0x100FU

#define KINCO_MODE_POSITION	           	1U
#define KINCO_MODE_SPEED	           	3U
#define KINCO_MODE_HOMING	           	6U

#define KINCO_RPM_GEAR					30.2f

#define DEFAULT_STEER_RPM               700.0f
#define MAX_WALK_RPM                    1300.0f
/* ===================== VEHICLE PARAM ===================== */

#define STEER_OMEGA_MAX_RADPS           1.0f
#define STEER_ANGLE_MAX_RAD             (3.1415926535f / 2.0f)



/* Wheel parameters for m/s -> RPM conversion (diameter 16 cm) */
#define WHEEL_DIAMETER_M                0.16f
#define WHEEL_CIRCUMFERENCE_M          	(WHEEL_DIAMETER_M * 3.1415926535f)
#define MPS_TO_RPM 						((60.0f / WHEEL_CIRCUMFERENCE_M) * KINCO_RPM_GEAR)


#define MODBUS_WRITE_SINGLE             0x06U
#define MODBUS_WRITE_MULTIPLE           0x10U
#define MODBUS_READ_HOLDING             0x03U

#define MODBUS_TIMEOUT_MS               20U
#define KINCO_FEEDBACK_PERIOD_MS        200U
#define KINCO_FEEDBACK_TIMEOUT_MS       (KINCO_FEEDBACK_PERIOD_MS * 2U)
#define KINCO_POSITION_COUNTS_PER_REV   12400000.0f
#define TWO_PI_F                        6.283185307f

typedef struct
{
  int32_t steerPosi;
  int32_t walkRpm;
  bool enabled;
  float feedbackWalkRpm;
  float feedbackWalkRadPerSec;
  int8_t walkDirectionSign;
  float feedbackSteerPositionRad;
  float feedbackSteerVelocityRad;
  uint32_t feedbackTimestampMs;
  bool feedbackValid;
} KincoState_t;

static KincoState_t g_state;
const int32_t POS_CENTER = 4200000U;
const int32_t POS_LEFT   = 1100000U;  /* -90 deg (measured) */
const int32_t POS_RIGHT  = 7300000U;  /* +90 deg (measured) */


static uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length);
static HAL_StatusTypeDef Modbus_WriteSingle(uint8_t id, uint16_t reg, uint16_t value);
static HAL_StatusTypeDef Modbus_Write32(uint8_t id, uint16_t reg, uint32_t value);
static HAL_StatusTypeDef Modbus_Read(uint8_t id, uint16_t reg, uint8_t byteCount, uint8_t *buffer);
static HAL_StatusTypeDef Kinco_EnableSpeedMode(uint8_t id);
static HAL_StatusTypeDef Kinco_EnablePositionMode(uint8_t id);
static void Kinco_SetSpeed(uint8_t id, float rpm);
static void Kinco_SetPosition(uint8_t id, int32_t position, float rpm);
static int32_t RpmToInternal(float rpm);
static float InternalToRpm(int32_t internal);
static float CountsToRadians(int32_t counts);
static HAL_StatusTypeDef RS485_WaitForResponse(uint16_t expectedLen, uint32_t timeoutMs, uint8_t *dest);

static HAL_StatusTypeDef Kinco_EnablePositionMode(uint8_t id)
{
  HAL_StatusTypeDef status = HAL_OK;
  status |= Modbus_WriteSingle(id, KINCO_REG_MODE_OF_OPERATION, KINCO_MODE_POSITION);	  HAL_Delay(10);
  status |= Modbus_WriteSingle(id, KINCO_REG_CONTROL_WORD, KINCO_CONTROL_SHUTDOWN);		  HAL_Delay(10);
  status |= Modbus_Write32(id, KINCO_REG_TARGETPOS, POS_CENTER);						  HAL_Delay(10);
  status |= Modbus_Write32(id, KINCO_REG_TARGET_SPEED_PO, (uint32_t)RpmToInternal(0));    HAL_Delay(10);
  status |= Modbus_WriteSingle(id, KINCO_REG_CONTROL_WORD, KINCO_CONTROL_Immed_Change);	  HAL_Delay(10);
  status |= Modbus_WriteSingle(id, KINCO_REG_CONTROL_WORD, KINCO_CONTROL_Set_Point);	  HAL_Delay(10);
  status |= Modbus_WriteSingle(id, KINCO_REG_CONTROL_WORD, KINCO_CONTROL_absolute);		  HAL_Delay(10);

  return status;
}


static HAL_StatusTypeDef Kinco_EnableSpeedMode(uint8_t id)
{
  HAL_StatusTypeDef status = HAL_OK;
  status |= Modbus_WriteSingle(id, KINCO_REG_MODE_OF_OPERATION, KINCO_MODE_SPEED);		  HAL_Delay(10);
  status |= Modbus_WriteSingle(id, KINCO_REG_CONTROL_WORD, KINCO_CONTROL_SHUTDOWN);		  HAL_Delay(10);
  status |= Modbus_WriteSingle(id, KINCO_REG_DIRECTION, 1U);							  HAL_Delay(10);
  status |= Modbus_Write32(id, KINCO_REG_TARGET_SPEED_SP, (uint32_t)RpmToInternal(0));	  HAL_Delay(10);

  status |= Modbus_WriteSingle(id, KINCO_REG_CONTROL_WORD, KINCO_CONTROL_SWITCH_ON);	  HAL_Delay(10);
  status |= Modbus_WriteSingle(id, KINCO_REG_CONTROL_WORD, KINCO_CONTROL_ENABLE);		  HAL_Delay(10);

  return status;
}

static void Kinco_SetPosition(uint8_t id, int32_t position, float rpm)
{
  /* position 0 or rpm 0 is treated as stop */
  int32_t internal = (rpm == 0) ? 0 : RpmToInternal(fabsf(rpm));
  Modbus_Write32(id, KINCO_REG_TARGETPOS, position);									  HAL_Delay(1);
  Modbus_Write32(id, KINCO_REG_TARGET_SPEED_PO, (uint32_t)internal);					  HAL_Delay(1);
}

static void Kinco_SetSpeed(uint8_t id, float rpm)
{
  static uint16_t Olddirect = 100u;
  uint16_t direction = (rpm >= 0) ? 1U : 0U;
  int32_t internal = RpmToInternal(fabsf(rpm));

  if(Olddirect != direction)
  {
	  Modbus_WriteSingle(id, KINCO_REG_CONTROL_WORD, KINCO_CONTROL_SHUTDOWN);			HAL_Delay(10L);
	  Modbus_WriteSingle(id, KINCO_REG_CONTROL_WORD, KINCO_CONTROL_SWITCH_ON);			HAL_Delay(10L);
	  Olddirect = direction;
  }


  Modbus_WriteSingle(id, KINCO_REG_DIRECTION, direction);							      HAL_Delay(1);
  Modbus_Write32(id, KINCO_REG_TARGET_SPEED_SP, (uint32_t)internal);				      HAL_Delay(1);

  Modbus_WriteSingle(id, KINCO_REG_CONTROL_WORD, KINCO_CONTROL_absolute);			      HAL_Delay(1);
}

static HAL_StatusTypeDef Modbus_WriteSingle(uint8_t id, uint16_t reg, uint16_t value)
{
  uint8_t frame[8];
  uint8_t response[8];

  frame[0] = id;
  frame[1] = MODBUS_WRITE_SINGLE;
  frame[2] = (uint8_t)(reg >> 8);
  frame[3] = (uint8_t)(reg & 0xFFU);
  frame[4] = (uint8_t)(value >> 8);
  frame[5] = (uint8_t)(value & 0xFFU);
  uint16_t crc = Modbus_CRC16(frame, 6U);
  frame[6] = (uint8_t)(crc & 0xFFU);
  frame[7] = (uint8_t)(crc >> 8);

  RS485_RxBuffer_Clear();
  RS485_TX_Set(1);
  HAL_StatusTypeDef status = HAL_UART_Transmit(&USART2_RS485Handler, frame, sizeof(frame), MODBUS_TIMEOUT_MS);
  RS485_TX_Set(0);
  if (status != HAL_OK)
  {
    return status;
  }

  status = RS485_WaitForResponse(8U, MODBUS_TIMEOUT_MS, response);
  if (status != HAL_OK)
  {
    return status;
  }

  uint16_t crcResp = Modbus_CRC16(response, 6U);
  if (((crcResp & 0xFFU) != response[6]) || ((crcResp >> 8) != response[7]))
  {
    return HAL_ERROR;
  }
  if ((response[0] != id) || (response[1] != MODBUS_WRITE_SINGLE))
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef Modbus_Write32(uint8_t id, uint16_t reg, uint32_t value)
{
  uint8_t frame[13];
  uint8_t response[8];
  uint16_t lowWord = (uint16_t)(value & 0xFFFFU);
  uint16_t highWord = (uint16_t)((value >> 16) & 0xFFFFU);

  frame[0] = id;
  frame[1] = MODBUS_WRITE_MULTIPLE;
  frame[2] = (uint8_t)(reg >> 8);
  frame[3] = (uint8_t)(reg & 0xFFU);
  frame[4] = 0x00;
  frame[5] = 0x02;
  frame[6] = 0x04;
  frame[7] = (uint8_t)(lowWord >> 8);
  frame[8] = (uint8_t)(lowWord & 0xFFU);
  frame[9] = (uint8_t)(highWord >> 8);
  frame[10] = (uint8_t)(highWord & 0xFFU);
  uint16_t crc = Modbus_CRC16(frame, 11U);
  frame[11] = (uint8_t)(crc & 0xFFU);
  frame[12] = (uint8_t)(crc >> 8);

  RS485_RxBuffer_Clear();
  RS485_TX_Set(1);
  HAL_StatusTypeDef status = HAL_UART_Transmit(&USART2_RS485Handler, frame, sizeof(frame), MODBUS_TIMEOUT_MS);
  RS485_TX_Set(0);
  if (status != HAL_OK)
  {
    return status;
  }

  status = RS485_WaitForResponse(8U, MODBUS_TIMEOUT_MS, response);
  if (status != HAL_OK)
  {
    return status;
  }

  uint16_t crcResp = Modbus_CRC16(response, 6U);
  if (((crcResp & 0xFFU) != response[6]) || ((crcResp >> 8) != response[7]))
  {
    return HAL_ERROR;
  }
  if ((response[0] != id) || (response[1] != MODBUS_WRITE_MULTIPLE))
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef Modbus_Read(uint8_t id, uint16_t reg, uint8_t byteCount, uint8_t *buffer)
{
  if ((buffer == NULL) || (byteCount == 0U))
  {
    return HAL_ERROR;
  }

  uint8_t frame[8];
  uint8_t response[64];
  uint16_t expectedLen = (uint16_t)(3U + byteCount + 2U);
  if (expectedLen > sizeof(response))
  {
    return HAL_ERROR;
  }

  memset(response, 0, sizeof(response));

  frame[0] = id;
  frame[1] = MODBUS_READ_HOLDING;
  frame[2] = (uint8_t)(reg >> 8);
  frame[3] = (uint8_t)(reg & 0xFFU);
  frame[4] = 0x00;
  frame[5] = byteCount;
  uint16_t crc = Modbus_CRC16(frame, 6U);
  frame[6] = (uint8_t)(crc & 0xFFU);
  frame[7] = (uint8_t)(crc >> 8);

  RS485_RxBuffer_Clear();
  RS485_TX_Set(1);
  HAL_StatusTypeDef status = HAL_UART_Transmit(&USART2_RS485Handler, frame, sizeof(frame), MODBUS_TIMEOUT_MS);
  RS485_TX_Set(0);
  if (status != HAL_OK)
  {
    return status;
  }

  status = RS485_WaitForResponse(expectedLen, MODBUS_TIMEOUT_MS, response);

  if (status != HAL_OK)
  {
    return status;
  }

  if ((response[0] != id) || (response[1] != MODBUS_READ_HOLDING) || (response[2] != byteCount))
  {
    return HAL_ERROR;
  }

  uint16_t crcCalc = Modbus_CRC16(response, (uint16_t)(3U + byteCount));
  if (((crcCalc & 0xFFU) != response[3U + byteCount]) || ((crcCalc >> 8) != response[4U + byteCount]))
  {
    return HAL_ERROR;
  }

  for (uint8_t i = 0U; i < byteCount; ++i)
  {
    buffer[i] = response[3U + i];
  }

  return HAL_OK;
}

static uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0; i < length; ++i)
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8U; ++j)
    {
      if (crc & 0x0001U)
      {
        crc = (crc >> 1U) ^ 0xA001U;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }
  return crc;
}

static int32_t RpmToInternal(float rpm)
{
  const int64_t scale = 512LL * 65536LL;
  int64_t scaled = (int64_t)rpm * scale;
  scaled /= 1875LL;
  return (int32_t)scaled;
}

static float InternalToRpm(int32_t internal)
{
  const float scale = (512.0f * 65536.0f);
  return (((float)internal * 1875.0f) / scale);
}

static float CountsToRadians(int32_t counts)
{
  return ((float)counts * TWO_PI_F) / KINCO_POSITION_COUNTS_PER_REV;
}


static HAL_StatusTypeDef RS485_WaitForResponse(uint16_t expectedLen, uint32_t timeoutMs, uint8_t *dest)
{
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < timeoutMs)
  {
    if (RS485_RxBuffer_Count() >= expectedLen)
    {
      if (dest != NULL)
      {
        RS485_RxBuffer_Read(dest, expectedLen);
      }
      else
      {
        RS485_RxBuffer_Read(NULL, expectedLen);
      }
      return HAL_OK;
    }
    HAL_Delay(1);
  }
  return HAL_TIMEOUT;
}



void KincoFD125_EStop(void)
{
  Kinco_SetPosition(STEER_MOTOR_ID, 0, 0);
  Kinco_SetSpeed(WALK_MOTOR_ID, 0);
  g_state.enabled = false;
}


static float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float SteerCountsPerRad(void)
{
  return ((float)(POS_RIGHT - POS_LEFT)) / 3.1415926535f;
}

static int32_t SteerRadToPos(float steerRad)
{
  steerRad = clampf(steerRad, -STEER_ANGLE_MAX_RAD, STEER_ANGLE_MAX_RAD);
  int32_t pos = POS_CENTER + (int32_t)(steerRad * SteerCountsPerRad());

  if (pos < POS_LEFT)  pos = POS_LEFT;
  if (pos > POS_RIGHT) pos = POS_RIGHT;

  return pos;
}

void KincoFD125_Update(float angularZ, float linearX)
{
  if (!g_state.enabled)
  {
	  KincoFD125_Init();
  }

  /* steerCommand now treated as desired steering angle (rad), clamp to physical limits */
  /* ROS angular.z > 0 = left turn, hardware position increases to the right → invert sign */
  float steerRad = clampf(-angularZ, -STEER_ANGLE_MAX_RAD, STEER_ANGLE_MAX_RAD);
  int32_t targetPos = SteerRadToPos(steerRad);

  float steerRpm = DEFAULT_STEER_RPM;

  if (targetPos != g_state.steerPosi)
  {
    Kinco_SetPosition(STEER_MOTOR_ID, targetPos, steerRpm);
    g_state.steerPosi = targetPos;
  }

  /* linearX is interpreted as linear velocity in m/s (from joystick/USB) */
  float linearMps = linearX;
  float walkRpmF = linearMps * MPS_TO_RPM;
  walkRpmF = clampf(walkRpmF, -MAX_WALK_RPM, MAX_WALK_RPM);
  
  if (walkRpmF != (float)g_state.walkRpm)
  {
    Kinco_SetSpeed(WALK_MOTOR_ID, walkRpmF);
    g_state.walkRpm = (int32_t)walkRpmF;
  }
}

void KincoFD125_PollFeedback(void)
{
  static uint32_t lastPollMs = 0U;
  uint32_t timestampMs = api_getCurtMs();

  if (HAL_TimDiff(timestampMs, lastPollMs) < KINCO_FEEDBACK_PERIOD_MS)
  {
    return;
  }

  bool updated = false;
  uint8_t readBuf[4];
  uint32_t rawValue = 0U;
  float walkRpmMeasured = 0.0f;
  bool walkSpeedValid = false;

  memset(readBuf,0,sizeof(readBuf));
  if (Modbus_Read(WALK_MOTOR_ID, KINCO_REG_ACTUAL_SPEED, 4U, readBuf) == HAL_OK)
  {
    uint16_t lowWord = (uint16_t)((readBuf[0] << 8) | readBuf[1]);
    uint16_t highWord = (uint16_t)((readBuf[2] << 8) | readBuf[3]);
    rawValue = ((uint32_t)highWord << 16) | lowWord;
    int32_t signedRaw = (int32_t)rawValue;
    walkRpmMeasured = InternalToRpm(signedRaw) / KINCO_RPM_GEAR;
    walkSpeedValid = true;
  }
  HAL_Delay(1);

  uint8_t dirBuf[2];
  memset(dirBuf,0,sizeof(dirBuf));
  if (Modbus_Read(WALK_MOTOR_ID, KINCO_REG_DIRECTION, 2U, dirBuf) == HAL_OK)
  {
    uint16_t dirRaw = (uint16_t)((dirBuf[0] << 8) | dirBuf[1]);
    /* 0 => CCW is positive, 1 => CW is positive; treat CW as forward (+) */
    g_state.walkDirectionSign = (dirRaw == 0U) ? -1 : 1;
  }

  float directionSign = (g_state.walkDirectionSign < 0) ? -1.0f : 1.0f;
  if (walkSpeedValid)
  {
    float signedRpm = walkRpmMeasured * directionSign;
    g_state.feedbackWalkRpm = signedRpm;
    g_state.feedbackWalkRadPerSec = signedRpm * (TWO_PI_F / 60.0f);
    updated = true;
  }

  HAL_Delay(1);

  memset(readBuf,0,sizeof(readBuf));
  if (Modbus_Read(STEER_MOTOR_ID, KINCO_REG_ACTUAL_POSITION, 4U, readBuf) == HAL_OK)
  {
    uint16_t lowWord = (uint16_t)((readBuf[0] << 8) | readBuf[1]);
    uint16_t highWord = (uint16_t)((readBuf[2] << 8) | readBuf[3]);
    rawValue = ((uint32_t)highWord << 16) | lowWord;
    int32_t signedCounts = (int32_t)rawValue;
    int32_t centeredCounts = signedCounts - POS_CENTER;
    float newRad = CountsToRadians(centeredCounts);
    float velocity = 0.0f;
    if (g_state.feedbackValid)
    {
      uint32_t diff = HAL_TimDiff(api_getCurtMs(), g_state.feedbackTimestampMs);
      if ((diff != UINT32_MAX) && (diff > 0U))
      {
        float dt = diff / 1000.0f;
        velocity = (newRad - g_state.feedbackSteerPositionRad) / dt;
      }
    }
    g_state.feedbackSteerVelocityRad = velocity;
    g_state.feedbackSteerPositionRad = newRad;
    updated = true;
  }

  if (updated)
  {
    g_state.feedbackTimestampMs = timestampMs;
    g_state.feedbackValid = true;
  } 
  else if (HAL_TimDiff(timestampMs, g_state.feedbackTimestampMs) > KINCO_FEEDBACK_TIMEOUT_MS)
  {
    g_state.feedbackValid = false;
    g_state.feedbackWalkRpm = 0.0f;
    g_state.feedbackWalkRadPerSec = 0.0f;
    g_state.feedbackSteerVelocityRad = 0.0f;
  }

  lastPollMs = timestampMs;
}

bool KincoFD125_GetJointFeedback(float *walkRpm, float *walkRadPerSec, float *steerPositionRad, float *steerVelocityRad)
{
  bool valid = g_state.feedbackValid;

  if (valid)
  {
    uint32_t age = HAL_TimDiff(api_getCurtMs(), g_state.feedbackTimestampMs);
    if ((age == UINT32_MAX) || (age > KINCO_FEEDBACK_TIMEOUT_MS))
    {
      valid = false;
    }
  }

  if (walkRpm != NULL)
  {
    *walkRpm = g_state.feedbackWalkRpm;
  }
  if (walkRadPerSec != NULL)
  {
    *walkRadPerSec = g_state.feedbackWalkRadPerSec;
  }
  if (steerPositionRad != NULL)
  {
    *steerPositionRad = g_state.feedbackSteerPositionRad;
  }
  if (steerVelocityRad != NULL)
  {
    *steerVelocityRad = g_state.feedbackSteerVelocityRad;
  }
  return valid;
}


void KincoFD125_Init(void)
{
  memset(&g_state, 0, sizeof(g_state));

  Kinco_EnablePositionMode(STEER_MOTOR_ID);
  Kinco_EnableSpeedMode(WALK_MOTOR_ID);

  g_state.enabled = true;
}
