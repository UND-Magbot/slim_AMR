#ifndef __RS485_KINCO_FD125_H
#define __RS485_KINCO_FD125_H

#include <stdint.h>
#include <stdbool.h>

void KincoFD125_Init(void);
void KincoFD125_EStop(void);
void KincoFD125_Update(float steerCommand, float walkCommand);
void KincoFD125_PollFeedback(void);
bool KincoFD125_GetJointFeedback(float *walkRpm, float *walkRadPerSec, float *steerPositionRad, float *steerVelocityRad);

#endif /* __RS485_KINCO_FD125_H */
