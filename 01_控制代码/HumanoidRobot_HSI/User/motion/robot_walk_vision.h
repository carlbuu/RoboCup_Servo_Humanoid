#ifndef ROBOT_WALK_VISION_H
#define ROBOT_WALK_VISION_H

#include "robot_control.h"

void RobotWalkVision_Init(void);
uint8_t RobotWalkVision_BeginNewRun(void);
void RobotWalkVision_Update(uint16_t elapsedMs);
RobotControlStatus_t RobotWalkVision_GetStatus(void);

#endif
