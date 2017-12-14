

#pragma once


#ifdef __cplusplus
 extern "C" {
#endif

#include "common/axis.h"
#include "flight/pid.h"//PS2

bool PositionController(int16_t desiredX, int16_t desiredY, int16_t desiredZ);
void VelocityController(int16_t VdesiredX, int16_t VdesiredY);
void PosVelController(int16_t desiredX, int16_t desiredY, int16_t desiredVel);
bool SimpleController(int16_t desiredX, int16_t desiredY, int16_t desiredZ);

extern int PID_x, PID_y;
extern uint8_t HeightAchieved;

int getrcDataRoll(void);
int getrcDataPitch(void);
int getDesiredVelocityX();
int getDesiredVelocityY();


void setdesiredHeight(int32_t desiredZ);
int32_t getdesiredHeight(void);

void configurePosHold(pidProfile_t *initialPidProfile);//PS2
void resetPosIntegral(void);

#ifdef __cplusplus
 }
#endif
