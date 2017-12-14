


#pragma once


#ifdef __cplusplus
 extern "C" {
#endif
#include "common/axis.h"
#include "flight/pid.h"//PS2

extern int16_t posX,posY,posZ, deltaTime;

extern int16_t PositionX, PositionY;
extern int16_t PrevPositionX, PrevPositionY;
extern int16_t VelocityX, VelocityY;
extern int16_t WhyconVx, WhyconVy;
extern float accel[2], accel_prev[2];


void addHistPositionBaseEstXY(float positionX, float positionY);
void getFrontHistPositionBaseEstXY(float *posbaseX, float *posbaseY);
bool isPositionBaseXYQueueIsFull(void);


void PosXYEstimate(uint32_t currentTime);
void resetPosition(void);
void updatePosGains(void);
void setPos(float newX, float newY);

extern int16_t print_posvariable1, print_posvariable2, print_posvariable3, print_posvariable4, print_posvariable5, print_posvariable6;
void configurePosHold2(pidProfile_t *initialPidProfile);
#ifdef __cplusplus
 }
#endif
