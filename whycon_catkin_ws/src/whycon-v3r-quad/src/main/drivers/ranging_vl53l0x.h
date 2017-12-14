#pragma once

#ifdef __cplusplus
 extern "C" {
#endif

#include "vl53l0x_def.h"
#include "vl53l0x_device.h"
#include "vl53l0x_platform.h"

extern VL53L0X_Dev_t MyDevice;
//extern VL53L0X_Dev_t *pMyDevice;
	
extern VL53L0X_Error Global_Status;
extern VL53L0X_RangingMeasurementData_t RangingMeasurementData;

void ranging_init(void);

void getRange(void);
extern uint16_t NewSensorRange;

bool isTofDataNew(void);
extern bool isTofDataNewflag;

bool isOutofRange(void);

extern uint16_t debug_range;
extern uint8_t Range_Status;
#ifdef __cplusplus
 }
#endif