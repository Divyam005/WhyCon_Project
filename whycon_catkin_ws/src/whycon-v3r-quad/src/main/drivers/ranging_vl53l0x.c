#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"

#include "vl53l0x_api_core.h"
#include "vl53l0x_api_strings.h"
#include "vl53l0x_def.h"
#include "vl53l0x_api.h"
#include "vl53l0x_types.h"

#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/bus_i2c.h"
#include "drivers/system.h"

#include "ranging_vl53l0x.h"

#ifdef LASER_TOF

VL53L0X_Dev_t MyDevice;
//VL53L0X_Dev_t *pMyDevice = &MyDevice;

VL53L0X_Error Global_Status = 0;
VL53L0X_RangingMeasurementData_t RangingMeasurementData;

uint16_t NewSensorRange;
uint16_t debug_range = 0;
uint8_t Range_Status = 0;
bool isTofDataNewflag = false, out_of_range = false;

void update_status(VL53L0X_Error Status){
    Global_Status = Status;
}

void ranging_init(void)
{	
	VL53L0X_Error Status = Global_Status;
	
	uint32_t refSpadCount;
    uint8_t isApertureSpads;
	uint8_t VhvSettings;
    uint8_t PhaseCal;
	
    // Initialize Comms
    MyDevice.I2cDevAddr      =  0x29;
    MyDevice.comms_type      =  1;	
    MyDevice.comms_speed_khz =  400;
	
    
	Status = VL53L0X_DataInit(&MyDevice); // Data initialization
    
	update_status(Status);
	
	if(Global_Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_StaticInit(&MyDevice); // Device Initialization
        update_status(Status);	
	
	}
	
	if( Global_Status == VL53L0X_ERROR_NONE ) 
	{
		Status = VL53L0X_PerformRefSpadManagement( &MyDevice, &refSpadCount, &isApertureSpads ); // Device Initialization
		update_status(Status);	
		
	}
	
	if( Global_Status == VL53L0X_ERROR_NONE ) 
	{
		Status = VL53L0X_PerformRefCalibration( &MyDevice, &VhvSettings, &PhaseCal );           // Device Initialization
	    update_status(Status);
		
	}
	
	if(Global_Status == VL53L0X_ERROR_NONE)
    {
        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        Status = VL53L0X_SetDeviceMode(&MyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
        update_status(Status);
		
    } 
	
	// Enable/Disable Sigma and Signal check
	if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(&MyDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
		
		update_status(Status);
    }
    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(&MyDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
				
		update_status(Status);
    }
				
    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(&MyDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t)(0.1*65536));
				
		update_status(Status);
	}			
    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(&MyDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(60*65536));			
    }
    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&MyDevice,33000);
			
		update_status(Status);
	}	
	
    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(&MyDevice, 
		        VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    
		update_status(Status);
	}	
	
    if (Global_Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(&MyDevice, 
		        VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    
		update_status(Status);
	}	
	
}

void getRange(void)
{	
	VL53L0X_Error Status = Global_Status;
	static uint8_t dataFlag = 0, SysRangeStatus = 0;
	static bool startNow = true;
	
    if(Global_Status == VL53L0X_ERROR_NONE)
    {	
		if(startNow){
			Status = VL53L0X_StartMeasurement(&MyDevice);
			update_status(Status);	
			startNow = false;
		}
    }
	
	if(Global_Status == VL53L0X_ERROR_NONE){
		if(!startNow){
			Status = VL53L0X_GetMeasurementDataReady(&MyDevice,&dataFlag);
			update_status(Status);
		}
	}
	
	if(Global_Status == VL53L0X_ERROR_NONE){
		if(dataFlag){
			Status = VL53L0X_GetRangingMeasurementData(&MyDevice, &RangingMeasurementData);
			update_status(Status);
			
			NewSensorRange = RangingMeasurementData.RangeMilliMeter; 
			/* if(RangingMeasurementData.RangeDMaxMilliMeter != 0){
				debug_range = RangingMeasurementData.RangeDMaxMilliMeter/10;
			} */
			
			startNow = true;
			isTofDataNewflag = true;
			Range_Status = RangingMeasurementData.RangeStatus;
			debug_range = (uint16_t)Range_Status;
			if(RangingMeasurementData.RangeStatus == 0){
				out_of_range = false;
			}else
				out_of_range = true;
		}
	}
	
}

bool isTofDataNew(void)
{
	return isTofDataNewflag;
}

bool isOutofRange(void)
{
	return out_of_range;
}

#endif



