#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/axis.h"
#include "common/utils.h"


#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/sonar.h"
#include "sensors/acceleration.h"
#include "rx/rx.h"
#include "io/rc_controls.h"
#include "mw.h"
#include "config/runtime_config.h"
#include "flight/posControl.h"


#include "command/command.h"
#define timeScale 1000000

int current_cmd_index = 0;
bool is_cmd_completed = false;
int total_cmd_index = 0;
int total_loop_times = -1;
cmd_def cmd_temp[TOTAL_CMDS];

void command_def(cmd_def *cmd){
	cmd_temp[total_cmd_index] = *cmd;
	total_cmd_index++;
}

bool command_verify(void){
	/* if(current_cmd_index == total_cmd_index - 1)
		return false;
	else*/	  
  return is_cmd_completed;
}

void command_next(void){
	if(current_cmd_index<total_cmd_index-1)
	current_cmd_index++;
	is_cmd_completed = false; //Reset this flag
	HeightAchieved = 0;
}

void command_jump(uint8_t jump){
	if(jump>=0 && jump < total_cmd_index)
		current_cmd_index = jump; 
	else{//Reset 
		current_cmd_index = 0;
		
	}
	if(jump==-1)//Reset Condition
	{
		current_cmd_index = 0;
		total_loop_times = -1;
	}
		
	is_cmd_completed = false; //Reset this flag
	
}
void command_previous(void){
	if(current_cmd_index)
		current_cmd_index--;
	is_cmd_completed = false;
}


bool pos_controller(float pos_x, float pos_y, float height){
	return true;
}

void command_run(uint32_t currentTime){
	static uint32_t timeKeeper;
	
	if(current_cmd_index < total_cmd_index){

	switch(cmd_temp[current_cmd_index].id){
		case TAKE_OFF:
		if(!ARMING_FLAG(ARMED)&&(rcData[AUX2] == 1850)&&IS_RC_MODE_ACTIVE(BOXARM))
			mwArm();
		//is_cmd_completed = SimpleController(cmd_temp[current_cmd_index].pos_x, cmd_temp[current_cmd_index].pos_y, cmd_temp[current_cmd_index].height);
		is_cmd_completed = PositionController(cmd_temp[current_cmd_index].pos_x, cmd_temp[current_cmd_index].pos_y, cmd_temp[current_cmd_index].height);
		break;

		case GO_TO_WAY_POINT:
		//is_cmd_completed = SimpleController(cmd_temp[current_cmd_index].pos_x, cmd_temp[current_cmd_index].pos_y, cmd_temp[current_cmd_index].height);		
		is_cmd_completed = PositionController(cmd_temp[current_cmd_index].pos_x, cmd_temp[current_cmd_index].pos_y, cmd_temp[current_cmd_index].height);		
		break;

		case SLEEP:	
		is_cmd_completed = false;
		static uint8_t sleepManager = 0;
		
		if(sleepManager==0){//Start Counting condition
			sleepManager = 1;//Start sleeping
			timeKeeper=currentTime;
			command_previous();
			
		}else{
			if((currentTime-timeKeeper)>cmd_temp[current_cmd_index].pos_x*timeScale){
				command_next();
				sleepManager=0;
			}else{
				command_previous();
			}
		}
		break;

		case LAND:
		//HeightAchieved = 0;
		is_cmd_completed = PositionController(cmd_temp[current_cmd_index].pos_x, cmd_temp[current_cmd_index].pos_y, -1);		
		if(is_cmd_completed)
			mwDisarm();
		break;

		case LOOP_FROM_START:
		if(total_loop_times==-1)//Fresh Start; 
		{
			if(cmd_temp[current_cmd_index].pos_x>0)
				total_loop_times = cmd_temp[current_cmd_index].pos_x;//Load the loop_times
			else
				total_loop_times = 1;//Default
		}
					
		if(total_loop_times>0){//If loop_times is non-zero then 
			total_loop_times--;
			if(cmd_temp[current_cmd_index].pos_x>=0)
				command_jump(cmd_temp[current_cmd_index].pos_x);
			else
				command_jump(0);
			}
		else{
			if((total_cmd_index - current_cmd_index)>0)
				command_next();
			else
				command_previous();
			}
		break;
		
		
		case ARM_MOTORS:
			mwArm();
		break;

		case DISARM_MOTORS:
			mwDisarm();
		break;
	}

	}
	
	
}
