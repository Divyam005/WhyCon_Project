/**********************Position Estimate - Drona Aviation **************************/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "io/serial_msp.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "config/runtime_config.h"

#include "posEstimate.h"
#include "posControl.h"


#define corr_scale 512/100
#define corr_scale2 1096/100

static pidProfile_t *pidProfile;//PS2
int16_t print_posvariable1=0, print_posvariable2=0, print_posvariable3=0, print_posvariable4=0, print_posvariable5=0, print_posvariable6=0;

int16_t posX=0, posY=0, posZ=0, deltaTime=0;
float whyconX = 0, whyconY = 0, dTime = 0;
float whyconPreX = 0, whyconPreY = 0;

static int first_read = 0;
int i;
float accel[2], accel_correction[2], acc_velocity[2], accel_prev[2];
float position_error[2], position_base[2], position_correction[2], est_position[2];
float velocity_increase[2];
float hist_position_baseX, hist_position_baseY;

float time_constant_xy =0.5f;
float k1,k2,k3;

int16_t PositionX = 0, PositionY = 0;
int16_t PrevPositionX=0, PrevPositionY=0;
int16_t VelocityX = 0, VelocityY = 0;
int16_t WhyconVx = 0, WhyconVy = 0;

int MAXIMUM=5;

static float bufferX[5], bufferY[5];
static int head=0;
static int rear=-1;
static int itemCount=0;


void resetPosition(void){//Reset Pos and velocity
	est_position[0] = 0;
	est_position[1] = 0;
	acc_velocity[0] = 0;
	acc_velocity[1] = 0;
	resetPosIntegral();
} 

void addHistPositionBaseEstXY(float positionX, float positionY)
{
	if(itemCount<MAXIMUM)
	{
		rear++;
		if(rear>=MAXIMUM)
		{
			rear=0;
		}
		bufferX[rear]=positionX;
		bufferY[rear]=positionY;
		itemCount++;
	}
	else
	{
		if(++rear==MAXIMUM)
		{
			rear=0;
			bufferX[rear]=positionX;
			bufferY[rear]=positionY;
			head++;
		}
		else
		{
			bufferX[rear]=positionX;
			bufferY[rear]=positionY;
			head++;
			if(head==MAXIMUM)
			{
				head=0;
			}
		}
	}
}

void getFrontHistPositionBaseEstXY(float *posbaseX, float *posbaseY)
{
	*posbaseX=bufferX[head];
	*posbaseY=bufferY[head];
	head++;

	if(head==MAXIMUM)
	{
		head=0;
	}
	itemCount--;
}


bool isPositionBaseXYQueueIsFull(void)
{
	return itemCount==MAXIMUM;
}


void PosXYEstimate(uint32_t currentTime)
{
	static uint32_t previousTime = 0, previousReadTime=0;
    float dt = (currentTime - previousTime)/ 1000000.0f;//sec

	if ((currentTime - previousTime) < 10000)	//10ms
		return;

	previousTime = currentTime;


	if(read_position){

		previousReadTime = currentTime;
		//WhyconVx = (int16_t)whyconX;//*10;//Storing previous values
		//WhyconVy = (int16_t)whyconY;//*10;//

		whyconX = -1.0f*(float)posY;///10;	//centimeter
		whyconY = -1.0f*(float)posX;///10;
		dTime = (float)deltaTime;		//millisecond

		//WhyconVx = (100*(-posY - WhyconVx))/(deltaTime); // Units cm/s
		//WhyconVy = (100*(-posX - WhyconVy))/(deltaTime); // Units cm/s

		read_position = false;


		if(first_read == 0) {
			setPos(whyconX, whyconY);
			first_read++;
		}

		if (isPositionBaseXYQueueIsFull())
			getFrontHistPositionBaseEstXY(&hist_position_baseX, &hist_position_baseY);
		else{
		 	hist_position_baseX = position_base[0];
			hist_position_baseY = position_base[1];
		}
		/*position_error[0] = whyconX - PositionX;
		position_error[1] = whyconY - PositionY;*/
		position_error[0] = whyconX - (hist_position_baseX + position_correction[0]);
		position_error[1] = whyconY - (hist_position_baseY + position_correction[1]);
		/* if(time_constant_xy!=(float)pidProfile->I8[PIDNAVR]/10){
			updatePosGains();
		} */

	/* if(pidProfile->P8[PIDNAVR]==6) //Desired States
	{
		print_posvariable3 = (int16_t)position_error[0]*corr_scale;//ax
		print_posvariable4 = (int16_t)position_error[1]*corr_scale;//ay
		print_posvariable1 = VelocityX*corr_scale2;//mx
		print_posvariable2 = VelocityY*corr_scale2;//my
	} */

	}else{
		if (currentTime-previousReadTime>2000000)//Didn't recieve data for 2 seconds reset integrator
		{		if(!ARMING_FLAG(ARMED))
					resetPosition();
		}

	}//read_pos

	/* if(pidProfile->P8[PIDNAVR]==7) //Desired States
	{
		print_posvariable3 = (int16_t)accel[0]*corr_scale;//ax
		print_posvariable4 = ((int16_t)(accel[1]))*corr_scale;//ay
		print_posvariable1 = (int16_t)accSumCountXY*corr_scale2;//mx
		print_posvariable2 = (int16_t)(dt*10000)*corr_scale2;//my
	} */

	for(i = 0; i<2; i++){
		if (accSumCountXY) {
			accel_prev[i] = accel[i];
			accel[i] =  (float)accSum[i] / (float)accSumCountXY;
		}else{
			accel[i] = 0;
			}

		accel[i] = accel[i] * accVelScale;
		accel[i] = constrainf(accel[i],-800,800);

		if(i==1)
			imuResetAccelerationSum(0);


		accel_correction[i] += position_error[i] * k3  * dt;
		acc_velocity[i] += position_error[i] * k2  * dt;
		position_correction[i] += position_error[i] * k1  * dt;


		//Estimation
		accel[i]+= accel_correction[i];
		velocity_increase[i] = (accel[i]) * dt; // acc * dt
		position_base[i] += (acc_velocity[i] + velocity_increase[i] *0.5f) * dt; //S = S0 + u + (1/2) at^2

		//Updated pos and vel
		est_position[i] = position_base[i]+ position_correction[i];
		acc_velocity[i] += velocity_increase[i];//v = u + at
	}

	addHistPositionBaseEstXY(position_base[0], position_base[1]);

	PositionX = (int16_t)est_position[0];
	PositionY = (int16_t)est_position[1];
	VelocityX = (int16_t)acc_velocity[0];
	VelocityY = (int16_t)acc_velocity[1];
//
//
//	    PositionX = posX;
//		PositionY = posY;
//
//
//
//		VelocityX = ((PositionX-PrevPositionX)*1000)/33;
//		VelocityY = ((PositionY-PrevPositionY)*1000)/33;
//		PrevPositionX=PositionX;
//		PrevPositionY=PositionY;
	
	/*print_posvariable3 = Po sitionX*corr_scale;
	print_posvariable4 = PositionY*corr_scale;
	print_posvariable1 = VelocityX*corr_scale2;
	print_posvariable2 = VelocityY*corr_scale2;*/

}

void updatePosGains(void)
{
	//time_constant_xy=(float)pidProfile->I8[PIDNAVR]/10;
	if (time_constant_xy == 0.0f) {
	k1 = k2 = k3 = 0.0f;
	}else{
		k1 = 3.0f / time_constant_xy;
		k2 = 3.0f / (time_constant_xy*time_constant_xy);
		k3 = 1.0f / (time_constant_xy*time_constant_xy*time_constant_xy);
		}
}


void setPos(float newX, float newY)
{
	position_base[0] = newX;
	position_base[1] = newY;
	position_correction[0] = position_correction[1] = 0;
	est_position[0] = newX;
	est_position[1] = newY;
	imuResetAccelerationSum(0);
}
void configurePosHold2(pidProfile_t *initialPidProfile){
	pidProfile = initialPidProfile;
}
