            /*
             * This file is part of Cleanflight.
             *
             * Cleanflight is free software: you can redistribute it and/or modify
             * it under the terms of the GNU General Public License as published by
             * the Free Software Foundation, either version 3 of the License, or
             * (at your option) any later version.
             *
             * Cleanflight is distributed in the hope that it will be useful,
             * but WITHOUT ANY WARRANTY; without even the implied warranty of
             * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
             * GNU General Public License for more details.
             *
             * You should have received a copy of the GNU General Public License
             * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
             */


            #include <stdbool.h>
            #include <stdint.h>
            #include <stdlib.h>
            #include <math.h>


            #include "platform.h"
            #include "debug.h"

            #include "common/maths.h"
            #include "common/axis.h"

            #include "drivers/sensor.h"
            #include "drivers/accgyro.h"
            #include "drivers/light_led.h"
            #include "drivers/gpio.h"
			#include "drivers/ranging_vl53l0x.h"
			#include "drivers/system.h"
			
            #include "sensors/sensors.h"
            #include "sensors/acceleration.h"
            #include "sensors/barometer.h"
            #include "sensors/sonar.h"

            #include "rx/rx.h"

            #include "io/rc_controls.h"
            #include "io/escservo.h"

            #include "flight/mixer.h"
            #include "flight/pid.h"
            #include "flight/imu.h"
            #include "flight/posControl.h"
            //#include "config/config.h"
            #include "config/runtime_config.h"

            #include "altitudehold.h"
			#include "posEstimate.h"

			#define corr_scale 512/100
			#define corr_scale2 1096/100
			
			#define TOLERANCE_Z 5

            int32_t setVelocity = 0;
            uint8_t velocityControl = 1;
            int32_t errorVelocityI = 0;
            int32_t altHoldThrottleAdjustment = 0;
            int32_t AltHold;
            int32_t vario = 0;                      // variometer in cm/s
			bool AltRstRequired=0;


            int32_t calculatedError=10;
            //extern uint16_t debug_d0;
            static barometerConfig_t *barometerConfig;
            static pidProfile_t *pidProfile;
            static rcControlsConfig_t *rcControlsConfig;
            static escAndServoConfig_t *escAndServoConfig;
            barometerConfig_t *barometerConfig_tmp;

            int MAX=15;

            static float buff[15];
            static int head=0;
            static int rear=-1;
            static int itemCount=0;
#ifdef LASER_TOF
            //_time_constant_z = (float)pidProfile->I8[PIDNAVR]/10;
			float  _time_constant_z=0.5;
			//bool sensor_TOF=false;

#else
			float  _time_constant_z=5;		//DD
#endif
            int32_t AltDeadband=5; //PS2
			int32_t VelConstraintDynamic=-50;
			float accZ_tmp;
            static float accZ_old = 0.0f;

         //    float _velocity_z=10;
           //  int32_t _position_z=10;

            static int32_t last_hist_position=0;
            int first_reads=0;
            int first_velocity_reads=0;
            int  ctr=0;



            void setAltitude(float new_altitude);


uint32_t baro_last_update;


//float                _time_constant_z;


float                   _k1_z;                      // gain for vertical position correction
float                   _k2_z;                      // gain for vertical velocity correction
float                   _k3_z;                      // gain for vertical accelerometer offset correction





// general variables
float              _position_base_z;             // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
float               _position_correction_z;       // sum of corrections to _position_base from delayed 1st order samples in cm
float               _accel_correction_hbf_z;
float               _velocity_z;                  // latest velocity estimate (integrated from accelerometer values) in cm/s
float               _position_error_z;            // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
float               _position_z;                  // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)

float               accel_ef_z;


int32_t VelocityZ;

int32_t PositionZ;


float ToF_Height = 0, Baro_Height =0, fused = 0, filtered = 0;
int32_t debug_variable = 0;

            void configureAltitudeHold(
                    pidProfile_t *initialPidProfile,
                    barometerConfig_t *intialBarometerConfig,
                    rcControlsConfig_t *initialRcControlsConfig,
                    escAndServoConfig_t *initialEscAndServoConfig
            )
            {
                pidProfile = initialPidProfile;
                barometerConfig = intialBarometerConfig;
                rcControlsConfig = initialRcControlsConfig;
                barometerConfig_tmp=barometerConfig;
                escAndServoConfig = initialEscAndServoConfig;
            }



            #if defined(BARO) || defined(SONAR) || defined(LASER_TOF)

            static int16_t initialThrottleHold; //DRONA
            static int32_t EstAlt=0;                // in cm
            int16_t initialThrottleHold_test; //DRONA
            int16_t debug_e1; //DRONA

            // 40hz update rate (20hz LPF on acc)
            #define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25)
            #define UPDATE_FREQUENCY (1000 * 10) //100Hz

            #define DEGREES_80_IN_DECIDEGREES 800
            /** Drone should maintian its altitude which it attins at the throttle of '1500'*/
            static void applyMultirotorAltHold(void)
            {
                static uint8_t isAltHoldChanged = 0;
                static int16_t throttle_history=0; //drona
                static int16_t sensitivity_inv = 6;
                // multirotor alt hold
               if (rcControlsConfig->alt_hold_fast_change) {
                    // rapid alt changes
                    if (ABS(rcData[THROTTLE] - initialThrottleHold) > rcControlsConfig->alt_hold_deadband) {
                        isAltHoldChanged = 1;
                        rcCommand[THROTTLE] = throttle_history +  constrain((rcData[THROTTLE] - initialThrottleHold)/sensitivity_inv,-80,80); //drona
                        if(rcData[THROTTLE]<1100)//Drona pras2
                        {
                            rcCommand[THROTTLE]= 1150;
                        }
                      } else {
                        if (isAltHoldChanged) {
                            AltHold = EstAlt;
                            isAltHoldChanged = 0;
                            if (ARMING_FLAG(ARMED)){
                            //altHoldThrottleAdjustment = throttle_history;//drona pras
                            }
                        }
                        rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);
                        throttle_history = rcCommand[THROTTLE] ; //drona

                    }
                } else {
                  if(rcData[AUX2] != 1850){       //when auto mode is off   //DD
                    // slow alt changes, mostly used for aerial photography
                    if (ABS(rcData[THROTTLE] - 1500) > rcControlsConfig->alt_hold_deadband) {
                        setVelocity = (rcData[THROTTLE] - 1500) / 4;
                        setVelocity = constrain(setVelocity,VelConstraintDynamic,60); //to descend smoothly		//DD
                        velocityControl = 1;//Switch on velocity hold
                        isAltHoldChanged=1;
                    } else{
						 velocityControl = 0;//Switch on altitude hold
						 setVelocity=0;
						 if(isAltHoldChanged){
							AltHold=EstAlt;//Record the height to be maintained
							isAltHoldChanged=0;
							}
                     }
                   }
                   else {
                     //height set by posControl
                     AltHold = getdesiredHeight();  //DD
                     velocityControl = 0;
                     setVelocity=0;
                   }
                   rcCommand[THROTTLE] = constrain(initialThrottleHold + altHoldThrottleAdjustment, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);
#ifdef LASER_TOF
                    if(rcData[THROTTLE]<1150 && isOutofRange())
#else
					if(rcData[THROTTLE]<1150)
#endif
                        {
                        rcCommand[THROTTLE]=1300;
                        }
					#ifdef LASER_TOF
					if(rcData[THROTTLE]<1150 && !isOutofRange())
					{
							if(ToF_Height<=3)
								rcCommand[THROTTLE]=1100;
					}
					#endif

				}
            }

            static void applyFixedWingAltHold(airplaneConfig_t *airplaneConfig)
            {
                // handle fixedwing-related althold. UNTESTED! and probably wrong
                // most likely need to check changes on pitch channel and 'reset' althold similar to
                // how throttle does it on multirotor

                rcCommand[PITCH] += altHoldThrottleAdjustment * airplaneConfig->fixedwing_althold_dir;
            }

            void applyAltHold(airplaneConfig_t *airplaneConfig)
            {
                if (STATE(FIXED_WING)) {
                    applyFixedWingAltHold(airplaneConfig);
                } else {
                    applyMultirotorAltHold();
                }
            }

            void updateAltHoldState(void)
            {
                // Baro alt hold activate
                if (!IS_RC_MODE_ACTIVE(BOXBARO)) {
                    DISABLE_FLIGHT_MODE(BARO_MODE);
                    return;
                }

                if (!FLIGHT_MODE(BARO_MODE)) {
                    ENABLE_FLIGHT_MODE(BARO_MODE);
                    if(rcData[AUX2] != 1850){
                    AltHold = EstAlt;//+100;//drona pras                      //DRONA
                  }else{
                      AltHold = getdesiredHeight(); }
                    initialThrottleHold = 1700;//Drona pras     //
                    errorVelocityI = 0;                               //DRONA
                    altHoldThrottleAdjustment = 0;                    //DRONA
                }
                else
                {
                }
                initialThrottleHold_test=initialThrottleHold;           //DRONA
                //debug_d0 = pidProfile->D8[PIDALT];
                debug_e1 = rcCommand[THROTTLE];                      //DRONA
            }

            void updateSonarAltHoldState(void)
            {
                // Sonar alt hold activate
                if (!IS_RC_MODE_ACTIVE(BOXSONAR)) {
                    DISABLE_FLIGHT_MODE(SONAR_MODE);
                    return;
                }

                if (!FLIGHT_MODE(SONAR_MODE)) {
                    ENABLE_FLIGHT_MODE(SONAR_MODE);
                    AltHold = EstAlt;
                    initialThrottleHold = rcData[THROTTLE];
                    errorVelocityI = 0;
                    altHoldThrottleAdjustment = 0;
                }
            }

            bool isThrustFacingDownwards(rollAndPitchInclination_t *inclination)
            {
                return ABS(inclination->values.rollDeciDegrees) < DEGREES_80_IN_DECIDEGREES && ABS(inclination->values.pitchDeciDegrees) < DEGREES_80_IN_DECIDEGREES;
            }

            /*
            * This (poorly named) function merely returns whichever is higher, roll inclination or pitch inclination.
            * //TODO: Fix this up. We could either actually return the angle between 'down' and the normal of the craft
            * (my best interpretation of scalar 'tiltAngle') or rename the function.
            */
            int16_t calculateTiltAngle(rollAndPitchInclination_t *inclination)
            {
                return MAX(ABS(inclination->values.rollDeciDegrees), ABS(inclination->values.pitchDeciDegrees));
            }

            int32_t calculateAltHoldThrottleAdjustment(int32_t vel_tmp, float accZ_tmp, float accZ_old)
            {
                int32_t result = 0;
                int32_t error;
                int32_t setVel;


                if (!isThrustFacingDownwards(&inclination)) {
                    return result;
                }

                // Altitude P-Controller
                if(!ARMING_FLAG(ARMED))//Drona alt //PS2
                    {
                      if(rcData[AUX2] != 1850)
			#ifdef LASER_TOF
						AltHold = 80;
			#else
                        AltHold= EstAlt + 20;
			#endif
                      else
                        AltHold = getdesiredHeight();
                    }
				
                if (!velocityControl) {
                    error = constrain(AltHold - EstAlt, -500, 500);
                    error = applyDeadband(error, AltDeadband); // remove small P parameter to reduce noise near zero position
					if (abs((int16_t)error)<TOLERANCE_Z)
						HeightAchieved=1;
					else
						HeightAchieved=0;
					
					calculatedError=error;


                   setVel = constrain((pidProfile->P8[PIDALT] * error / 128), VelConstraintDynamic, +50); // limit velocity to +/- 3 m/s
				   #ifdef LASER_TOF
				   if(AltHold<0){//Special condition for landing
						setVel = -35;
						if(EstAlt <= 4)
							HeightAchieved = 1;
						else
							HeightAchieved = 0;
				   }
				   #endif
                } else {
                    setVel = setVelocity;
                }

                // Velocity PID-Controller

                // P
                error = setVel - vel_tmp;
                result = constrain((pidProfile->P8[PIDVEL] * error / 32), -300, +300);

                // I
                if(ARMING_FLAG(ARMED))/*//Drona alt*/
                {
                    errorVelocityI += (pidProfile->I8[PIDVEL] * error);
                }
                else
                {
                    errorVelocityI = 0;
                }/*//Drona alt*/


                errorVelocityI = constrain(errorVelocityI, -(8192 * 300), (8192 * 300));
                result += errorVelocityI / 8192;     // I in range +/-200

                // D
                result -= constrain(pidProfile->D8[PIDVEL] * (accZ_tmp + accZ_old) / 512, -150, 150);

				//Debug
				if(pidProfile->P8[PIDNAVR]==21) //Desired States
				{
					print_posvariable3 = (int16_t)AltDeadband*corr_scale;//ax
					print_posvariable4 = (int16_t)velocityControl*10*corr_scale;//ay
					print_posvariable1 = (int16_t)AltHold*corr_scale2;//mx
					print_posvariable2 = (int16_t)EstAlt*corr_scale2;//my		
				}
				
                return result;
            }
            int16_t accalttemp;
            float Temp;


            void calculateEstimatedAltitude(uint32_t currentTime)
            {
                static uint32_t previousTime;
                uint32_t dTime;
                int32_t baroVel;
                float dt;
                float vel_acc;
                int32_t vel_tmp;
                float accZ_tmp;
                int32_t sonarAlt = -1;
                static float accZ_old = 0.0f;
                static float vel = 0.0f;
                static float accAlt = 0.0f;
                static int32_t lastBaroAlt;

                static int32_t baroAlt_offset = 0;
                float sonarTransition;

            #ifdef SONAR
                int16_t tiltAngle;
            #endif

                dTime = currentTime - previousTime;
                if (dTime < BARO_UPDATE_FREQUENCY_40HZ)
                    return;

                previousTime = currentTime;

            #ifdef BARO
                if (!isBaroCalibrationComplete()) {
                    performBaroCalibrationCycle();
                    vel = 0;
                    accAlt = 0;
                }

                //BaroAlt = baroCalculateAltitude();
            #else
                BaroAlt = 0;
            #endif

            #ifdef SONAR
                tiltAngle = calculateTiltAngle(&inclination);
                sonarAlt = sonarRead();
                sonarAlt = sonarCalculateAltitude(sonarAlt, tiltAngle);
            #endif

                if (sonarAlt > 0 && sonarAlt < 200) {
                    baroAlt_offset = BaroAlt - sonarAlt;
                    BaroAlt = sonarAlt;
                } else {
                    BaroAlt -= baroAlt_offset;
                    if (sonarAlt > 0  && sonarAlt <= 300) {
                        sonarTransition = (300 - sonarAlt) / 100.0f;
                        BaroAlt = sonarAlt * sonarTransition + BaroAlt * (1.0f - sonarTransition);
                    }
                }

                dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

                // Integrator - velocity, cm/sec
                if (accSumCount) {
                    accZ_tmp = (float)accSum[2] / (float)accSumCount;
                } else {
                    accZ_tmp = 0;
                }
                vel_acc = accZ_tmp * accVelScale * (float)accTimeSum;

                // Integrator - Altitude in cm
                accAlt += (vel_acc * 0.5f) * dt + vel * dt;
                accalttemp=lrintf(100*accAlt); //Checking how acc measures height                                                                // integrate velocity to get distance (x= a/2 * t^2)
                accAlt = accAlt * barometerConfig->baro_cf_alt + (float)BaroAlt * (1.0f - barometerConfig->baro_cf_alt);    // complementary filter for altitude estimation (baro & acc)
                vel += vel_acc;

            #ifdef DEBUG_ALT_HOLD
                debug[1] = accSum[2] / accSumCount; // acceleration
                debug[2] = vel;                     // velocity
                debug[3] = accAlt;                  // height
            #endif

                imuResetAccelerationSum(1);

            #ifdef BARO
                if (!isBaroCalibrationComplete()) {
                    return;
                }
            #endif

                if (sonarAlt > 0 && sonarAlt < 200) {
                    // the sonar has the best range
                    EstAlt = BaroAlt;
                } else {
                    EstAlt = accAlt;
                }

                baroVel = (BaroAlt - lastBaroAlt) * 1000000.0f / dTime;
                lastBaroAlt = BaroAlt;

                baroVel = constrain(baroVel, -1500, 1500);  // constrain baro velocity +/- 1500cm/s
                baroVel = applyDeadband(baroVel, 10);       // to reduce noise near zero

                // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
                // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
                vel = vel * barometerConfig->baro_cf_vel + baroVel * (1.0f - barometerConfig->baro_cf_vel);
                vel_tmp = lrintf(vel);

                // set vario
                vario = applyDeadband(vel_tmp, 5);
                 if (1)//!(ABS(rcData[THROTTLE] - initialThrottleHold) > rcControlsConfig->alt_hold_deadband))
                 {
                    altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(vel_tmp, accZ_tmp, accZ_old);
                 }//dronadrona_1200am
                Temp=pidProfile->I8[PIDALT];
                barometerConfig->baro_cf_alt=1-Temp/1000;
                accZ_old = accZ_tmp;
            }

            int32_t altitudeHoldGetEstimatedAltitude(void)
            {
                return EstAlt;
}


            /* queue implementation */


            void addHistPositionBaseEstZ(float position)
            {

            if(itemCount<MAX)
            {

             rear++;

            if(rear>=MAX)
            {

            rear=0;

            }

            buff[rear]=position;

            itemCount++;



            }

            else
            {




            if(++rear==MAX)
            {

            rear=0;

            buff[rear]=position;

            head++;

            }

            else
            {
            buff[rear]=position;

            head++;

            if(head==MAX)
            {
            head=0;
            }



            }


            }



            }

            float getFrontHistPositionBaseEstZ()
            {


            float return_value=buff[head];

            head++;

            if(head==MAX)
            {
            head=0;

            }

            itemCount--;


            return return_value;

            }


            bool isPositionBaseQueueIsFull()
            {

            return itemCount==MAX;

            }



            /* using ArduPilots Third Order Compilmentary filter */

            void apmCalculateEstimatedAltitude(uint32_t currentTime)
            {


             static uint32_t previousTime;
			 //int32_t vel_tmp;
             float dt = (currentTime - previousTime) / 1000000.0f;
             uint32_t dTime;
             dTime = currentTime - previousTime;

             if (dTime < UPDATE_FREQUENCY)
                return;

			previousTime = currentTime;

				/* Sanity Check */
				if (dTime > 2*UPDATE_FREQUENCY){//Too long. Reset things.
					imuResetAccelerationSum(1);
				}

				if(AltRstRequired && !ARMING_FLAG(ARMED))//Velocity out of bounds reset variables
					AltRst();

#if defined(BARO) && !(defined(LASER_TOF))
                checkBaro();  // check if new baro readings have arrived and use them to correct vertical accelerometer offsets.
#else
				checkReading();
#endif

				if (accSumCount) {
                        accel_ef_z =  (float)accSum[2] / (float)accSumCount;
                }else{
                        accel_ef_z  = 0;
                }

				accZ_tmp=accel_ef_z;
                accel_ef_z = accel_ef_z* accVelScale;
				accel_ef_z=constrainf(accel_ef_z,-800,800);
				//EstAlt = accel_ef_z * 100;// Value sent to viewing
                //EstAlt = _velocity_z;
                imuResetAccelerationSum(1); //Check position of this

				if(first_velocity_reads<=5){	// discarding first five readings TODO why?
                first_velocity_reads++;
				return;
				}

                _accel_correction_hbf_z += _position_error_z * _k3_z  * dt;
                _velocity_z += _position_error_z * _k2_z  * dt;
                // EstAlt=  _velocity_z ;
                _position_correction_z += _position_error_z * _k1_z  * dt;

                // calculate velocity increase adding new acceleration from accelerometers
                float velocity_increase_z;
                velocity_increase_z = (accel_ef_z + _accel_correction_hbf_z) * dt;

                // calculate new estimate of position
                _position_base_z += (_velocity_z + velocity_increase_z*0.5) * dt;

                // update the corrected position estimate
                _position_z = _position_base_z+ _position_correction_z;

                // calculate new velocity

				_velocity_z += velocity_increase_z;

				VelocityZ=lrintf(_velocity_z);

				EstAlt=lrintf(_position_z);

				//EstAlt= _velocity_z;

                // store 3rd order estimate (i.e. estimated vertical position) for future use

                addHistPositionBaseEstZ(_position_base_z);

                altHoldThrottleAdjustment = calculateAltHoldThrottleAdjustment(VelocityZ, accZ_tmp, accZ_old);

                accZ_old = accZ_tmp;
                // set vario
                vario = applyDeadband(VelocityZ, 5);
				if(abs(VelocityZ)>200)
					AltRstRequired = 1;


            //Debug althold
			 
				if(pidProfile->P8[PIDNAVR]==20) //Desired States
				{
					print_posvariable3 = (int16_t)_position_z*corr_scale;//ax
					print_posvariable4 = (int16_t)_velocity_z*corr_scale;//ay
					print_posvariable1 = (int16_t)EstAlt*corr_scale2;//mx
					print_posvariable2 = (int16_t)VelocityZ*corr_scale2;//my		
				}
				
			
			}



#ifdef LASER_TOF
            void checkReading()
            {
                uint32_t baro_update_time;
				float dt;
				float tilt = 0;
				//float tofTransition;
				static int32_t baro_offset = 0;
				static uint32_t debug_previousTime =0;
				//Baro update reading
                baro_update_time = getBaroLastUpdate();
				if( baro_update_time != baro_last_update )
				{
					dt = (float)(baro_update_time - baro_last_update) * 0.001f; // in seconds
                    Baro_Height = apmBaroCalculateAltitude();
					filtered = (0.75f * filtered) + ((1-0.75f)* Baro_Height);

					baro_last_update = baro_update_time;
                }

				//Laser sensor update reading
				if(isTofDataNew() && (!isOutofRange())){
					debug_variable = (uint16_t)(millis() - debug_previousTime);
					debug_previousTime = millis();
					ToF_Height = (float)NewSensorRange/10.0f;
					isTofDataNewflag = false;


					tilt = degreesToRadians(calculateTiltAngle(&inclination)/10);
					if(tilt < 25)
						ToF_Height *= cos_approx(tilt);
				}

				//Fusion
				if ((ToF_Height > 0 && ToF_Height < 120) && (!isOutofRange())) {
					AltDeadband = 0;
					if (ToF_Height<100&&rcData[THROTTLE]<1150){//Feather landing
						//VelConstraintDynamic=-ToF_Height/6;
						//VelConstraintDynamic = constrain(VelConstraintDynamic,-20,-5); //to
						VelConstraintDynamic=-15;
					}
					else{
						VelConstraintDynamic=-15;
					}
					baro_offset = filtered-ToF_Height;
                    correctedWithTof(ToF_Height);

//                   LED_M_ON;
//                   LED_L_OFF;

                } /* else
				//{ Baro_Height -= baro_offset;
					if (ToF_Height >= 120  && ToF_Height <= 200) {
                        //tofTransition = (200 - ToF_Height) / 100.0f;
                        tofTransition = 0.5f;
						fused = ToF_Height * tofTransition + Baro_Height * (1.0f - tofTransition);

					correctedWithBaro( fused, dt);
                } */
				else{

//					 LED_L_ON;
//					 LED_M_OFF;

					AltDeadband = 5;
					correctedWithBaro(Baro_Height-baro_offset, dt);

				}
            }
#endif


			void checkBaro()
            {

                uint32_t baro_update_time;

                // calculate time since last baro reading (in ms)

               // EstAlt=10;
                baro_update_time = getBaroLastUpdate();

              // EstAlt= baro_update_time;
                if( baro_update_time != baro_last_update ) {

               // EstAlt=300;
                    const float dt = (float)(baro_update_time - baro_last_update) * 0.001f; // in seconds
                    // call correction method
                    correctedWithBaro( apmBaroCalculateAltitude(), dt);
                baro_last_update = baro_update_time;
                }

            }

            void correctedWithBaro(float baroAlt, float dt)
            {

                   //  EstAlt=baroAlt;

                     if( dt > 0.5f ) {
                         return;
                        }

                      //LED0_TOGGLE;

                        float hist_position_base_z=0;

                       if( first_reads <= 10 ) {
						setAltitude(baroAlt);

                         // if(first_reads=0)
                            first_reads++;
                          //EstAlt=first_reads;
                      }

                        if (isPositionBaseQueueIsFull()) {
                            hist_position_base_z = getFrontHistPositionBaseEstZ();


                        } else {
                            hist_position_base_z = _position_base_z;
                        }

                        // calculate error in position from baro with our estimate
                        _position_error_z = baroAlt - (hist_position_base_z + _position_correction_z);

						// _position_error_z =0 ;
#ifdef LASER_TOF
						if(_time_constant_z != 5.0f)
						{
							_time_constant_z = 5;
							updateGains();
						}
#endif
            }

#ifdef LASER_TOF
			void correctedWithTof(float ToF_Height){
				if( !ARMING_FLAG(ARMED) )
				{
					setAltitude(ToF_Height);
				}
				_position_error_z = ToF_Height - EstAlt;
				 if(_time_constant_z!=0.5f){
					_time_constant_z = 0.5;
					updateGains();
				} 
			}
#endif
			void updateGains()
            {
				//_time_constant_z = (float)pidProfile->I8[PIDNAVR]/10;
             if (_time_constant_z == 0.0f) {
                    _k1_z = _k2_z = _k3_z = 0.0f;
                }else{
                    _k1_z = 3.0f / _time_constant_z;
                    _k2_z = 3.0f / (_time_constant_z*_time_constant_z);
                    _k3_z = 1.0f / (_time_constant_z*_time_constant_z*_time_constant_z);
                }

            }


			void setAltitude(float new_altitude)

            {
                _position_base_z = new_altitude;
                _position_correction_z = 0;
                _position_z = new_altitude; // _position = _position_base + _position_correction
                last_hist_position=0; // reset z history to avoid fake z velocity at next baro calibration (next rearm)
                imuResetAccelerationSum(1);
			}

            int32_t getSetVelocity(void){
           // return VelocityZ;

            return _velocity_z;
            }

			void AltRst(void){
				_velocity_z = 0;
				imuResetAccelerationSum(1);
				AltRstRequired=0;
			}



             float getTimeConstant(){
				return _time_constant_z;
			}


			#endif
