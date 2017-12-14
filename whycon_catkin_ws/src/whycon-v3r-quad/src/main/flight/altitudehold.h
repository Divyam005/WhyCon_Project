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

#ifdef __cplusplus
 extern "C" {
#endif 

#include "io/escservo.h"
#include "io/rc_controls.h"
#include "flight/pid.h"

#include "sensors/barometer.h"

extern int32_t AltHold;
extern int32_t vario;
extern int16_t accalttemp;
extern barometerConfig_t *barometerConfig_tmp;
//extern bool AltRstRequired;

void configureAltitudeHold(pidProfile_t *initialPidProfile, barometerConfig_t *intialBarometerConfig, rcControlsConfig_t *initialRcControlsConfig, escAndServoConfig_t *initialEscAndServoConfig);
void applyAltHold(airplaneConfig_t *airplaneConfig);
void updateAltHoldState(void);
void updateSonarAltHoldState(void);

int32_t altitudeHoldGetEstimatedAltitude(void);

int32_t getSetVelocity(void);

void AltRst(void);
extern int32_t debug_variable;
// new Additions for Ardupilots althold

//uint32_t baro_last_update;


//float                _time_constant_z;


//float                   _k1_z;                      // gain for vertical position correction
//float                   _k2_z;                      // gain for vertical velocity correction
//float                   _k3_z;                      // gain for vertical accelerometer offset correction





// general variables
//float              _position_base_z;             // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
//float               _position_correction_z;       // sum of corrections to _position_base from delayed 1st order samples in cm
//float               _accel_correction_hbf_z;
//float               _velocity_z;                  // latest velocity estimate (integrated from accelerometer values) in cm/s
//float               _position_error_z;            // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
//float               _position_z;                  // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)

//float               accel_ef_z;


//int32_t VelocityZ;

//int32_t PositionZ;
void correctedWithTof(float ToF_Height);


void correctedWithBaro(float baroAlt, float dt);

void checkReading();
void checkBaro();

void updateGains();

void updateTimeConstantandGains(uint8_t timeConstant);


float getTimeConstant();



void apmCalculateEstimatedAltitude(uint32_t currtime);

extern float ToF_Height;
#ifdef __cplusplus
 }
#endif 