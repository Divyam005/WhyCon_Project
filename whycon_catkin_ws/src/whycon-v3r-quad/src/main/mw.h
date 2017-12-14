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

extern int16_t magHold;

void applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta);
void handleInflightCalibrationStickPosition();

void mwDisarm(void);
void mwArm(void);

void loop(void);




extern uint32_t total_time;

extern uint32_t previous_time;

extern uint32_t current_time;

extern uint32_t mode_checker;

extern bool isCalibrated;
extern int16_t headFreeModeHold;

#ifdef __cplusplus
 }
#endif 