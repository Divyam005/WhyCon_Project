//
// Created by Drona Aviation on 7/21/2016.
//


#include <stdbool.h>
#include <stdint.h>


#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"



#include "sensors/sensors.h"
#include "sensors/gyro.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"


#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"

#include "io/gps.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "acrobats.h"

uint16_t tempstate=0;
uint16_t tstate=0;

void flip(uint32_t reset)
{

    /**********************
    Flip States

    **********************/
    static uint32_t state=1;
    static uint32_t timeon=0;
    static uint32_t stateon = 0;
    uint32_t stateoff;
    uint32_t timeoff;

    if (reset==0)//when AUX1 is on
    {
        switch (state)
        {
            case ASCEND ://State=1
                tstate = state;
                ACTIVATE_RC_MODE(BOXBARO);
                ENABLE_FLIGHT_MODE(ANGLE_MODE);


                if (rcData[THROTTLE]>=1700)//&&vbatscaled>=350)
                {
                    DEACTIVATE_RC_MODE(BOXBARO); //deactivate baromode to ensure greater vertical velocity than when the mode is on.
                    if (tempstate == 5) {tempstate = 1;}

                    if (inclination.values.pitchDeciDegrees <=100 && inclination.values.rollDeciDegrees >= -250 && inclination.values.rollDeciDegrees <= 250)
                    {
                        currentControlRateProfile->rates[FD_PITCH] = 85;
                        tempstate=11;
                        if (rcData[THROTTLE]>=1900&&vario>=150 )//ascend till throttle is greater than 1900
                            {


                            if (tempstate == 1) {tempstate = 2;}

                            //Execute Backflip
                            DISABLE_FLIGHT_MODE(ANGLE_MODE);

                            rcData[THROTTLE]=1100;
                            rcData[PITCH]=1000;        //give initial pitch to start flip
                            state = 2;
                            if (tempstate == 2) {tempstate = 3;}
                            stateon = millis();
                        }
                    }
                    else
                    {//Execute Forward flip
                        if (rcData[THROTTLE]>=1900&&vario>=120)
                        {
                            state = 5;
                            if (tempstate == 2) {tempstate = 4;}
                            stateon = millis();
                        }
                    }
                }


                break;

            case PITCHING ://State=2 continue fliping of the drone
                tstate = state;

                //if (FLIGHT_MODE(BARO_MODE)){
                DEACTIVATE_RC_MODE(BOXBARO);
                //}

                DISABLE_FLIGHT_MODE(ANGLE_MODE);
                rcData[THROTTLE]=1100;
                rcData[PITCH]=1000;

                if (inclination.values.pitchDeciDegrees > 450)
                {
                    state = 3;
                    stateon = millis();
                }
                else
                {
                    state = 2;
                    stateoff = millis();
                    if ((stateoff-stateon) >= 2000)
                    {
                        state = 1;
                        stateon = 0;
                    }
                }
                break;

            case SLOWDOWNANDEXIT : //state = 3 slowdown down when the angle of the drone is >225 degrees
                tstate = state;
                if (inclination.values.pitchDeciDegrees > 450)
                {
                    DEACTIVATE_RC_MODE(BOXBARO);
                    //}

                    DISABLE_FLIGHT_MODE(ANGLE_MODE);
                    rcData[THROTTLE]=1100;
                    rcData[PITCH] = 1680;
                    stateoff = millis();
                    if ((stateoff-stateon) >= 2000)
                    {
                        state = 1;
                        stateon=0;
                    }

                }
                else
                {//rate command of .8 rad/s
                    if (inclination.values.pitchDeciDegrees >= 0 && inclination.values.pitchDeciDegrees <= 450 && inclination.values.rollDeciDegrees >= -250 && inclination.values.rollDeciDegrees <= 250)
                    {//exiting after one flip

                        ACTIVATE_RC_MODE(BOXBARO);
                        ENABLE_FLIGHT_MODE(ANGLE_MODE);
                        if(vbatscaled>370)
                            rcData[THROTTLE]=1500;
                        else
                            rcData[THROTTLE]=1700;

                        rcData[PITCH]=1500;
                        timeon = millis();
                        state=4;
                        tempstate=5;
                    }
                }
                break;

            case HOLD :
                tstate = state;
                ACTIVATE_RC_MODE(BOXBARO);
                ENABLE_FLIGHT_MODE(ANGLE_MODE);

                rcData[THROTTLE]=2000;
                rcData[PITCH]=1500;
                currentControlRateProfile->rates[FD_PITCH] = 75;

                timeoff = millis();
                if ((timeoff-timeon)<=1000)
                {

                    state = 4;
                }
                else
                {
                    state = 1;
                    tempstate = 5;
                    stateon=0;
                }
                break;

            case PITCHINGPOS ://State=2 continue fliping of the drone
                tstate = state;
                DISABLE_FLIGHT_MODE(ANGLE_MODE);
                rcData[THROTTLE]=1100;
                rcData[PITCH]=2000;

                if (inclination.values.pitchDeciDegrees < -450)
                {
                    state = 6;
                    stateon = millis();
                }
                else
                {
                    state = 5;
                    stateoff = millis();
                    if ((stateoff-stateon) >= 2000)
                    {
                        state = 1;
                        stateon = 0;
                    }
                }
                break;

            case SLOWDOWNANDEXITPOS : //state = 3 slowdown down when the angle of the drone is >270 degrees
                tstate = state;
                if (inclination.values.pitchDeciDegrees < -450)
                {
                    DEACTIVATE_RC_MODE(BOXBARO);
                    DISABLE_FLIGHT_MODE(ANGLE_MODE);
                    rcData[THROTTLE]=1100;
                    rcData[PITCH] = 1320; //giving negative pitch rate to slow down rate of pitch
                    stateoff = millis();
                    if ((stateoff-stateon) >= 2000)
                    {
                        state = 1;
                        stateon = 0;
                    }
                }
                else
                {
                    if (inclination.values.pitchDeciDegrees >= -450 && inclination.values.pitchDeciDegrees <= -50 && inclination.values.rollDeciDegrees >= -250 && inclination.values.rollDeciDegrees <= 250)
                    {

                        ACTIVATE_RC_MODE(BOXBARO);
                        ENABLE_FLIGHT_MODE(ANGLE_MODE);
                        if(vbatscaled>370)
                            rcData[THROTTLE]=1500;
                        else
                            rcData[THROTTLE]=1700;
                        rcData[PITCH]=1500;
                        timeon = millis();
                        state=7;
                        tempstate = 5;
                    }
                }
                break;

            case HOLDPOS :
                tstate = state;
                ACTIVATE_RC_MODE(BOXBARO);
                ENABLE_FLIGHT_MODE(ANGLE_MODE);

                rcData[THROTTLE]=2000;
                rcData[PITCH]=1500;
                currentControlRateProfile->rates[FD_PITCH] = 75;

                timeoff = millis();
                if ((timeoff-timeon)<=1000)
                {

                    state = 7;
                }
                else
                {
                    state = 1;
                    tempstate = 5;
                }
                break;
        }
    }
    else //reseting the state when the AUX1 is off. This is done so that state is 1 again when next flip is to be executed and AUX1 is on.
    {
        state=1;
        timeon=0;
        stateon=0;
        tstate=1;
        currentControlRateProfile->rates[FD_PITCH] = 75;

    }
}