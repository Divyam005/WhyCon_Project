#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#include "common/maths.h"


#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/system.h"

#include "io/serial_msp.h"

#include "print.h"



uint8_t checksum;

    uint32_t CurrentTimeRG=0;
	uint32_t LoopTimeRG=0;
	uint32_t CurrentTimeBG=0;
	uint32_t LoopTimeBG=0;
	uint32_t CurrentTimeGG=0;
	uint32_t LoopTimeGG=0;
	uint32_t lTRG=0;
	uint32_t lTBG=0;
	uint32_t lTGG=0;


void serializeString(const char* msg)
{
    for (uint8_t i = 0; i < strlen(msg); i++) {
        serialize8Debug(msg[i]);
        checksum ^= (msg[i]);
    }
}

void debugPrint(const char* msg)
{
    // if(isDebugPrintEnabled){
    checksum = 0;
    serialize8Debug('$');
    serialize8Debug('D');
    serialize8Debug(strlen(msg));
    checksum ^= strlen(msg);
    serializeString(msg);
    serialize8Debug(checksum);
    // }
}

void debugPrint(const char* msg, double number, uint8_t digits)
{
    // if(isDebugPrintEnabled){
    checksum = 0;
    serialize8Debug('$');
    serialize8Debug('D');

    if (isnan(number)) {
        const char* err = "not a number";
        uint8_t data_size = strlen(msg) + strlen(err);
        serialize8Debug(data_size);
        checksum ^= data_size;
        serializeString(msg);
        serialize8Debug('\t');
        checksum ^= '\t';
        serializeString(err);
        serialize8Debug(checksum);
        return;
    }
    if (isinf(number)) {
        const char* err = "infinite";
        uint8_t data_size = (uint8_t) (strlen(msg) + strlen(err));
        serialize8Debug(data_size);
        checksum ^= data_size;
        serializeString(msg);
        serialize8Debug('\t');
        checksum ^= '\t';
        serializeString(err);
        serialize8Debug(checksum);
        return;
    }
    if (number > (double) 4294967040.0 || number < (double) -4294967040.0) {

        const char* err = "overflow";
        uint8_t data_size = strlen(msg) + strlen(err) + 1;
        serialize8Debug(data_size);
        checksum ^= data_size;
        serializeString(msg);
        serialize8Debug('\t');
        checksum ^= '\t';
        serializeString(err);
        serialize8Debug(checksum);
        return;
    }

    bool isNumNeg = false;
    if (number < (double) 0.0) {
        number = -number;
        isNumNeg = true;
    }

    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i)
        rounding /= (double) 10.0;
    number += rounding;

    uint32_t int_part = (uint32_t) number;
    double remainder = number - (double) int_part;

    char buf[8 * sizeof(int_part) + 1];
    char *str = &buf[sizeof(buf) - 1];
    *str = '\0';
    do {
        char digit = int_part % 10;
        int_part /= 10;
        *--str = digit < 10 ? digit + '0' : digit + 'A' - 10;
    } while (int_part);

    uint8_t data_size = (uint8_t) strlen(msg) + (uint8_t) strlen(str) + (isNumNeg ? 1 : 0) + (
            digits > 0 ? (digits + 1) : 0) + 1;
    serialize8Debug(data_size);
    checksum ^= data_size;
    serializeString(msg);
    serialize8Debug('\t');
    checksum ^= '\t';

    if (isNumNeg) {
        serialize8Debug('-');
        checksum ^= '-';
    }

    serializeString(str);

    if (digits > 0) {
        serialize8Debug('.');
        checksum ^= '.';
    }
    while (digits-- > 0) {
        remainder *= (double) 10.0;
        uint8_t toPrint = (uint8_t) (remainder);
        toPrint = toPrint < 10 ? toPrint + '0' : toPrint + 'A' - 10;
        serialize8Debug(toPrint);
        checksum ^= toPrint;
        remainder -= toPrint;
    }
    serialize8Debug(checksum);
    //  }
}

void debugPrint(const char* msg, int32_t number)
{

    checksum = 0;
    serialize8Debug('$');
    serialize8Debug('D');

    bool isNumNeg = false;
    if (number < 0) {
        number = -number;
        isNumNeg = true;
    }
    char buf[8 * sizeof(number) + 1];
    char *str = &buf[sizeof(buf) - 1];

    *str = '\0';
    do {
        char digit = number % 10;
        number /= 10;
        *--str = digit < 10 ? digit + '0' : digit + 'A' - 10;
    } while (number);

    uint8_t data_size = (uint8_t) strlen(msg) + (uint8_t) strlen(str) + (isNumNeg ? 1 : 0) + 1;
    serialize8Debug(data_size);
    checksum ^= data_size;

    serializeString(msg);
    serialize8Debug('\t');
    checksum ^= '\t';

    if (isNumNeg) {
        serialize8Debug('-');
        checksum ^= '-';
    }
    serializeString(str);
    serialize8Debug(checksum);

}

printManager::printManager()
       {

	CurrentTimeInt=0;
		 LoopTimeInt=0;
	 CurrentTimeFloat=0;
		 LoopTimeFloat=0;
	 CurrentTime=0;
		 LoopTime=0;
		lTI=0;
		  lTF=0;
		  lTM=0;
       }


void printManager::pInt(const char* msg, int32_t number,uint32_t frequency)
    {

	if(lTI==0)
	{
		lTI=constrain(frequency, 20, 5000);
		lTI=lTI*1000;
	}

	CurrentTimeInt = micros();
           if ((int32_t) (CurrentTimeInt - LoopTimeInt) >= 0) {
        	   LoopTimeInt = CurrentTimeInt + lTI;

               debugPrint(msg, number);
               // isDebugPrintEnabled = true;
           }

       }

       void printManager::pFloat(const char* msg, double number, uint8_t digits,uint32_t frequency)
       {


    	   if(lTF==0)
    	   	{
    		   lTF=constrain(frequency, 20, 5000);
    		   lTF=lTF*1000;
    	   	}

    	   CurrentTimeFloat = micros();
           if ((int32_t) (CurrentTimeFloat - LoopTimeFloat) >= 0) {
        	   LoopTimeFloat = CurrentTimeFloat + lTF;

               debugPrint(msg, number, digits);
               // isDebugPrintEnabled = true;
           }

       }

       void printManager::p(const char* msg,uint32_t frequency)
       {



    	   if(lTM==0)
    	   	{
    		   lTM=constrain(frequency, 20, 5000);
    		   lTM=lTM*1000;
    	   	}

    	   CurrentTime = micros();
           if ((int32_t) (CurrentTime - LoopTime) >= 0) {
        	   LoopTime = CurrentTime + lTM;

               debugPrint(msg);
               // isDebugPrintEnabled = true;
           }

       }

       void printRG(double value, uint32_t frequency)
       {



    	   if(lTRG==0)
    	      	   	{
    		   lTRG=constrain(frequency, 20, 5000);
    		   lTRG=lTRG*1000;
    	      	   	}

    	      	   CurrentTimeRG = micros();
    	             if ((int32_t) (CurrentTimeRG - LoopTimeRG) >= 0) {
    	            	 LoopTimeRG = CurrentTimeRG + lTRG;

    	                 debugPrint("~R", value, 3);
    	                 // isDebugPrintEnabled = true;
    	             }




       }



       void printBG(double value, uint32_t frequency)
             {



          	   if(lTBG==0)
          	      	   	{
          		 lTBG=constrain(frequency, 20, 5000);
          		lTBG=lTBG*1000;
          	      	   	}

          	      	   CurrentTimeBG = micros();
          	             if ((int32_t) (CurrentTimeBG - LoopTimeBG) >= 0) {
          	            	LoopTimeBG = CurrentTimeBG + lTBG;

          	                 debugPrint("~B", value, 3);
          	                 // isDebugPrintEnabled = true;
          	             }




             }


       void printGG(double value, uint32_t frequency)
                   {



                	   if(lTGG==0)
                	      	   	{
                		   lTGG=constrain(frequency, 20, 5000);
                		   lTGG=lTGG*1000;
                	      	   	}

                	      	   CurrentTimeGG = micros();
                	             if ((int32_t) (CurrentTimeGG - LoopTimeGG) >= 0) {
                	            	LoopTimeGG = CurrentTimeGG + lTGG;

                	                 debugPrint("~G", value, 3);
                	                 // isDebugPrintEnabled = true;
                	             }




                   }


