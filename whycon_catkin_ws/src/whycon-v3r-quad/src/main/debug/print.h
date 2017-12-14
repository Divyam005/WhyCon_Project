/*
 Pluto API V.0.1
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

class printManager {
private:
	uint32_t CurrentTimeInt;
	uint32_t LoopTimeInt;
	uint32_t CurrentTimeFloat;
	uint32_t LoopTimeFloat;
	uint32_t CurrentTime;
	uint32_t LoopTime;
	uint32_t lTI;
	uint32_t lTF;
	uint32_t lTM;
public:

	printManager();

	//default frequency is 100 milliseconds with constrain from 20 milliseconds to 5 seconds, yep here you can default frequency.

	void pInt(const char* tag, int32_t number, uint32_t frequency=100);

	void pFloat(const char* tag, double number, uint8_t pricision, uint32_t frequency=100);

	void p(const char* msg, uint32_t frequency=100);

};

#define printInt(a, b) ({ static printManager pr; pr.pInt(a, b); } )
#define printFloat(a,b,c) ({ static printManager pr; pr.pFloat(a,b,c); } )
#define print(a) ({ static printManager pr; pr.p(a); } )
#define lprintInt(a, b,c) ({ static printManager pr; pr.pInt(a, b,c); } )
#define lprintFloat(a,b,c,d) ({ static printManager pr; pr.pFloat(a,b,c,d); } )
#define lprint(a,b) ({ static printManager pr; pr.p(a,b); } )



void printRG(double value, uint32_t frequency=100);
void printBG(double value, uint32_t frequency=100);
void printGG(double value, uint32_t frequency=100);



#ifdef __cplusplus
}
#endif
