//
// Created by Drona Aviation on 7/21/2016.
//


#ifdef __cplusplus
 extern "C" {
#endif 

#define ASCEND 1
#define PITCHING 2
#define SLOWDOWNANDEXIT 3
#define HOLD 4
#define PITCHINGPOS 5
#define SLOWDOWNANDEXITPOS 6
#define HOLDPOS 7

extern uint16_t tempstate;
extern uint16_t tstate;

void flip(uint32_t reset);


#ifdef __cplusplus
 }
#endif 