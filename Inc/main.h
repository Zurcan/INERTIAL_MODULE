/*
 * main.h
 *
 *  Created on: 13 џэт. 2015 у.
 *      Author: User
 */

#ifndef INERTIALMODULE_INC_MAIN_H_
#define INERTIALMODULE_INC_MAIN_H_
//#define __cplusplus
#include "stdbool.h"
#include "stdlib.h"

#define CoreFreq 30000000
int Freq,FreqMDAD,FreqMDLU,FreqMDUS,FreqPresc;
typedef enum
{
	MDLUFrequency = 0,
	MDUSFrequency = 1,
	MDADFrequency = 2
}freqTypes;
typedef struct
{
	uint32_t totalFrequency;
	uint32_t MDLUFrequency;
	uint32_t MDUSFrequency;
	uint32_t MDADFrequency;
	uint32_t MDLUtickCounter;
	uint32_t MDUStickCounter;
	uint32_t MDADtickCounter;
	freqTypes currentFreqType;
}IMfrequencies;

CAN_HandleTypeDef CanHandle;
TIM_Base_InitTypeDef Timer;
IMfrequencies IMfreqs;

char makeFramedCANMessage(int *currentArrIndex, uint8_t *outArr, uint8_t *mode);
char parseArray(uint8_t *inArr, uint16_t inArrLength, int arrType, uint8_t *outArr, uint16_t *outArrLength);
char freqQueue();					//procedure used to get which of inertial data is used to send via can on next step
char checkFrequencies();

//int var2ArrConverter(char * inpArr, int arrSize, char* outArr);
#endif /* INERTIALMODULE_INC_MAIN_H_ */
