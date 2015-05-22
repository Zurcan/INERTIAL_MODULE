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



CAN_HandleTypeDef CanHandle;


char makeFramedCANMessage(int *currentArrIndex, uint8_t *outArr, uint8_t *mode);
char parseArray(uint8_t *inArr, uint16_t inArrLength, int arrType, uint8_t *outArr, uint16_t *outArrLength);


//int var2ArrConverter(char * inpArr, int arrSize, char* outArr);
#endif /* INERTIALMODULE_INC_MAIN_H_ */
