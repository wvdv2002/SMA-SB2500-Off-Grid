/*
 * controlLoop.h
 *
 *  Created on: 11 aug. 2017
 *      Author: wvdv2
 */

#ifndef CONTROLLOOP_H_
#define CONTROLLOOP_H_

#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "datatypes.h"


#define IINSAMPLEDEPTH 60

void controlLoop(void);
void setupControlLoop(void);
void setDutyCycle(float);
void calculateInputs(void);
void calculateOutputs(void);
void calculateState(void);
void calculateCapacitorVoltage(void);
void controlInit(void);
float getDutyCycle(void);
void setDutyCycle(float);
void controlSetUCapMax(float UCapMax);
void controlSetSetPoints(void* buffer);
void controlSetUCapSet(float UCapSet);
void controlSetR(float R);
void controlGotoFixedDutyCycleState(float);
void controlGetOutOfSafety(void);
void controlGetOutOfOvervoltage(void);
void controlStopRunning(void);
void controlStopAndReset(void);
void calculateFilters(void);
int getSyncTime(void);
void controlGotoBridgeMalfunction(void);
void toggleCapRealVoltage(void);
float getRNow(void);
void controlSetFBR(float);

solarFullControlState_t* getControlState(void);
solarSetPoints_t* getSetPoint(void);
solarAuxInputs_t* getAuxInputs(void);




#endif /* CONTROLLOOP_H_ */
