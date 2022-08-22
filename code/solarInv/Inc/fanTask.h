/*
 * fanTask.h
 *
 *  Created on: 19 feb. 2018
 *      Author: wvdv2
 */

#ifndef FANTASK_H_
#define FANTASK_H_

#include "stm32f4xx_hal.h"

void fanTaskInit(GPIO_TypeDef* aGPIO,uint16_t aPin,float* aTemp);
void fanTaskInitWithVal(GPIO_TypeDef* aGPIO,uint16_t aPin,float* aTemp, float* aValue);
void fanTask(void);
void fanSetMinOnTime(int);
void fanSetOnTemperature(float);
void fanSetOffTemperature(float);
void fanSetTripValue(float);
void fanOnOff(int);
void fanForceOff(void);
void fanReset(void);

#endif /* FANTASK_H_ */
