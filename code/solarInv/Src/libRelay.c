/*
 * libRelay.c
 *
 *  Created on: 13 feb. 2018
 *      Author: wvdv2
 */


#include "libRelay.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "stdbool.h"

void openPrechargeRelay(void){
	HAL_GPIO_WritePin(GRIDRELAY_1_GPIO_Port,GRIDRELAY_1_Pin,0);
}


void closePrechargeRelay(void){
	HAL_GPIO_WritePin(GRIDRELAY_1_GPIO_Port,GRIDRELAY_1_Pin,1);
}


void openBridgeRelay(void){
	HAL_GPIO_WritePin(GRIDRELAY_0_GPIO_Port,GRIDRELAY_0_Pin,0);
}

void closeBridgeRelay(void){
	HAL_GPIO_WritePin(GRIDRELAY_0_GPIO_Port,GRIDRELAY_0_Pin,1);
}


bool isBridgeRelayClosed(void){
	return HAL_GPIO_ReadPin(GRIDRELAY_0_GPIO_Port,GRIDRELAY_0_Pin);
}

bool isPrechargeRelayClosed(void){
	return HAL_GPIO_ReadPin(GRIDRELAY_1_GPIO_Port,GRIDRELAY_1_Pin);
}
