#include "fanTask.h"
#include "stm32f4xx_hal.h"
#include "main.h"

typedef enum{
	FAN_IDLE,
	FAN_ON,
	FAN_OFF,
	FAN_GOTO_ON,
	FAN_GOTO_OFF,
	FAN_DISABLE,
}fanState_t;

typedef struct{
	uint32_t minOnTime;
	uint32_t lastOnTime;
	float onTemp;
	float offTemp;
	float tripValue;
	float* theTemp;
	float* aValue;
	GPIO_TypeDef* GPIO;
	uint16_t pin;
	fanState_t state;
}fanStruct_t;


static fanStruct_t aFan;

void fanTaskInit(GPIO_TypeDef* aGPIO,uint16_t aPin,float* aTemp){
	aFan.GPIO = aGPIO;
	aFan.pin = aPin;
	aFan.state = FAN_IDLE;
	aFan.lastOnTime = HAL_GetTick();
	aFan.theTemp = aTemp;
	aFan.state = FAN_OFF;
}

void fanTaskInitWithVal(GPIO_TypeDef* aGPIO,uint16_t aPin,float* aTemp, float* aValue){
	fanTaskInit(aGPIO,aPin,aTemp);
	aFan.aValue = aValue;
}

void fanSetTripValue(float aTripValue){
	aFan.tripValue = aTripValue;
}

void fanSetOnTemperature(float aTemp){
	aFan.onTemp = aTemp;
}

void fanSetOffTemperature(float aTemp){
	aFan.offTemp = aTemp;
}

void fanSetMinOnTime(int aTime){
	aFan.minOnTime = aTime;
}

void fanForceOff(void){
	aFan.state = FAN_DISABLE;
	HAL_GPIO_WritePin(aFan.GPIO,aFan.pin,GPIO_PIN_RESET);
}

void fanReset(void){
	aFan.state = FAN_GOTO_OFF;
}

void fanOnOff(int onOrOff){
	if(onOrOff){
		aFan.state = FAN_GOTO_ON;
	}else{
		aFan.state = FAN_GOTO_OFF;
	}
}

void fanTask(void){
	uint32_t nowTick=HAL_GetTick();
	switch(aFan.state){
		case FAN_ON:
			if(aFan.aValue){
				if(*(aFan.aValue)>aFan.tripValue){
					aFan.lastOnTime=nowTick;
				}
			}
			if(*(aFan.theTemp)<aFan.offTemp){
				if(nowTick-aFan.lastOnTime>aFan.minOnTime){
					aFan.state = FAN_GOTO_OFF;
				}
			}else{
				aFan.lastOnTime=nowTick;
			}
		break;
		case FAN_OFF:
			if(*(aFan.theTemp)>aFan.onTemp){
				aFan.state = FAN_GOTO_ON;
			}
			if(aFan.aValue){
				if(*(aFan.aValue)>aFan.tripValue){
					aFan.state = FAN_GOTO_ON;
				}
			}
			break;
		case FAN_GOTO_ON:
			HAL_GPIO_WritePin(aFan.GPIO,aFan.pin,GPIO_PIN_SET);
			aFan.state=FAN_ON;
			aFan.lastOnTime=nowTick;
		break;
		case FAN_GOTO_OFF:
			HAL_GPIO_WritePin(aFan.GPIO,aFan.pin,GPIO_PIN_RESET);
			aFan.state=FAN_OFF;
		break;
		default:
		break;
	}
}

