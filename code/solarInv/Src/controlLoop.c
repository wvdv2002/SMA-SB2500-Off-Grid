/*
 * controlLoop.c
 *
 *  Created on: 11 aug. 2017
 *      Author: wvdv2
 */
#include "HardwareDefines.h"
#include "controlLoop.h"
#include "controlMonitor.h"
#include "filterCoeff.h"
#include "tm_stm32_filters.h"
#include "PID.h"
#include "temperature/Thermistor.h"
#include "libRelay.h"
#include "fanTask.h"
//#include "syncLoop.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
adc3Results_t adc3Results[FILTERDEPTH];
adc2Results_t adc2Results[FILTERDEPTH];
adc1Results_t adc1Results[FILTERDEPTH];

solarFullControlState_t systemState;
solarSetPoints_t setPoint;
solarSetPoints_t setPointCopy;
solarAuxInputs_t auxInputs;
PID_t capacitorPid;

bool updateControl = false;
bool updateSetPoint = false;
uint32_t closeRelayTime=0;

void calculateTemperature(void);

#define DCRATIO (float) 0.999
TM_FILTER_FIR_F32_t* currentFIR;
TM_FILTER_FIR_F32_t* uCapFIR;
extern TIM_HandleTypeDef htim4;

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))

void controlLoop(void){
//	HAL_GPIO_WritePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin, GPIO_PIN_SET);

	if(updateSetPoint){
		memcpy(&setPoint,&setPointCopy,sizeof(solarSetPoints_t));
//		pidSetTunings(&capacitorPid,setPoint.UCapP,setPoint.UCapI,setPoint.UCapD,P_ON_E);
//		auxInputs.Vpp = syncCalculateVpp();
		updateSetPoint = false;
	}
	calculateInputs();
	calculateFilters();
	calculateState();

	switch(systemState.controlState.controlTaskState){
	case CONTROL_IDLE:
		if(HAL_GetTick()>10000){
			systemState.controlState.controlTaskState = CONTROL_WAITFOR_SAFESTART;
		}
		setDutyCycle(0.0);
		break;
	case CONTROL_WAITFOR_SAFESTART:
		systemState.controlState.controlTaskState = CONTROL_RUNNING;
		break;

	case CONTROL_RUNNING:
		setDutyCycle(getDutyCycle());
		break;

	case CONTROL_FIXED_DUTY_CYLE:
//		setDutyCycle(getDutyCycle());
		break;

	case CONTROL_STOPANDRESET:
		setDutyCycle(0.5);
		break;

	case CONTROL_SAFETY:
	case CONTROL_OVERVOLTAGE:
	case CONTROL_STOPPING:
	case CONTROL_OVERTEMPERATURE:
//		if(systemState.controlState.UCap<(float)7.0){
//			closeRelayTime = HAL_GetTick();
//			closeBridgeRelay();
//			systemState.controlState.controlTaskState++;
//		}
		break;
	case CONTROL_SAFETY_RELAYOFF:
		if(HAL_GetTick()-closeRelayTime>60*1000){ //restart after a minute automatically.
			systemState.controlState.controlTaskState = CONTROL_IDLE;
		}
		break;
	case CONTROL_OVERVOLTAGE_RELAYOFF:
	case CONTROL_OVERTEMPERATURE_RELAYOFF:
	case CONTROL_STOPPING_RELAYOFF:
	case CONTROL_POWERLOSS_RELAYOFF:
	case CONTROL_BRIDGEMALFUNCTION_RELAYOFF:
		if(HAL_GetTick()-closeRelayTime>200){
			setDutyCycle(0.0);
		}
		break;
	case CONTROL_STOPANDRESET_RELAYOFF:
		if((HAL_GetTick()-closeRelayTime)>5000){
			NVIC_SystemReset();
		}
		break;
	default:
		systemState.controlState.controlTaskState = CONTROL_IDLE;
	}

//	HAL_GPIO_WritePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin, GPIO_PIN_RESET);
}



void checkVACSupplyState(void){
	if(__HAL_TIM_GET_FLAG(&htim4,TIM_FLAG_UPDATE)){
		__HAL_TIM_CLEAR_FLAG(&htim4,TIM_FLAG_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim4,0);
		if(systemState.controlState.controlTaskState != CONTROL_POWERLOSS_RELAYOFF){
//			HAL_GPIO_WritePin(TEST_PIN_GPIO_Port, TEST_PIN_Pin, GPIO_PIN_SET);
			systemState.controlState.controlTaskState = CONTROL_POWERLOSS;
			fanForceOff();
			setDutyCycle(0.5);
		}
	}else{
		if(systemState.controlState.controlTaskState == CONTROL_POWERLOSS_RELAYOFF){
			if ((HAL_GetTick() - closeRelayTime) > (60*60*1000)){
				systemState.controlState.controlTaskState = CONTROL_STOPANDRESET_RELAYOFF;
			}
		}
	}
	if(__HAL_TIM_GET_FLAG(&htim4,TIM_FLAG_CC4)){
		__HAL_TIM_CLEAR_FLAG(&htim4,TIM_FLAG_CC4);
		__HAL_TIM_SET_COUNTER(&htim4,0);
	}
}


void controlStopAndReset(void){
	if(systemState.controlState.controlTaskState != CONTROL_STOPANDRESET_RELAYOFF){
		systemState.controlState.controlTaskState = CONTROL_STOPANDRESET;
	}
}



static int filterState = 0;
void calculateFilters(void){
//	filterIOut = systemState.inputs.IOUT;
//	filterUCap = systemState.controlState.UCap;
//	filterIIn  = systemState.inputs.IIN;
//	IOUT_DC = IOUT_DC * DCRATIO + filterIOut * ((float)1.0-DCRATIO);
//	IIn_DC = IIn_DC * DCRATIO + filterIIn*((float)1.0-DCRATIO);
//	filterIOut = filterIOut - IOUT_DC;
//	TM_FILTER_FIR_F32_Process(currentFIR,&filterIOut,&systemState.controlState.IOutFiltered);
//	TM_FILTER_FIR_F32_Process(uCapFIR,&filterUCap,&systemState.controlState.UCapFiltered);
//	filterState = 1; //Do extra calculations in next iteration.
	filterState++;
	//Do some calculations in slower sampling loop but in next iteration, so we keep within the time limit.
	switch(filterState){
		case 0:
			break;
		case 1:
			calculateTemperature();
			break;
		case 2:
//			pidCompute(&capacitorPid);
			break;
		case 3:
			break;
		default:
			if(filterState >9){
				filterState = 0;
			}
		break;
	}
}

void calculateTemperature(void){
auxInputs.TEMPERATURE = 16.0f; //adcToCelsius();
	if(auxInputs.TEMPERATURE> 65.0f){

		if(auxInputs.TEMPERATURE>70.0f){
			setDutyCycle(0.0f);
			systemState.controlState.controlTaskState = CONTROL_OVERTEMPERATURE;
		}
	}
}

void calculateState(void){
		float now = HAL_GetTick();
		systemState.controlState.U12 = setPoint.voltage*1.41f*arm_sin_f32(50.0f*6.28f/1000.0f*now);
		systemState.controlState.dutyCycle = systemState.controlState.U12/systemState.inputs.VDCBUS;
}

void setDutyCycle(float dutyCycle){
	//We need about dutyCyle
	bool neg = dutyCycle<0;
	int dutyCounts = abs((int)((MAX_PWM_COUNTS)*dutyCycle));

	if (dutyCounts >= MAX_PWM_COUNTS){
		dutyCounts = MAX_PWM_COUNTS-1;
	}

//	}else if (dutyCounts < 8){
//		dutyCounts = 8; //limit the minimum duty cycle so the measurements can be done without interference.
//	}
	//TODO: Check the theory of sampling.
	//The ADC should sample during the switch being high after the switching has settled. Sampling all channels costs 15*3=45 cycles, at 84 MHz ABP2, a cycle costs 0.0238us, this is 1.01 us, this is 1.01/10*840=84 ticks,
	//When the duty cycle is larger than that, this is not a problem. Otherwise sample at end of the low state but leave time for next calculation (more noise sadly).
/*	int adcSamplePoint = 0;
	if (dutyCounts > MAX_PWM_COUNTS*3/4){
		adcSamplePoint = 839;
		//		adcSamplePoint = dutyCounts - 168;
	}else if (dutyCounts > MAX_PWM_COUNTS/4){
		adcSamplePoint = 839;
	}else{
		adcSamplePoint = 839;
	}
*/
//  	HAL_GPIO_WritePin(TEST_PIN_GPIO_Port,TEST_PIN_Pin,GPIO_PIN_SET);
	htim1.Instance->CCR1 = MAX_PWM_COUNTS-1; //adcSamplePoint;

	if(neg){
		htim1.Instance->CCR2 = 0;
		htim1.Instance->CCR3 = dutyCounts;
	}else{
		htim1.Instance->CCR2 = dutyCounts;
		htim1.Instance->CCR3 = 0;
	}
	//  	HAL_GPIO_WritePin(TEST_PIN_GPIO_Port,TEST_PIN_Pin,GPIO_PIN_RESET);
}

float getDutyCycle(void){
	return systemState.controlState.dutyCycle;
}

void calculateInputs(void){
	int i = 0;
	int temp[9];
	for (i=0;i<9;i++){
		temp[i] = 0;
	}
	for(i=0;i<FILTERDEPTH;i=i+SKIPFILTER){
		temp[0] = temp[0] + adc1Results[i].TEMPERATURE;
		temp[1] = temp[1] + adc1Results[i].RELAY_CHECK0;
		temp[2] = temp[2] + adc1Results[i].RELAY_CHECK1;
		temp[3] = temp[3] + adc1Results[i].ISOLATION;
		temp[4] = temp[4] + adc1Results[i].LCD_MIC;
		temp[5] = temp[5] + adc2Results[i].VDCBUS;
		temp[6] = temp[6] + adc2Results[i].VDCBUS2;
		temp[7] = temp[7] + adc3Results[i].CURRENTSENS;
		temp[8] = temp[8] + adc3Results[i].TRAFO;
	}

	systemState.inputs.CURSENS = ADCTO_CURRENT * ((temp[7])/(FILTERDEPTH/SKIPFILTER)-ADCOFFSET);
	systemState.inputs.TRAFO = ADCTO_TRAFO_VPEAK * ((temp[8])/(FILTERDEPTH/SKIPFILTER)-ADCOFFSET);
	systemState.inputs.VDCBUS = ADCTO_VDCBUS * ((temp[5]+temp[6])/(2*FILTERDEPTH/SKIPFILTER));

//	auxInputs.TEMPERATURE = ADCTOI_IN * (temp[0]/(FILTERDEPTH/SKIPFILTER)-ADCTOI_OFFSET_IN);
}

void controlSetFBR(float FB){
       if(FB>-0.0001f && FB < 1.0f){
               setPoint.fbR = FB;
       }
}

void controlInit(void){
	setPoint.voltage = 50;
	setPoint.freq = 50.0;
//	currentFIR = TM_FILTER_FIR_F32_Init(currentFilterCoefSize,currentFilterCoef,NULL,1);
//	currentFIR = TM_FILTER_FIR_F32_Init(uCapFilterCoeffSize,uCapFilterCoeffFloat,NULL,1);
//	uCapFIR = TM_FILTER_FIR_F32_Init(uCapFilterCoeffSize,uCapFilterCoeffFloat,NULL,1);
//	pidInit(&capacitorPid,&systemState.controlState.UCapFiltered,&systemState.controlState.UCapErr,&setPoint.UCapSet,setPoint.UCapP,setPoint.UCapI,setPoint.UCapD,P_ON_E,DIRECT);
//	pidSetSampleTime(&capacitorPid,0.001);
//	pidSetOutputLimits(&capacitorPid,-30,20);
	HAL_TIM_IC_Start(&htim4,TIM_CHANNEL_4);
	__HAL_TIM_CLEAR_FLAG(&htim4,TIM_FLAG_UPDATE);

	//	  HAL_GPIO_WritePin(BRIDGE_RELAY_GPIO_Port,BRIDGE_RELAY_Pin,GPIO_PIN_SET);
}

void controlSetFB(float FB){
	if(FB>-0.0001f && FB < 1.0f){
		setPoint.fbR = FB;
	}
}

void controlSetSetPoints(void* buffer){
	memcpy(&setPointCopy,buffer,sizeof(solarSetPoints_t));
	updateSetPoint = true;
}

void setupControlLoop(void){
	controlInit();
	controlMonitorInit();
	setDutyCycle(0.0);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*) adc1Results,ADC1_CHANNELS*FILTERDEPTH);
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*) adc2Results,ADC2_CHANNELS*FILTERDEPTH);
	HAL_ADC_Start_DMA(&hadc3,(uint32_t*) adc3Results,ADC3_CHANNELS*FILTERDEPTH);
	hdma_adc1.Instance->CR &= ~((uint32_t) DMA_SxCR_HTIE);
	hdma_adc2.Instance->CR &= ~((uint32_t) DMA_SxCR_TCIE);
	hdma_adc3.Instance->CR &= ~((uint32_t) DMA_SxCR_TCIE);
	hdma_adc1.Instance->CR &= ~((uint32_t) DMA_SxCR_HTIE);
	hdma_adc2.Instance->CR &= ~((uint32_t) DMA_SxCR_HTIE);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	htim1.Instance->CCMR1 |= (uint32_t) TIM_CCMR1_OC1PE;
	htim1.Instance->CCMR1 |= (uint32_t) TIM_CCMR1_OC1M;
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
	hadc1.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
	hadc2.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
	hadc3.Instance->CR2 |= (uint32_t)ADC_CR2_SWSTART;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if (hadc == &hadc3){
//	  	HAL_GPIO_WritePin(TEST_PIN_GPIO_Port,TEST_PIN_Pin,GPIO_PIN_SET);
//	  	HAL_GPIO_WritePin(HEARTBEAT_LED_GPIO_Port,HEARTBEAT_LED_Pin,GPIO_PIN_SET);
	  	controlLoop();
		controlStateCopy();
//	  	HAL_GPIO_WritePin(HEARTBEAT_LED_GPIO_Port,HEARTBEAT_LED_Pin,GPIO_PIN_RESET);
//	  	HAL_GPIO_WritePin(TEST_PIN_GPIO_Port,TEST_PIN_Pin,GPIO_PIN_RESET);
	}
}

solarFullControlState_t* getControlState(void){
	return &systemState;
}

void controlGetOutOfSafety(void){

	switch(systemState.controlState.controlTaskState){
	case CONTROL_SAFETY:
	case CONTROL_SAFETY_RELAYOFF:
	case CONTROL_OVERTEMPERATURE:
	case CONTROL_OVERTEMPERATURE_RELAYOFF:
	case CONTROL_POWERLOSS:
	case CONTROL_POWERLOSS_RELAYOFF:
		systemState.controlState.controlTaskState = CONTROL_IDLE;
	break;
	default:
	break;
	}
}

void controlGotoFixedDutyCycleState(float duty){
	systemState.controlState.controlTaskState = CONTROL_FIXED_DUTY_CYLE;
	systemState.controlState.dutyCycle = duty;
	setDutyCycle(getDutyCycle());
}

void controlStopRunning(void){
	solarSetPoints_t aSetPoint;
	memcpy(&aSetPoint,getSetPoint(),sizeof(solarSetPoints_t));
	controlSetSetPoints(&aSetPoint);
	systemState.controlState.controlTaskState = CONTROL_STOPPING;
	setDutyCycle(0.0);
}

void controlGetOutOfOvervoltage(void){

}

solarSetPoints_t* getSetPoint(void){
	return &setPoint;
}

solarAuxInputs_t* getAuxInputs(void){
	return &auxInputs;
}
