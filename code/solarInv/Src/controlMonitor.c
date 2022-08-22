/*
 * controlMonitor.c
 *
 *  Created on: 23 aug. 2017
 *      Author: wvdv2
 */
#include "controlMonitor.h"
#include "commands.h"
#include "HardwareDefines.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "math.h"
#include "fanTask.h"
solarFullControlState_t systemStateBuffer[SYSTEMSTATEBUFFERSIZE];
int systemStateBufferPointer = 0;
solarFullControlState_t* theSystemState;
volatile int doStateCopy;
unsigned int sampleDivider;
unsigned int sampleCounter;
int lastControlState;
unsigned int samplesTillEnd;
int stopSamplingOnThisEvent=-1;
unsigned int aNewEvent;
solarFullControlState_t aReadSystemState;
uint32_t lastIInErrUpdate = 0;
bool updateGenerator = false;

void controlCheckForSamplingStop(void);
void controlCloseLogFile(void);
void controlOpenLogFile(void);


void controlStateCopy(void){

	HAL_DMA_Abort(&hdma_memtomem_dma2_stream4);
	HAL_DMA_Start(&hdma_memtomem_dma2_stream4,(uint32_t*)theSystemState,&systemStateBuffer[systemStateBufferPointer],sizeof(solarFullControlState_t)/4);
	if(doStateCopy>0){
		controlCheckForSamplingStop();
		if(sampleCounter++>=sampleDivider){
			systemStateBufferPointer++;
			sampleCounter = 0;
			if(samplesTillEnd>0){
				if(samplesTillEnd==1){
					doStateCopy = 0;
					aNewEvent = 1;
				}
				samplesTillEnd--;
			}
		}
	}
	if(systemStateBufferPointer>=SYSTEMSTATEBUFFERSIZE){
		systemStateBufferPointer = 0;
	}
}


void controlStopSamplingOnThisEvent(int event){
	stopSamplingOnThisEvent = event;
	if(event==0){stopSamplingOnThisEvent = -1;};
	if(doStateCopy==0){
		controlStartCopy(controlGetDivider());
	}
}

void controlCheckForSamplingStop(void){
	if(theSystemState->controlState.controlTaskState != lastControlState){
		lastControlState = theSystemState->controlState.controlTaskState;
		if(theSystemState->controlState.controlTaskState==stopSamplingOnThisEvent)
		{
			stopSamplingOnThisEvent = -1;
			samplesTillEnd = SYSTEMSTATEBUFFERSIZE/2;
		}else if((theSystemState->controlState.controlTaskState>=CONTROL_SAFETY) && (samplesTillEnd==0)){ //samplestillend zero check is necessary, otherwise 2 consecutive errors will keep the sampling going.
			samplesTillEnd = SYSTEMSTATEBUFFERSIZE/8;
		}
	}
}

bool controlMonitorIsNewEvent(void){
	bool temp = (aNewEvent>0) && (doStateCopy==0);
	if(temp){
		aNewEvent=0;
	}
	return temp;
}
void setSampleDivider(int divider){
	sampleDivider = divider;
}


//void controlFindNextFreeFile(void)
//{
//    FRESULT fr;     /* Return value */
//    DIR dj;         /* Directory search object */
//    FILINFO fno;    /* File information */
//
//    fr = f_findfirst(&dj, &fno, "", "*.DAT");  /* Start to search for photo files */
//
//    while (fr == FR_OK && fno.fname[0]) {         /* Repeat while an item is found */
//        printf("%s\n", fno.fname);                /* Display the object name */
//        fr = f_findnext(&dj, &fno);               /* Search for next item */
//    }
//
//    f_closedir(&dj);
//}

void controlMonitorInit(void){
	doStateCopy = 1;
	samplesTillEnd = 0;
	aNewEvent = 0;
	sampleDivider = 10;
	theSystemState = getControlState();
	lastIInErrUpdate = HAL_GetTick();
	lastControlState = theSystemState->controlState.controlTaskState;
	memset(systemStateBuffer,0,sizeof(systemStateBuffer));
}

solarFullControlState_t* getCopyControlState(void){
	int lastPointer;
	if(doStateCopy){
		lastPointer = systemStateBufferPointer-1;
	}else{
		lastPointer = systemStateBufferPointer;
	}
	if (lastPointer < 0){lastPointer = SYSTEMSTATEBUFFERSIZE-1;}
	return &systemStateBuffer[lastPointer];
}

solarFullControlState_t* getIndexControlState(int index){
	index += systemStateBufferPointer;
	if(index>=SYSTEMSTATEBUFFERSIZE){
		index -= SYSTEMSTATEBUFFERSIZE;
	}
	if(index>SYSTEMSTATEBUFFERSIZE){
		index = 0;
	}
	return &systemStateBuffer[index];
}


int controlGetDivider(void){
	return sampleDivider;
}

void controlStartCopy(int divider){
	doStateCopy = 1;
	setSampleDivider(divider);
}

void controlStopCopy(void){
	doStateCopy = 0;
}


void controlMonitorTask(void){

}

void controlCalculateIInError(void){
	float avgIIn=0;
	float iInError = 0;
	float* aBuffer = getIInSampleBuffer();
	for(int i = 0;i<IINSAMPLEDEPTH;i++){
		avgIIn += aBuffer[i];
	}
	avgIIn = avgIIn/IINSAMPLEDEPTH;
	for(int i = 0;i<IINSAMPLEDEPTH;i++){
		iInError += powf(aBuffer[i]-avgIIn,2);
	}
	iInError = sqrtf(iInError/IINSAMPLEDEPTH);
	getAuxInputs()->IErr = iInError;
}

