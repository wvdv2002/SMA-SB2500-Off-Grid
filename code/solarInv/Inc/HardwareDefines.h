/*
 * HardwareDefines.h
 *
 *  Created on: 11 aug. 2017
 *      Author: wvdv2
 */

#ifndef HARDWAREDEFINES_H_
#define HARDWAREDEFINES_H_
#include "stm32f4xx_hal.h"
#include "main.h" //Defines of GPIO pins

#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 1

#define CONTROLUSART USART1
#define PWMFREQ 16000
#define MAX_PWM_COUNTS (168000000/PWMFREQ/2)

/*=650 volt is 3.34 volt on ADC input (650/(4.91*22/32/3.3*4096)) 3.3v vref.*/
#define ADCTO_VDCBUS ((float) (650/(4.91*22/32/3.3*4096)))

/*15/(2.5*10/33/3.3*4096), 15A is 2.5v, With LM358 amplifier it comes down to:*/
#define ADCTO_CURRENT ((float)(15/(2.5*10/33/3.3*4096)))

/*=230*SQRT(2)*/
#define ADCTO_TRAFO_VPEAK ((float)(230*1.41421)/(4.88*10/39/3.3*4096))

/*=Negative offset.*/
#define ADCOFFSET 2048

#define ADCTOI_IN (float)0.020345//ACS751LCB-050U has 39.6mV/A, 1/(0.0396/3.3*4096)=0.05371
#define ADCTOI_OUT (float)0.020345
#define ADCTOI_OFFSET_IN 409 //0.33/3.3*4096 = 409
#define ADCTOI_OFFSET_OUT 409

#define FILTERDEPTH 2
#define SKIPFILTER 1

#define ADC_CHANNELS 2
#define ADC1_CHANNELS 5
#define ADC2_CHANNELS ADC_CHANNELS
#define ADC3_CHANNELS ADC_CHANNELS

#define LOOPFREQ (PWMFREQ/FILTERDEPTH/ADC_CHANNELS)
#define DELTATMS (1000.0/LOOPFREQ)
#endif /* HARDWAREDEFINES_H_ */


typedef struct {
	uint16_t TEMPERATURE;
	uint16_t RELAY_CHECK0;
	uint16_t RELAY_CHECK1;
	uint16_t ISOLATION;
	uint16_t LCD_MIC;
}adc1Results_t;


typedef struct {
	uint16_t VDCBUS;
	uint16_t VDCBUS2;
}adc2Results_t;

typedef struct {
	uint16_t CURRENTSENS;
	uint16_t TRAFO;
}adc3Results_t;


extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream4;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;

