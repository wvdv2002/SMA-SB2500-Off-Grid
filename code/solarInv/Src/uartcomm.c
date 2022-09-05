/*
 * uartcomm.c
 *
 *  Created on: 24 aug. 2017
 *      Author: wvdv2
 */

#include "uartcomm.h"
#include "stm32f4xx_hal.h"
#include "HardwareDefines.h"
#include "cmsis_os.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_usart.h"
#include "tm_stm32_usart_dma.h"
#include "packet.h"
#include "commands.h"
#include "controlLoop.h"
#include "controlMonitor.h"
#include "fanTask.h"
#include "leds/Leds.h"
#include "tm_stm32_hd44780.h"

#define UART_RX_BUFFER_SIZE 512
#define PACKET_HANDLER 0

static unsigned char UART_DMA_RX_BUFFER[UART_RX_BUFFER_SIZE];
osSemaphoreId controlUartNewDataFlag_id;     // message queue id
uint32_t rxBufferReadPointer = 0;
static TickType_t lastDataReceived;
extern

LED led_table[] = {
	{{LED_HEARTBEAT_GPIO_Port,LED_HEARTBEAT_Pin},led_mode_heartbeat,0},
	{{LED_GREEN_GPIO_Port,LED_GREEN_Pin},led_mode_blink_slow,0},
	{{LED_RED_GPIO_Port,LED_RED_Pin},led_mode_SOS,0},
	{{LED_YELLOW_GPIO_Port,LED_YELLOW_Pin},led_mode_blink_fast,0},
    LED_TABLE_END //! definition of when a led table ends
};


void resetWatchTask(void);
void ledTask(void);
void watchDogTask(void);
void lcdTask(void);

void startCommandThread(void const * argument){
	int32_t dataReceived;
	setupControlLoop();

	packet_init(send_packet, process_packet, PACKET_HANDLER);
	osSemaphoreDef(SEM);
	led_group_init(led_table);
	controlUartNewDataFlag_id = osSemaphoreCreate(osSemaphore(SEM),2);
	osSemaphoreWait(controlUartNewDataFlag_id,0);
	osSemaphoreWait(controlUartNewDataFlag_id,0);
	int rxBufferWritepointer;
//	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_ERR);

	lastDataReceived = HAL_GetTick();
	HAL_UART_Receive_DMA(&huart1,UART_DMA_RX_BUFFER,UART_RX_BUFFER_SIZE);

	TM_HD44780_Init(16, 2);
	TM_HD44780_Puts(0, 0, "Hello World");
	TM_HD44780_BlinkOff();
	HAL_GPIO_WritePin(RS485_RXENA_GPIO_Port, RS485_RXENA_Pin, 0);
	HAL_GPIO_WritePin(RS485_TXENA_GPIO_Port, RS485_TXENA_Pin, 1);
	for (;;){
	 	dataReceived = osSemaphoreWait(controlUartNewDataFlag_id,0);
	 	if(dataReceived){
	 		lastDataReceived = HAL_GetTick();
			rxBufferWritepointer = UART_RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA2,LL_DMA_STREAM_5);
			while(rxBufferReadPointer!=rxBufferWritepointer){
				packet_process_byte(UART_DMA_RX_BUFFER[rxBufferReadPointer++],PACKET_HANDLER);
				if(rxBufferReadPointer>=UART_RX_BUFFER_SIZE){
					rxBufferReadPointer = 0;
				}
			}
	 	}else{
	 		if(HAL_GetTick()-lastDataReceived > 120*1000){
	 			lastDataReceived = HAL_GetTick();
	 			__HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
	 			__HAL_UART_DISABLE_IT(&huart1,UART_IT_ERR);
	 			HAL_UART_AbortReceive(&huart1);
	 			__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	 			__HAL_UART_ENABLE_IT(&huart1,UART_IT_ERR);
	 			rxBufferReadPointer = 0;
	 			HAL_UART_Receive_DMA(&huart1,UART_DMA_RX_BUFFER,UART_RX_BUFFER_SIZE);
	 		}
	 	}
	 	osDelay(1);
	 	packet_timerfunc();
	 	controlMonitorTask();
//	 	fanTask();
//	 	resetWatchTask();
	 	ledTask();
	 	watchDogTask();
	 	lcdTask();
	}
}

#define SOLARREBOOTTIME (3600*1000*9)

int isResetSend = 0;
void resetWatchTask(void){
	if(HAL_GetTick()>SOLARREBOOTTIME){
		if(isResetSend!=1){
			controlStopRunning();
			isResetSend = 1;
		}
	}
	if(HAL_GetTick()>(SOLARREBOOTTIME+5000)){
		if(isResetSend==1){
			controlStopAndReset();
			isResetSend = 1;
		}
	}
}

static TickType_t temperatureErrorTime = 0;
static TickType_t watchdogCheckTime = 0;

void watchDogTask(void){
	TickType_t now = HAL_GetTick();
	if((now-watchdogCheckTime)>250){
		watchdogCheckTime = now;

		//Check if the overtemperature error can be reset.
		if((getControlState()->controlState.controlTaskState == CONTROL_OVERTEMPERATURE_RELAYOFF) && (getAuxInputs()->TEMPERATURE < 50.0f)){
			if((now-temperatureErrorTime) > 60*60*1000){
			controlStopAndReset();
			}
		}else{
			temperatureErrorTime = now;
		}
	}
}

void lcdTask(void){
	static TickType_t lastLCDUpdate = 0;
	TickType_t now = HAL_GetTick();
	if ((now-lastLCDUpdate)>1000){
		lastLCDUpdate = now;
		char info[25];
		sprintf(info,"%i B:%1.0f V:%1.0f   ",getCopyControlState()->controlState.controlTaskState,getCopyControlState()->inputs.VDCBUS,getSetPoint()->voltage);
		TM_HD44780_Puts(0, 0, info);
		sprintf(info,"D:%i   ",getSetPoint()->deadTime);
		TM_HD44780_Puts(0, 1, info);

	}
}

void ledTask(void){
	static TickType_t lastLedUpdate = 0;
	static bool lastGreenLedState = 0;
	static bool lastRedLedState = 0;
	//! definition of when a led table ends
	if ((HAL_GetTick()-lastLedUpdate)>250){
		if (lastGreenLedState != HAL_GPIO_ReadPin(LCD_SWA_GPIO_Port, LCD_SWA_Pin)){
			lastGreenLedState = !lastGreenLedState;
			if (lastGreenLedState){
				led_set_mode(&led_table[1], led_mode_blink_fast);
			}else{
				led_set_mode(&led_table[1], led_mode_blink_slow);
			}
		}

		if (lastRedLedState != HAL_GPIO_ReadPin(LCD_SWB_GPIO_Port, LCD_SWB_Pin)){
			lastRedLedState = !lastRedLedState;
			if(lastRedLedState){
				led_set_mode(&led_table[2], led_mode_SOS);
			}else {
				led_set_mode(&led_table[2], led_mode_on);
			}
		}
		lastLedUpdate=HAL_GetTick();
		led_group_update(led_table);
	}
}



void HAL_USART_ErrorCallback(USART_HandleTypeDef *husart){
//	HAL_GPIO_WritePin(HEARTBEAT_LED_GPIO_Port,HEARTBEAT_LED_Pin,GPIO_PIN_SET);
}

static void process_packet(unsigned char *data, unsigned int len) {
	commands_set_send_func(send_packet_wrapper);
	commands_process_packet(data, len);
}

static void send_packet_wrapper(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, PACKET_HANDLER);
}

static void send_packet(unsigned char *data, unsigned int len) {
	TickType_t timeout = HAL_GetTick();
	while(TM_USART_DMA_Transmitting(CONTROLUSART)){
		if(HAL_GetTick()-timeout>100){
			break;
		}
		osDelay(1);
	}
	TM_USART_DMA_Send(CONTROLUSART,data,len);
}

void controlUsartHandler(void) {
	uint8_t temp  __attribute__((unused));
	if (__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)) {
		temp = huart1.Instance->SR;
        temp = huart1.Instance->DR;
		__HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_IDLE);
        osSemaphoreRelease(controlUartNewDataFlag_id);
    }else{
		temp = huart1.Instance->SR;
        temp = huart1.Instance->DR;
    	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_FE);
    	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE);
    	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_NE);
    	__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
//    	HAL_GPIO_WritePin(HEARTBEAT_LED_GPIO_Port,HEARTBEAT_LED_Pin,GPIO_PIN_SET);
    	HAL_UART_DMAResume(&huart1);
    }
}
