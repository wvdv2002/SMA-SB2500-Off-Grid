/*
	Copyright 2012-2015 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * commands.c
 *
 *  Created on: 19 sep 2014
 *      Author: benjamin
 */

#include "stm32f4xx_hal.h"
#include "HardwareDefines.h"
#include "commands.h"
#include "buffer.h"
#include "packet.h"
#include "controlMonitor.h"
#include "controlLoop.h"
#include "flash_helper.h"
#include "libRelay.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// Threads


// Private variables
static uint8_t send_buffer[PACKET_MAX_PL_LEN];
static void(*send_func)(unsigned char *data, unsigned int len) = 0;

//private functions

/**
 * Provide a function to use the next time there are packets to be sent.
 *
 * @param func
 * A pointer to the packet sending function.
 */
void commands_set_send_func(void(*func)(unsigned char *data, unsigned int len)) {
	send_func = func;
}

/**
 * Send a packet using the set send function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void commands_send_packet(unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(data, len);
	}
}

/**
 * Process a received buffer with commands and data.
 *
 * @param data
 * The buffer to process.
 *
 * @param len
 * The length of the buffer.
 */

void controlStateToBuffer(uint8_t* data,int32_t* ind,solarFullControlState_t* control){
	data[*ind] = control->controlState.controlTaskState;
	*ind+=1;
	buffer_append_float32(data,control->controlState.U12,100,ind);
}

void commands_process_packet(unsigned char *data, unsigned int len) {
	if (!len) {
		return;
	}
	int32_t ind = 0;
	solarCommPacketId_t packet_id;
	solarSetPoints_t aSetPoint;
	uint16_t flash_res;
	uint32_t new_app_offset;

	packet_id = data[0];
	data++;
	len--;

	switch (packet_id) {
	case COMM_FW_VERSION:
		ind = 0;
		send_buffer[ind++] = COMM_FW_VERSION;
		send_buffer[ind++] = FW_VERSION_MAJOR;
		send_buffer[ind++] = FW_VERSION_MINOR;
		commands_send_packet(send_buffer, ind);
		break;

	case COMM_GET_CONTROLSTATE:
		ind = 0;
		send_buffer[ind++] = COMM_GET_CONTROLSTATE;
		controlStateToBuffer(send_buffer,&ind,getCopyControlState());
		commands_send_packet(send_buffer, ind);
//		commands_printf("syncTime: %i",getSyncTime());
		break;

	case COMM_GET_ALLCONTROLSIZE:
		ind = 0;
		send_buffer[ind++] = COMM_GET_ALLCONTROLSIZE;
		buffer_append_uint16(send_buffer,(uint16_t)SYSTEMSTATEBUFFERSIZE,&ind);
		commands_send_packet(send_buffer, ind);
		break;

	case COMM_SENDGET_ALLCONTROLSTATE:
	case COMM_GET_ALLCONTROLSTATE:
		flash_res = buffer_get_uint16(data,&ind);
		ind = 0;
		if(flash_res ==0){
			controlStopCopy();
		}
		send_buffer[ind++] = COMM_GET_ALLCONTROLSTATE;
		buffer_append_uint16(send_buffer,(uint16_t)flash_res,&ind);
		controlStateToBuffer(send_buffer,&ind,getIndexControlState(flash_res));
		commands_send_packet(send_buffer, ind);
		break;

	case COMM_GET_LAST_EVENTS:
		break;

	case COMM_ERASE_LOG:
//		controlEraseLog();
		break;

	case COMM_APPEND_TO_LOG:
//		controlAppendToLog();
		break;

	case COMM_SET_DUTY_CYCLE:
		controlGotoFixedDutyCycleState(buffer_get_float32(data,100,&ind));
		break;

	case COMM_START_SAMPLING:
		controlStartCopy(buffer_get_int32(data,&ind));
		break;

	case COMM_STOP_SAMPLING:
		controlStopCopy();
		break;

	case COMM_GET_AUXVALUES:
		ind = 0;
		solarAuxInputs_t* aux = getAuxInputs();
		send_buffer[ind++] = COMM_GET_AUXVALUES;
		buffer_append_float32(send_buffer,aux->TEMPERATURE,10,&ind);
		buffer_append_float32(send_buffer,aux->VAC_DETECTION,10,&ind);
		commands_send_packet(send_buffer, ind);
		break;

	case COMM_SET_SETPOINTS:
		memset(&aSetPoint,0,sizeof(solarSetPoints_t));
		aSetPoint.voltage = buffer_get_float32(data,10,&ind);
		aSetPoint.freq = buffer_get_float32(data,100,&ind);
		aSetPoint.deadTime = buffer_get_float32(data,1,&ind);
		aSetPoint.vbusMin = buffer_get_float32(data,1,&ind);
		aSetPoint.vbusMax = buffer_get_float32(data,1,&ind);
		aSetPoint.iMax = buffer_get_float32(data,10,&ind);
		controlSetSetPoints(&aSetPoint);
		break;


	case COMM_GET_SETPOINTS:
		ind = 0;
		send_buffer[ind++] = COMM_GET_SETPOINTS;
		memcpy(&aSetPoint,getSetPoint(),sizeof(solarSetPoints_t));
		buffer_append_float32(send_buffer,aSetPoint.voltage,10,&ind);
		buffer_append_float32(send_buffer,aSetPoint.freq,100,&ind);
		buffer_append_float32(send_buffer,aSetPoint.deadTime,1,&ind);
		buffer_append_float32(send_buffer,aSetPoint.vbusMin,1,&ind);
		buffer_append_float32(send_buffer,aSetPoint.vbusMax,1,&ind);
		buffer_append_float32(send_buffer,aSetPoint.iMax,10,&ind);
		commands_send_packet(send_buffer,ind);
		break;

	case COMM_ZERO_ADC:
		break;
	case COMM_REBOOT:
		// Lock the system and enter an infinite loop. The watchdog will reboot.
		NVIC_SystemReset();
		break;

	case COMM_RESET_OVERVOLTAGE:
		controlGetOutOfOvervoltage();
		break;

	case COMM_RESET_SAFETY:
		controlGetOutOfSafety();
		break;

	case COMM_ALIVE:
//		timeout_reset();
		break;
	case COMM_ERASE_NEW_FW:
		ind = 0;
		flash_res = flash_helper_erase_new_app(buffer_get_uint32(data, &ind));

		ind = 0;
		send_buffer[ind++] = COMM_ERASE_NEW_FW;
		send_buffer[ind++] = flash_res == HAL_OK ? 1 : 0;
		commands_send_packet(send_buffer, ind);
		break;

	case COMM_WRITE_NEW_FW_DATA:
		ind = 0;
		new_app_offset = buffer_get_uint32(data, &ind);
		flash_res = flash_helper_write_new_app_data(new_app_offset, data + ind, len - ind);

		ind = 0;
		send_buffer[ind++] = COMM_WRITE_NEW_FW_DATA;
		send_buffer[ind++] = flash_res == HAL_OK ? 1 : 0;
		commands_send_packet(send_buffer, ind);
		break;

    case COMM_JUMP_TO_BOOTLOADER:
    	if(isBridgeRelayClosed()){
    		int *p = (int*) 0x0080E0000; //position of the bootloader
    		if(*p != 0xFFFFFFFF){ //If the bootloader is flashed.
    			flash_helper_jump_to_bootloader();
    		}
    	}
    	ind = 0;
    	send_buffer[ind++] = COMM_JUMP_TO_BOOTLOADER;
    	send_buffer[ind++] = 0;
    	commands_send_packet(send_buffer, ind);
    	break;

    case COMM_GET_STATUS:
		ind = 0;
		solarFullControlState_t* aCState = getCopyControlState();
		solarAuxInputs_t* aAState = getAuxInputs();
		send_buffer[ind++] = COMM_GET_STATUS;
		buffer_append_float32(send_buffer,aAState->TEMPERATURE,10,&ind);
		buffer_append_float32(send_buffer,aAState->IErr,1000,&ind);
		send_buffer[ind++] = (uint8_t) aCState->controlState.controlTaskState;
		send_buffer[ind++] = (uint8_t) 0;
		commands_send_packet(send_buffer,ind);
		break;

	case COMM_SET_CALIBRATION:
		break;

	case COMM_CLOSE_BRIDGERELAY:
		closeBridgeRelay();
		break;

	case COMM_OPEN_BRIDGERELAY:
		openBridgeRelay();
		break;

	case COMM_STOP_RUNNING:
		controlStopRunning();
		break;

	case COMM_STOPANDRESET:
		controlStopAndReset();
		break;
	case COMM_GET_ISNEWSAMPLEDEVENT:
		send_buffer[ind++] = COMM_GET_ISNEWSAMPLEDEVENT;
		send_buffer[ind++] = controlMonitorIsNewEvent();
		commands_send_packet(send_buffer, ind);
		break;
	case COMM_SET_STOPSAMPLINGONTHISEVENT:
		controlStopSamplingOnThisEvent(buffer_get_int32(data,&ind));
		break;
	default:
		break;
	}
}

#define PRINTMAXLEN 1024
void commands_printf(char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[PRINTMAXLEN];

	print_buffer[0] = COMM_PRINT;
	len = vsnprintf(print_buffer+1, PRINTMAXLEN-1, format, arg);
	va_end (arg);

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer, (len<PRINTMAXLEN-1)? len+1: PRINTMAXLEN-1);
	}
}

/*void commands_send_samples(uint8_t *data, int len) {
	uint8_t buffer[len + 1];
	int index = 0;

	buffer[index++] = COMM_ALIVE;

	for (int i = 0;i < len;i++) {
		buffer[index++] = data[i];
	}

	commands_send_packet(buffer, index);
}
*/
