/*
 * datatypes.h
 *
 *  Created on: 23 aug. 2017
 *      Author: wvdv2
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

typedef struct {
	float CURSENS;
	float TRAFO;
	float VDCBUS;
}solarSysInputs_t;

typedef struct {
	float CURSENS;
	float TRAFO;
	float VDCBUS;
}solarExternalSysInputs_t;


typedef struct {
	float TEMPERATURE;
	float VAC_DETECTION;
	float ISOLATION_SENSE;
	float LCD_MIC;
	float Vpp;
	float IErr;
	uint8_t relayState;
	uint8_t realCapVoltageUsed;
}solarAuxInputs_t;

typedef enum{
	CONTROL_IDLE,
	CONTROL_WAITFOR_SAFESTART,
	CONTROL_WAITFOR_GRIDRELAYCHECK,
	CONTROL_WAITFOR_GRIDCONTROLCHECK,
	CONTROL_RUNNING,
	CONTROL_STOPPING,
	CONTROL_STOPPING_RELAYOFF,
	CONTROL_FIXED_DUTY_CYLE,
	CONTROL_SAFETY,
	CONTROL_SAFETY_RELAYOFF,
	CONTROL_OVERVOLTAGE,
	CONTROL_OVERVOLTAGE_RELAYOFF,
	CONTROL_OVERTEMPERATURE,
	CONTROL_OVERTEMPERATURE_RELAYOFF,
	CONTROL_POWERLOSS,
	CONTROL_POWERLOSS_RELAYOFF,
	CONTROL_STOPANDRESET,
	CONTROL_STOPANDRESET_RELAYOFF,
	CONTROL_BRIDGEMALFUNCTION,
	CONTROL_BRIDGEMALFUNCTION_RELAYOFF,
}solarControlTaskState_t;

typedef enum{
	EVENT_OVERVOLTAGE,
	EVENT_SYNCRPM,
}solarEvents_t;


typedef struct {
	solarControlTaskState_t controlTaskState;
	float U12;
	float dutyCycle;
	float IErr;
	float IOutFiltered;
	float TrafoFiltered;
	float VDCBusFiltered;
}solarControlState_t;

typedef struct {
	solarControlTaskState_t controlTaskState;
	float U;
	float dutyCycle;
	float IErr;
	float IOutFiltered;
	float TrafoFiltered;
	float VDCBusFiltered;
}solarExternalControlState_t;

typedef struct {
	float power;
	float I;
	float freq;
	float voltage;
	float fbR;
	float vbusMin;
	float vbusMax;
	float iMax;
	uint8_t deadTime;
}solarSetPoints_t;

typedef struct {
	solarControlState_t controlState;
	solarSysInputs_t inputs;
}solarFullControlState_t;

typedef struct {
	float temp;
	float power;
	float voltage;
	solarControlTaskState_t controlState;
	solarEvents_t lastEvent;
}solarStatus_t;


// Communication commands
typedef enum {
	COMM_FW_VERSION = 0,
	COMM_GET_AUXVALUES,
	COMM_GET_CONTROLSTATE,
	COMM_GET_ALLCONTROLSTATE,
	COMM_GET_SETPOINTS,
	COMM_SET_POWER,
	COMM_SET_DUTY_CYCLE,
	COMM_SET_SETPOINTS,
	COMM_START_SAMPLING,
	COMM_STOP_SAMPLING,
	COMM_PRINT,
	COMM_ZERO_ADC,
	COMM_REBOOT,
	COMM_ALIVE,
	COMM_GET_LAST_EVENTS,
	COMM_ERASE_LOG,
	COMM_APPEND_TO_LOG,
	COMM_RESET_OVERVOLTAGE,
	COMM_RESET_SAFETY,
	COMM_JUMP_TO_BOOTLOADER = 22,
	COMM_ERASE_NEW_FW,
	COMM_WRITE_NEW_FW_DATA,
	COMM_GET_STATUS,
	COMM_INVALID_PACKAGE,
	COMM_GET_ALLCONTROLSIZE,
	COMM_OPEN_BRIDGERELAY,
	COMM_CLOSE_BRIDGERELAY,
	COMM_STOP_RUNNING,
	COMM_SET_CALIBRATION,
	COMM_GET_CALIBRATION,
	COMM_SENDGET_ALLCONTROLSTATE,
	COMM_STOPANDRESET,
	COMM_GET_ISNEWSAMPLEDEVENT,
	COMM_SET_STOPSAMPLINGONTHISEVENT,
} solarCommPacketId_t;




#endif /* DATATYPES_H_ */
