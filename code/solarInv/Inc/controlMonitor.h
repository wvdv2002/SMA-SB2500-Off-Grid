/*
 * controlCopy.h
 *
 *  Created on: 23 aug. 2017
 *      Author: wvdv2
 */

#ifndef CONTROLMONITOR_H_
#define CONTROLMONITOR_H_
#include "controlLoop.h"

#define SYSTEMSTATEBUFFERSIZE 512
#define MAXLOGFILESIZE (1024*1024*25)

void controlMonitorInit(void);
void controlMonitorTask(void);
void controlStateCopy(void);
void controlSetStatePointer(void);
void controlLoopToString(const char*);
void controlStartCopy(int divider);
void controlStopCopy(void);
bool controlMonitorIsNewEvent(void);
void controlStopSamplingOnThisEvent(int);
solarFullControlState_t* getCopyControlState(void);
solarFullControlState_t* getIndexControlState(int index);
int controlStartGetLastEvents(int);
solarFullControlState_t* controlReadNextState(void);
void controlStopGetLastEvents(void);
void _controlLoopToString(solarFullControlState_t* controlImage,solarSetPoints_t* setPoint, const char* buffer);
void controlCalculateIInError(void);
#endif /* CONTROLMONITOR_H_ */
