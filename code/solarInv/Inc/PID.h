/*
 * PID.h
 *
 *  Created on: 4 sep. 2017
 *      Author: wvdv2
 */

#ifndef PID_H_
#define PID_H_

#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.1.1
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define P_ON_M 0
#define P_ON_E 1
#include <stdbool.h>

typedef struct{
	float dispKp;				// * we'll hold on to the tuning parameters in user-entered
	float dispKi;				//   format for display purposes
	float dispKd;				//
	float kp;                  // * (P)roportional Tuning Parameter
	float ki;                  // * (I)ntegral Tuning Parameter
	float kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

	float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
	float *myOutput;             //   This creates a hard link between the variables and the
	float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
									  //   what these values are.  with pointers we'll just know.

	unsigned long lastTime;
	float outputSum, lastInput;

	float sampleTime;
	float outMin, outMax;
	bool inAuto, pOnE;
}PID_t;


//commonly used functions **************************************************************************
void pidInit(PID_t*, float*, float*, float*,        // * constructor.  links the PID to the Input, Output, and
        float, float, float, int, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

void pidSetMode(PID_t*, int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

bool pidCompute(PID_t*);                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

void pidSetOutputLimits(PID_t*, float, float); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application



  //available but not commonly used functions ********************************************************
void pidSetTunings(PID_t*, float, float,       // * overload for specifying proportional mode
                    float, int);

void pidSetControllerDirection(PID_t*, int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
void pidSetSampleTime(PID_t*, float);              // * sets the frequency, in Milliseconds, with which
                                          //   the PID calculation is performed.  default is 100



  //Display functions ****************************************************************
float pidGetKp(PID_t*);						  // These functions query the pid for interal values.
float pidGetKi(PID_t*);						  //  they were created mainly for the pid front-end,
float pidGetKd(PID_t*);						  // where it's important to know what is actually
int pidGetMode(PID_t*);						  //  inside the PID.
int pidGetDirection(PID_t*);					  //

void pidInitialize(PID_t*);


#endif
#endif /* PID_H_ */
