/*
 * PID_t.c
 *
 *  Created on: 4 sep. 2017
 *      Author: wvdv2
 */


/**********************************************************************************************
 * Arduino PID_t Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include <PID.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void pidInit(PID_t* aPid,float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd, int POn, int ControllerDirection)
{
    aPid->myOutput = Output;
    aPid->myInput = Input;
    aPid->mySetpoint = Setpoint;
    aPid->inAuto = false;

    pidSetOutputLimits(aPid,0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    aPid->sampleTime = 0.1;							//default Controller Sample Time is 0.1 seconds

    pidSetControllerDirection(aPid, ControllerDirection);
    pidSetTunings(aPid,Kp, Ki, Kd, POn);
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool pidCompute(PID_t* aPid)
{
   if(!aPid->inAuto) return false;
      /*Compute all the working error variables*/
      float input = *aPid->myInput;
      float error = *aPid->mySetpoint - input;
      float dInput = (input - aPid->lastInput);
      aPid->outputSum+= (aPid->ki * error);

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!aPid->pOnE) aPid->outputSum-= aPid->kp * dInput;

      if(aPid->outputSum > aPid->outMax) aPid->outputSum= aPid->outMax;
      else if(aPid->outputSum < aPid->outMin) aPid->outputSum= aPid->outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
	  float output;
      if(aPid->pOnE) output = aPid->kp * error;
      else output = 0;

      /*Compute Rest of PID_t Output*/
      output += aPid->outputSum - aPid->kd * dInput;

	    if(output > aPid->outMax) output = aPid->outMax;
      else if(output < aPid->outMin) output = aPid->outMin;
	    *aPid->myOutput = output;

      /*Remember some variables for next time*/
	    aPid->lastInput = input;
	    return true;
}



/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void pidSetTunings(PID_t* aPid, float Kp, float Ki, float Kd, int POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   aPid->pOn = POn;
   aPid->pOnE = POn == P_ON_E;

   aPid->dispKp = Kp; aPid->dispKi = Ki; aPid->dispKd = Kd;

   aPid->kp = Kp;
   aPid->ki = Ki * aPid->sampleTime;
   aPid->kd = Kd / aPid->sampleTime;

  if(aPid->controllerDirection ==REVERSE)
   {
	  aPid->kp = (0 - aPid->kp);
	  aPid->ki = (0 - aPid->ki);
	  aPid->kd = (0 - aPid->kd);
   }
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void pidSetSampleTime(PID_t* aPid, float NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = NewSampleTime
                      / aPid->sampleTime;
      aPid->ki *= ratio;
      aPid->kd /= ratio;
      aPid->sampleTime = NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void pidSetOutputLimits(PID_t* aPid, float Min, float Max)
{
   if(Min >= Max) return;
   aPid->outMin = Min;
   aPid->outMax = Max;

   if(aPid->inAuto)
   {
	   if(*aPid->myOutput > aPid->outMax) *aPid->myOutput = aPid->outMax;
	   else if(*aPid->myOutput < aPid->outMin) *aPid->myOutput = aPid->outMin;

	   if(aPid->outputSum > aPid->outMax) aPid->outputSum= aPid->outMax;
	   else if(aPid->outputSum < aPid->outMin) aPid->outputSum= aPid->outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void pidSetMode(PID_t* aPid,int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !aPid->inAuto)
    {  /*we just went from manual to auto*/
        pidInitialize(aPid);
    }
    aPid->inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void pidInitialize(PID_t* aPid)
{
	aPid->outputSum = *aPid->myOutput;
	aPid->lastInput = *aPid->myInput;
   if(aPid->outputSum > aPid->outMax) aPid->outputSum = aPid->outMax;
   else if(aPid->outputSum < aPid->outMin) aPid->outputSum = aPid->outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID_t will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void pidSetControllerDirection(PID_t* aPid, int Direction)
{
   if(aPid->inAuto && Direction !=aPid->controllerDirection)
   {
	   aPid->kp = (0 - aPid->kp);
	   aPid->ki = (0 - aPid->ki);
	   aPid->kd = (0 - aPid->kd);
   }
   aPid->controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID_t.  they're here for display
 * purposes.  this are the functions the PID_t Front-end uses for example
 ******************************************************************************/
float pidGetKp(PID_t* aPid){ return  aPid->dispKp; }
float pidGetKi(PID_t* aPid){ return  aPid->dispKi;}
float pidGetKd(PID_t* aPid){ return  aPid->dispKd;}
int pidGetMode(PID_t* aPid){ return  aPid->inAuto ? AUTOMATIC : MANUAL;}
int pidGetDirection(PID_t* aPid){ return aPid->controllerDirection;}

