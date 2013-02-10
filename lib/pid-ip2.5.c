/*
 * Name: UpdatePID.c
 * Desc: Control code to compute the new input to the plant
 * Date: 2009-04-03
 * Author: AMH
   modified to include hall effect sensor by RSF.
 * modified Dec. 2011 to include telemetry capture
 * modified Jan. 2012 to include median filter on back emf
 * modified Jan. 2013 to include AMS Hall encoder, and MPU 6000 gyro
 */
#include "pid-ip2.5.h"
#include "dfmem.h"
#include "gyro.h"
#include "steering.h"
#include "motor_ctrl.h"
#include "timer.h"
#include "adc_pid.h"
#include "pwm.h"
#include "led.h"
#include "adc.h"
#include "move_queue.h"
//#include "hall.h"
#include "p33Fxxxx.h"
#include "stopwatch.h"
//#include "incap.h" // input capture
#include "ams-enc.h"
#include "tih.h"
#include <stdlib.h> // for malloc
#include "init.h"  // for Timer1

//#define HALFTHROT 10000
#define HALFTHROT 2000
#define FULLTHROT 2*HALFTHROT
// MAXTHROT has to allow enough time at end of PWM for back emf measurement
// was 3976
#define MAXTHROT 3800

#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

// PID control structure
pidPos pidObjs[NUM_PIDS];

// structure for reference velocity for leg
pidVelLUT  pidVel[NUM_PIDS];


#define T1_MAX 0xffffff  // max before rollover of 1 ms counter
// may be glitch in longer missions at rollover
volatile unsigned long t1_ticks;
unsigned long lastMoveTime;
int seqIndex;

//for battery voltage:
char calib_flag = 0;   // flag is set if doing calibration
long offsetAccumulatorL, offsetAccumulatorR;
unsigned int offsetAccumulatorCounter;

MoveQueue moveq;
moveCmdT currentMove, idleMove, manualMove;


// 2 last readings for median filter
int measLast1[NUM_PIDS];
int measLast2[NUM_PIDS];
int bemf[NUM_PIDS]; //used to store the true, unfiltered speed

static struct piddata {
    int output[NUM_PIDS];
    unsigned int measurements[NUM_PIDS];
	long p[2],i[2],d[2];
} PIDTelemData;


// -------------------------------------------
// called from main()
void pidSetup()
{
	int i;
	for(i = 0; i < NUM_PIDS; i++){
		initPIDObjPos( &(pidObjs[i]), DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KAW, DEFAULT_FF); 
	}
	initPIDVelProfile();
	SetupTimer1();  // main interrupt used for leg motor PID

// returns pointer to queue with 8 move entries
	moveq = mqInit(8); 
	idleMove = malloc(sizeof(moveCmdStruct));
	idleMove->inputL = 0;
	idleMove->inputR = 0;
	idleMove->duration = 0;
	currentMove = idleMove;
	
	manualMove = malloc(sizeof(moveCmdStruct));
	manualMove->inputL = 0;
	manualMove->inputR = 0;
	manualMove->duration = 0;

	lastMoveTime = 0;
//  initialize PID structures before starting Timer1
	pidSetInput(0,0,0);
	pidSetInput(1,0,0);

	EnableIntT1; // turn on pid interrupts
	
	calibBatteryOffset(100);
}



// ----------   all the initializations  -------------------------
// set expire time for first segment in pidSetInput - use start time from MoveClosedLoop
// set points and velocities for one revolution of leg
// called from pidSetup()
void initPIDVelProfile()
{ int i,j;
	for(j = 0; j < NUM_PIDS; j++)
	{    	pidVel[j].index = 0;  // point to first velocity
		pidVel[j].interpolate = 0; 
		pidVel[j].leg_stride = 0;  // set initial leg count
  		
		for(i = 0; i < NUM_VELS; i++)
		{   	// interpolate values between setpoints, <<4 for resolution
			pidVel[j].interval[i] = 128;  // 128 ms intervals
		       pidVel[j].delta[i] =  0x1000; // 1/16 rev
			pidVel[j].vel[i] = (pidVel[j].delta[i] << 8) / pidVel[j].interval[i];
		 }
		pidObjs[j].p_input = 0; // initialize first set point 
//		pidObjs[j].v_input = 0; // initialize first velocity
		pidObjs[j].v_input = (int)(((long) pidVel[j].vel[0] * K_EMF) >>8);	//initialize first velocity, scaled
	}
}

// set values from packet - leave previous motor_count, p_input, etc.
// called from cmd.c
void setPIDVelProfile(int pid_num, int *interval, int *delta, int *vel)
{ int i;
	for (i = 0; i < NUM_VELS; i++)
	{ 	pidVel[pid_num].interval[i]= interval[i];
		pidVel[pid_num].delta[i]= delta[i];
		pidVel[pid_num].vel[i]= vel[i];
	}
}

// called from pidSetup()
void initPIDObjPos(pidPos *pid, int Kp, int Ki, int Kd, int Kaw, int ff)
{
    pid->p_input = 0;
    pid->v_input = 0;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    pid->Kp = Kp;
    pid->Ki= Ki;
    pid->Kd = Kd;
    pid->Kaw = Kaw; 
	pid->feedforward = 0;
  pid->output = 0;
    pid->onoff = 0;
	pid->p_error = 0;
	pid->v_error = 0;
	pid->i_error = 0;
}

// called from steeringSetup()
void initPIDObj(pidT *pid, int Kp, int Ki, int Kd, int Kaw, int ff)
{
    pid->input = 0;
    pid->dState = 0;
    pid->iState = 0;
    pid->output = 0;
    pid->y_old = 0;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    //This is just a guess for the derivative filter time constant
    pid->N = 5;
    pid->Kp = Kp;
    pid->Ki= Ki;
    pid->Kd = Kd;
    pid->Kaw = Kaw; 
	pid->feedforward = 0;
    pid->onoff = 0;
    pid->error = 0;
}


// called from set thrust closed loop, etc. Thrust 
void pidSetInput(int pid_num, int input_val, unsigned int run_time){
unsigned long temp;	
/*      ******   use velocity setpoint + throttle for compatibility between Hall and Pullin code *****/
/* otherwise, miss first velocity set point */
    pidObjs[pid_num].v_input = input_val + (int)(( (long)pidVel[pid_num].vel[0] * K_EMF) >> 8);	//initialize first velocity ;
    pidObjs[pid_num].run_time = run_time;
    pidObjs[pid_num].start_time = t1_ticks;
    //zero out running PID values
    pidObjs[pid_num].i_error = 0;
    pidObjs[pid_num].p = 0;
    pidObjs[pid_num].i = 0;
    pidObjs[pid_num].d = 0;
	//Seed the median filter
	measLast1[pid_num] =input_val;
	measLast2[pid_num] =input_val;
	temp = t1_ticks;  // need atomic read due to interrupt 
	lastMoveTime = temp + (unsigned long) run_time;  // only one run time for both sides
	// set initial time for next move set point
  
/*   need to set index =0 initial values */
/* position setpoints start at 0 (index=0), then interpolate until setpoint 1 (index =1), etc */
	pidVel[pid_num].expire = temp + (long) pidVel[pid_num].interval[0];   // end of first interval
	pidVel[pid_num].interpolate = 0;	
/*	pidObjs[pid_num].p_input += pidVel[pid_num].delta[0];	//update to first set point
***  this should be set only after first .expire time to avoid initial transients */
	pidVel[pid_num].index =0; // reset setpoint index
// set first move at t = 0
//	pidVel[0].expire = temp;   // right side
//	pidVel[1].expire = temp;   // left side

}

// from steering?
void pidSetInputSameRuntime(int pid_num, int input_val){
    pidObjs[pid_num].v_input = input_val;
    //zero out running PID values
    pidObjs[pid_num].i_error = 0;
    pidObjs[pid_num].p = 0;
    pidObjs[pid_num].i = 0;
    pidObjs[pid_num].d = 0;
}

// from cmd.c  PID set gains
void pidSetGains(int pid_num, int Kp, int Ki, int Kd, int Kaw, int ff){
    pidObjs[pid_num].Kp  = Kp;
    pidObjs[pid_num].Ki  = Ki;
    pidObjs[pid_num].Kd  = Kd;
    pidObjs[pid_num].Kaw = Kaw;
	pidObjs[pid_num].feedforward = ff;
}

void pidOn(int pid_num){
	pidObjs[pid_num].onoff = 1;
}

// zero position setpoint for both motors (avoids big offset errors)
void pidZeroPos(int pid_num){ 
// disable interrupts to reset state variables
	DisableIntT1; // turn off pid interrupts
	//amsHallSetup(); //  reinitialize rev count and relative zero encoder position for both motors
	pidObjs[pid_num].p_state = 0;
// reset position setpoint as well
	pidObjs[pid_num].p_input = 0;
	pidObjs[pid_num].v_input = 0;
	pidVel[pid_num].leg_stride = 0; // strides also reset 
	EnableIntT1; // turn on pid interrupts
}

// from cmdGetPIDTelemetry
unsigned char* pidGetTelemetry(void){
	PIDTelemData.output[0] = pidObjs[0].output;
	PIDTelemData.output[1] = pidObjs[1].output;
	PIDTelemData.measurements[0] = pidObjs[0].v_state;
	PIDTelemData.measurements[1] = pidObjs[1].v_state;
	PIDTelemData.p[0] = pidObjs[0].p;
	PIDTelemData.p[1] = pidObjs[1].p;
	PIDTelemData.i[0] = pidObjs[0].i;
	PIDTelemData.i[1] = pidObjs[1].i;
	PIDTelemData.d[0] = pidObjs[0].d;
	PIDTelemData.d[1] = pidObjs[1].d;
	return (unsigned char*)&PIDTelemData;
}


// calibrate A/D offset, using PWM synchronized A/D reads inside 
// timer 1 interrupt loop
// BATTERY CHANGED FOR IP2.5 ***** need to fix
void calibBatteryOffset(int spindown_ms){
	long temp;  // could be + or -
	unsigned int battery_voltage;
// save current PWM config
	int tempPDC1 = PDC1;
	int tempPDC2 = PDC2;
	PDC1 = 0; PDC2 = 0;  /* SFR for PWM? */

// save current PID status, and turn off PID control
	short tempPidObjsOnOff[NUM_PIDS];
	tempPidObjsOnOff[0] = pidObjs[0].onoff;
	tempPidObjsOnOff[1] = pidObjs[1].onoff;
	pidObjs[0].onoff = 0; pidObjs[1].onoff = 0;

	delay_ms(spindown_ms); //motor spin-down
	LED_RED = 1;
	offsetAccumulatorL = 0;
	offsetAccumulatorR = 0; 
	offsetAccumulatorCounter = 0; // updated inside servo loop
	calib_flag = 1;  // enable calibration
	while(offsetAccumulatorCounter < 100); // wait for 100 samples
	calib_flag = 0;  // turn off calibration
	battery_voltage = adcGetVbatt();
	//Left
	temp = offsetAccumulatorL;
	temp = temp/(long)offsetAccumulatorCounter;
	pidObjs[0].inputOffset = (int) temp;

	//Right
	temp = offsetAccumulatorR;
	temp = temp/(long)offsetAccumulatorCounter;
	pidObjs[1].inputOffset = (int) temp;

	LED_RED = 0;
// restore PID values
	PDC1 = tempPDC1;
	PDC2 = tempPDC2;
	pidObjs[0].onoff = tempPidObjsOnOff[0];
	pidObjs[1].onoff = tempPidObjsOnOff[1];
}


/*****************************************************************************************/
/*****************************************************************************************/
/*********************** Stop Motor and Interrupts *********************************************/
/*****************************************************************************************/
/*****************************************************************************************/
void EmergencyStop(void)
{
	pidSetInput(0 ,0, 0);
	pidSetInput(1,0,0);
	DisableIntT1; // turn off pid interrupts
       SetDCMCPWM(MC_CHANNEL_PWM1, 0, 0);    // set PWM to zero
       SetDCMCPWM(MC_CHANNEL_PWM2, 0, 0); 
}
	

// -------------------------   control  loop section  -------------------------------

/*********************** Motor Control Interrupt *********************************************/
/*****************************************************************************************/
/*****************************************************************************************/

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) 
{
//  unsigned long time_start, time_end; 
//	time_start =  swatchTic(); 

    if (t1_ticks == T1_MAX) t1_ticks = 0;
    t1_ticks++;
	
	if(t1_ticks > lastMoveTime) // turn off if done running
	{ //	pidSetInput(0, 0, 0);    don't reset state when done run, keep for recording telemetry
		pidObjs[0].onoff = 0;
	//	pidSetInput(1, 0, 0);
		pidObjs[1].onoff = 0;
	}
	else 		// update velocity setpoints if needed - only when running
	{ pidGetSetpoint();}

	pidGetState();

       pidSetControl();
/* better read telemetry at slower tick rate - updated by steering servo */

//	time_end =  swatchToc();
    //Clear Timer1 interrupt flag
    _T1IF = 0;
}


void pidGetSetpoint()
{ int j, index; long temp_v;

	 for(j=0; j < NUM_PIDS; j++)
		{  	index = pidVel[j].index;		
			// update desired position between setpoints, scaled by 256
			pidVel[j].interpolate += pidVel[j].vel[index];

			if (t1_ticks >= pidVel[j].expire)  // time to reach previous setpoint has passed
			{ 	pidVel[j].interpolate = 0;	
				pidObjs[j].p_input += pidVel[j].delta[index];	//update to next set point
				pidVel[j].expire += pidVel[j].interval[index];  // expire time for next interval
				temp_v = ((long)pidVel[j].vel[index] * K_EMF)>>8;  // scale velocity to A/D units
			       pidObjs[j].v_input = (int)(temp_v);	  //update to next velocity 
			
				// got to next index point	
				pidVel[j].index++;
				if (pidVel[j].index >= NUM_VELS) 
				{     pidVel[j].index = 0;
					pidVel[j].leg_stride++;  // one full leg revolution
	/**** maybe need to handle round off in position set point ***/
				}  // loop on index
			}
		}
}

 // select either back emf or backwd diff for vel est
#define VEL_BEMF 1
/* update state variables including motor position and velocity */
void pidGetState()
{   int i;
	long p_state, oldpos[NUM_PIDS], velocity;
// get diff amp offset with motor off at startup time
	if(calib_flag)
	{ 	
// mis wired motor
#define HACK 0  
#if HACK == 0 
		offsetAccumulatorL += adcGetMotorA();  
		offsetAccumulatorR += adcGetMotorB();   
#endif
#if HACK == 1 // swapped motor A,B
		offsetAccumulatorL += adcGetMotorB();  
		offsetAccumulatorR += adcGetMotorA();   // statically, doesn't matter motor turn direction...
#endif
		offsetAccumulatorCounter++; 	}
 // choose velocity estimate  


#if VEL_BEMF == 0    // use first difference on position for velocity estimate
	for(i=0; i<NUM_PIDS; i++)
	{ oldpos[i] = pidObjs[i].p_state; }
#endif

// only works to +-32K revs- might reset after certain number of steps? Should wrap around properly
	for(i =0; i<NUM_PIDS; i++)
	{	//amsGetPos(i);
	      p_state = (long)(encPos[i].pos << 2);		// pos 14 bits 0x0 -> 0x3fff
	      p_state = p_state + (encPos[i].oticks << 16);
		p_state = p_state - (long)(encPos[i].offset <<2); 	// subtract offset to get zero position
		pidObjs[i].p_state = p_state;
	}

#if VEL_BEMF == 0    // use first difference on position for velocity estimate
	for(i=0; i<NUM_PIDS; i++)
	{	velocity = pidObjs[i].p_state - oldpos[i];  // 2**16 * revs per ms
		velocity = velocity >> 6; // divide by 2**16, mult by 2**10 to get approx revs/sec
	      if (velocity > 0x7fff) velocity = 0x7fff; // saturate to int
		if(velocity < -0x7fff) velocity = -0x7fff;	
		pidObjs[i].v_state = (int) velocity;
	}
#endif

 // choose velocity estimate  

#if VEL_BEMF == 1
int measurements[NUM_PIDS];
// Battery: AN0, MotorA AN8, MotorB AN9, MotorC AN10, MotorD AN11
 /*** HACK tih uses channel+1, motor A and B swapped, A wired backwards ****/

#if HACK == 0 
	measurements[0] = pidObjs[0].inputOffset - adcGetMotorA(); // watch sign for A/D? unsigned int -> signed?
     	measurements[1] = pidObjs[1].inputOffset - adcGetMotorB(); // MotorB
#endif
#if HACK == 1 // swapped motor A,B
	measurements[0] = pidObjs[0].inputOffset - adcGetMotorB(); // watch sign for A/D? unsigned int -> signed?
     	measurements[1] = -(pidObjs[1].inputOffset - adcGetMotorA()); // backwds MotorA
#endif
	
//Get motor speed reading on every interrupt - A/D conversion triggered by PWM timer to read Vm when transistor is off
// when motor is loaded, sometimes see motor short so that  bemf=offset voltage
// get zero sometimes - open circuit brush? Hence try median filter
	for(i = 0; i<2; i++) 	// median filter
	{	if(measurements[i] > measLast1[i])	
		{	if(measLast1[i] > measLast2[i]) {bemf[i] = measLast1[i];}  // middle value is median
			else // middle is smallest
	     		{ if(measurements[i] > measLast2[i]) {bemf[i] = measLast2[i];} // 3rd value is median
	        	   else{ bemf[i] = measurements[i];}  // first value is median
            	}
      	}           
		else  // first is not biggest
		{	if(measLast1[i] < measLast2[i]) {bemf[i] = measLast1[i];}  // middle value is median
			else  // middle is biggest
	     		{    if(measurements[i] < measLast2[i]) {bemf[i] = measLast2[i];} // 3rd value is median
		     		else
				{ bemf[i] = measurements[i];  // first value is median			 
				}
			}
		}
	} // end for
// store old values
	measLast2[0] = measLast1[0];  measLast1[0] = measurements[0];
	measLast2[1] = measLast1[1];  measLast1[1] = measurements[1];
   	pidObjs[0].v_state = bemf[0]; 
	pidObjs[1].v_state = bemf[1];  //  might also estimate from deriv of pos data
    //if((measurements[0] > 0) || (measurements[1] > 0)) {
    if((measurements[0] > 0)) { LED_BLUE = 1;}
    else{ LED_BLUE = 0;}
#endif
}


void pidSetControl()
{ int j;
// 0 = right side
    for(j=0; j < NUM_PIDS; j++)
   {  //pidobjs[0] : right side
	// p_input has scaled velocity interpolation to make smoother
	// p_state is [16].[16]
        	pidObjs[j].p_error = pidObjs[j].p_input + pidVel[j].interpolate  - pidObjs[j].p_state;
            pidObjs[j].v_error = pidObjs[j].v_input - pidObjs[j].v_state;  // v_input should be revs/sec
            //Update values
            UpdatePID(&(pidObjs[j]));
       } // end of for(j)

  /*** HACK tih uses channel+1, motor A and B swapped, A wired backwards ****/
		if(pidObjs[0].onoff && pidObjs[1].onoff)  // both motors on to run
		{
#if HACK == 0
 		   tiHSetDC(1, pidObjs[0].output); 
		   tiHSetDC(2, pidObjs[1].output); 
#endif
# if HACK ==1
	 // change sign if motor is wired backwards...
		tiHSetDC(1, -pidObjs[1].output); 
		   tiHSetDC(2, pidObjs[0].output); 
#endif
		} 
		else // turn off motors if PID loop is off
		{ tiHSetDC(1,0); tiHSetDC(2,0); }	
}


void UpdatePID(pidPos *pid)
{
    pid->p = ((long)pid->Kp * pid->p_error) >> 12 ;  // scale so doesn't over flow
    pid->i = (long)pid->Ki  * pid->i_error;
    pid->d=(long)pid->Kd *  (long) pid->v_error;
    // better check scale factors

    pid->preSat = pid->feedforward + pid->p +
		 ((pid->i ) >> 4) +  // divide by 16
		(pid->d >> 4); // divide by 16
	pid->output = pid->preSat;
 
	pid-> i_error = (long)pid-> i_error + (long)pid->p_error; // integrate error
// saturate output - assume only worry about >0 for now
// apply anti-windup to integrator  
	if (pid->preSat > MAXTHROT) 
	{ 	      pid->output = MAXTHROT; 
			pid->i_error = (long) pid->i_error + 
				(long)(pid->Kaw) * ((long)(MAXTHROT) - (long)(pid->preSat)) 
				/ ((long)GAIN_SCALER);		
	}      
	if (pid->preSat < -MAXTHROT)
      { 	      pid->output = -MAXTHROT; 
			pid->i_error = (long) pid->i_error + 
				(long)(pid->Kaw) * ((long)(MAXTHROT) - (long)(pid->preSat)) 
				/ ((long)GAIN_SCALER);		
	}      
}


