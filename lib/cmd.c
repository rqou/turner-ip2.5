/***************************************************************************
* Name: cmd.c
* Desc: Receiving and transmitting queue handler
* Date: 2010-07-10
* Author: stanbaek
**************************************************************************/

#include "cmd.h"
#include "cmd_const.h"
#include "dfmem.h"
#include "utils.h"
#include "ports.h"
#include "gyro.h"
#include "xl.h"
#include "stopwatch.h"
#include "led.h"
#include "payload.h"
#include "mac_packet.h"
#include "dfmem.h"
#include "pid-ip2.5.h"
#include "radio.h"
#include "move_queue.h"
#include "steering.h"
#include "dfmem.h"
#include "tests.h"
#include "queue.h"
#include "version.h"
#include "../MyConsts/radio_settings.h"
#include "tiH.h"
#include "timer.h"


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

unsigned char tx_frame_[127];

extern MoveQueue moveq;
extern int offsz;
extern pidPos pidObjs[NUM_PIDS];
extern TelemStruct TelemControl;
extern unsigned long t1_ticks;
extern int samplesToSave;

extern moveCmdT currentMove, idleMove, manualMove;
// updated version string to identify robot
//extern static char version[];

// use an array of function pointer to avoid a number of case statements
// MAX_CMD_FUNC_SIZE is defined in cmd_const.h
// arg to all commands are: type, status, length, *frame
void (*cmd_func[MAX_CMD_FUNC_SIZE])(unsigned char, unsigned char, unsigned char, unsigned char*);
void cmdError(void);
/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/
static void cmdSetThrust(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSteer(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdGetImuData(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdGetImuLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);

static void cmdStartImuDataSave(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdStopImuDataSave(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdTxSavedImuData(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdEraseMemSector(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);

//static void cmdEcho(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
void cmdEcho(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);

static void cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);

//User commands
static void cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSetThrustClosedLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSetPIDGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdGetPIDTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSetCtrldTurnRate(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdGetImuLoopZGyro(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSetMoveQueue(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSetSteeringGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSoftwareReset(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSpecialTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdEraseSector(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdFlashReadback(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSetVelProfile(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdWhoAmI(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdStartTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdZeroPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);

/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/
void cmdSetup(void) {

    unsigned int i;

    // initialize the array of func pointers with Nop()
    for(i = 0; i < MAX_CMD_FUNC_SIZE; ++i) {
        cmd_func[i] = &cmdNop;
    }

    cmd_func[CMD_ECHO] = &cmdEcho;
    cmd_func[CMD_SET_THRUST] = &cmdSetThrust;
    cmd_func[CMD_SET_STEER] = &cmdSteer;    
    cmd_func[CMD_GET_IMU_DATA] = &cmdGetImuData;
    cmd_func[CMD_GET_IMU_LOOP] = &cmdGetImuLoop;
    cmd_func[CMD_START_IMU_SAVE] = &cmdStartImuDataSave;
    cmd_func[CMD_STOP_IMU_SAVE] = &cmdStopImuDataSave;
    cmd_func[CMD_TX_SAVED_IMU_DATA] = &cmdTxSavedImuData;
    cmd_func[CMD_ERASE_MEM_SECTOR] = &cmdEraseMemSector;
	//Use commands
	cmd_func[CMD_SET_THRUST_OPENLOOP] = &cmdSetThrustOpenLoop;
	cmd_func[CMD_SET_THRUST_CLOSEDLOOP] = &cmdSetThrustClosedLoop;
	cmd_func[CMD_SET_PID_GAINS] = &cmdSetPIDGains;
	cmd_func[CMD_GET_PID_TELEMETRY] = &cmdGetPIDTelemetry;
	cmd_func[CMD_SET_CTRLD_TURN_RATE] = &cmdSetCtrldTurnRate;
	cmd_func[CMD_GET_IMU_LOOP_ZGYRO] = &cmdGetImuLoopZGyro;
	cmd_func[CMD_SET_MOVE_QUEUE] = &cmdSetMoveQueue;
	cmd_func[CMD_SET_STEERING_GAINS] = &cmdSetSteeringGains;
	cmd_func[CMD_SOFTWARE_RESET] = &cmdSoftwareReset;
	cmd_func[CMD_SPECIAL_TELEMETRY] = &cmdSpecialTelemetry;
	cmd_func[CMD_ERASE_SECTORS] = &cmdEraseSector;
	cmd_func[CMD_FLASH_READBACK] = &cmdFlashReadback;
	cmd_func[CMD_SET_VEL_PROFILE] = &cmdSetVelProfile;
	cmd_func[CMD_WHO_AM_I] = &cmdWhoAmI;
	cmd_func[CMD_START_TELEM] = &cmdStartTelemetry;
	cmd_func[CMD_ZERO_POS] = &cmdZeroPos;
}

// Jan 2013- new command handler using function queue
void cmdPushFunc(MacPacket rx_packet)
{   Payload rx_payload;
    unsigned char command, status;  
	 rx_payload = macGetPayload(rx_packet);
	 
	 Test* test = (Test*) malloc(sizeof(Test));
        if(!test) return;
	  test->packet = rx_packet;

        command = payGetType(rx_payload);
	   if( command < MAX_CMD_FUNC_SIZE)
	  {     test->tf=cmd_func[command];
		   queuePush(fun_queue, test); 
	  }   
	  else 
	 {  cmdError();   // halt on error - could also just ignore....
	 }

}

/* not used since now have function queue
void cmdHandleRadioRxBuffer(void) {
    Payload pld;
    unsigned char command, status;  

    if ((pld = radioReceivePayload()) != NULL) {
        status = payGetStatus(pld);
       command = payGetType(pld);      

        if( command < MAX_CMD_FUNC_SIZE)
	{        cmd_func[command](status, pld->data_length, payGetData(pld)); }
//	else { cmdError(); }
	else 
	{  cmdNop(status, pld->data_length, payGetData(pld));
	    cmdError();
	}
        payDelete(pld);
    } 
    return;
}
*******/

// handle bad command packets
// we might be in the middle of a dangerous maneuver- better to stop and signal we need resent command
// wait for command to be resent
void cmdError()
{ int i;
 	EmergencyStop();
	for(i= 0; i < 10; i++)
	 {	LED_1 ^= 1;
			delay_ms(200);
			LED_2 ^= 1;
			delay_ms(200);
			LED_3 ^= 1;
			delay_ms(200);
          }
}

/*-----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * The functions below are intended for internal use, i.e., private methods.
 * Users are recommended to use functions defined above.
 * ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

// set PWM values for short duration for each motor
// throttle[0], throttle[1], duration
static void cmdSetThrust(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
 {	int thrust1 = frame[0] + (frame[1] << 8);
	int thrust2 = frame[2] + (frame[3] << 8);
	unsigned int run_time_ms = frame[4] + (frame[5] << 8);

	DisableIntT1;	// since PID interrupt overwrites PWM values
// wiring was scrambled Jan. 9, 2013 on RSF velociRoACH robot
  	tiHSetDC(1, -thrust2);
	tiHSetDC(2, thrust1); 
	delay_ms(run_time_ms);
	tiHSetDC(1,0);
	tiHSetDC(2,0);
	EnableIntT1;
 }  
	


static void cmdSteer(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
    
    unsigned char chr_test[4];
    float *steer_value = (float*)chr_test;

    chr_test[0] = frame[0];
    chr_test[1] = frame[1];
    chr_test[2] = frame[2];
    chr_test[3] = frame[3];
/*
    mcSteer(steer_value[0]);
*/
}


/*-----------------------------------------------------------------------------
 *          IMU functions
-----------------------------------------------------------------------------*/
static void cmdGetImuData(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {

//	senGetIMUData(status, CMD_GET_IMU_DATA);


    /*unsigned char *xl_data, *gyro_data;
    unsigned char i;
    Payload pld = payCreateEmpty(14);  // data length = 12

    xlReadXYZ();
    xl_data = xlGetsXYZ();
    gyroReadXYZ();
    gyro_data = gyroGetsXYZ();
    for(i = 0; i < 6; ++i) {
        pld->pld_data[i] = xl_data[i];
        pld->pld_data[i+6] = gyro_data[i];
    }

    pld->status = status;
    pld->type = CMD_GET_IMU_DATA;
    radioTxPayload(pld);
    */
}


// return packet format:
// 4 bytes for time
// 6 bytes for xl data
// 6 bytes for gyro data
static void cmdGetImuLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
/*
    unsigned int count;
    unsigned long tic;
    unsigned char *tic_char;
    Payload pld;
  
    LED_RED = 1;

    count = frame[0] + (frame[1] << 8);

    tic_char = (unsigned char*)&tic;
    swatchReset();
    tic = swatchTic();
    
    while (count) {
        pld = payCreateEmpty(16);  // data length = 16
        paySetData(pld, 4, tic_char);
        payAppendData(pld, 4, 6, xlReadXYZ());
        payAppendData(pld,10, 6, gyroReadXYZ());
        paySetStatus(pld, status);
        paySetType(pld, CMD_GET_IMU_DATA);

        radioSendPayload(macGetDestAddr(),pld);
        count--;
		payDelete(pld);
        delay_ms(4);  
        tic = swatchTic();
    }
    LED_RED = 0;
*/
}


static void cmdStartImuDataSave(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
//    senSetImuDataSave(1);  
}

static void cmdStopImuDataSave(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
  //  senSetImuDataSave(0);  
}

static void cmdTxSavedImuData(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
 /*  
    unsigned int page, byte;
    unsigned int i, j;
    Payload pld;

    //senGetMemLocIndex(&page, &byte);
    page = 0x0200;
    byte = 0;

    LED_RED = 1;

    dfmemEraseSector(0x0100);   // erase Sector 1 (page 256 - 511)

    for (i = 0x0100; i < 0x0200; ++i) {
        j = 0;
        while (j < 512) {
            pld = payCreateEmpty(18);  // data length = 16
            dfmemRead(i, j, 16, pld->pld_data);
            paySetStatus(pld, status);
            paySetType(pld, CMD_GET_IMU_DATA);
            while(!radioReceivePayload());
            j += 16;
        }
        delay_ms(200);
    }
    LED_RED = 0;
*/
}



static void cmdEraseMemSector(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
    unsigned int page;
	page = frame[0] + (frame[1] << 8);
    LED_RED = 1;
    dfmemEraseSector(0x0100);   // erase Sector 1 (page 256 - 511)
    LED_RED = 0;
}





/*-----------------------------------------------------------------------------
 *          AUX functions
-----------------------------------------------------------------------------*/
void cmdEcho(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) 
{ // MacPacket packet; Payload pld;
	//Send confirmation packet
	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_ECHO, status, length, frame);  
    return; //success     
}

static void cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
    Nop();
}


/*-----------------------------------------------------------------------------
 *         User function
-----------------------------------------------------------------------------*/
static void cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	//unsigned char chan1[4], chan2[4];
    //float *duty_cycle1 = (float*)chan1;
	//float *duty_cycle2 = (float*)chan2;
	int dc1, dc2;

	dc1 = *(int*)(frame);
	dc2 = *(int*)(frame + sizeof(int) );

    /*chan2[0] = frame[0];
    chan2[1] = frame[1];
    chan2[2] = frame[2];
    chan2[3] = frame[3];

	chan1[4] = frame[4];
    chan1[5] = frame[5];
    chan1[6] = frame[6];
    chan1[7] = frame[7];*/
    
	PDC1 = dc1;
	PDC2 = dc2;

    //mcSetDutyCycle(MC_CHANNEL_PWM1, duty_cycle1[0]);
	//mcSetDutyCycle(MC_CHANNEL_PWM2, duty_cycle2[0]);
	//mcSetDutyCycle(1, duty_cycle[0]);
}

static void cmdSetThrustClosedLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
{	int thrust1 = frame[0] + (frame[1] << 8);
	unsigned int run_time_ms1 = frame[2] + (frame[3] << 8);
	int thrust2 = frame[4] + (frame[5] << 8);
	unsigned int run_time_ms2 = frame[6] + (frame[7] << 8);
	//currentMove = manualMove;
	pidSetInput(0 ,thrust1, run_time_ms1);
	pidOn(0);
	pidSetInput(1 ,thrust2, run_time_ms2);
	pidOn(1);
}

static void cmdSetPIDGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	int Kp, Ki, Kd, Kaw, ff;
	int idx = 0;

	Kp = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Ki = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kd = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kaw = frame[idx] + (frame[idx+1] << 8); idx+=2;
	ff = frame[idx] + (frame[idx+1] << 8); idx+=2;
	pidSetGains(0,Kp,Ki,Kd,Kaw, ff);
	Kp = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Ki = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kd = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kaw = frame[idx] + (frame[idx+1] << 8); idx+=2;
	ff = frame[idx] + (frame[idx+1] << 8); idx+=2;
	pidSetGains(1,Kp,Ki,Kd,Kaw, ff);

	//Send confirmation packet
	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_SET_PID_GAINS, status, 20, frame);  
      return; //success
}

static void cmdGetPIDTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	unsigned int count;
	unsigned long tic;
    unsigned char *tic_char = (unsigned char*)&tic;
	//unsigned long sampNum = 0;
	int i;
	unsigned short idx = 0;
	MacPacket packet; Payload pld;
	unsigned char* telem_ptr;

	count = frame[0] + (frame[1] << 8);
	swatchReset();
    tic = swatchTic();
	
	while(count){
		pld = payCreateEmpty(36);  // data length = 12
	
		//*(long*)(pld->pld_data + idx) = tic;
		pld->pld_data[2] = tic_char[0];
        pld->pld_data[3] = tic_char[1];
        pld->pld_data[4] = tic_char[2];
        pld->pld_data[5] = tic_char[3];
		idx += sizeof(tic);

		telem_ptr = pidGetTelemetry();

        
		//memcpy((pld->pld_data)+idx , telem_ptr, 4*sizeof(int));
		for(i = 0; i < (4*sizeof(int)+6*sizeof(long)); ++i) {
            pld->pld_data[i+6] = telem_ptr[i];
        }

        pld->pld_data[0] = status;
        pld->pld_data[1] = CMD_GET_PID_TELEMETRY;
//        radioSendPayload(macGetDestAddr(), pld);
      // Enqueue the packet for broadcast
    	while(!radioEnqueueTxPacket(packet));

	   count--;
        //delay_ms(2);   // ~3ms delay
        //delay_us(695);
		delay_ms(10);
        tic = swatchTic();
	}
}

static void cmdSetCtrldTurnRate(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	int rate;
	int idx = 0;
	MacPacket packet; Payload pld;
	rate = frame[0] + (frame[1] << 8);
	setSteeringAngRate(rate);
	
	//Send confirmation packet
	pld = payCreateEmpty(sizeof(int));
	pld->pld_data[0] = status;
    pld->pld_data[1] = CMD_SET_CTRLD_TURN_RATE;
	memcpy((pld->pld_data)+2, frame, sizeof(int));
//	radioSendPayload((WordVal)macGetDestAddr(), pld);	
// ip2.5c radio command:

    // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));
}


static void cmdGetImuLoopZGyro(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
    unsigned int count;     unsigned long tic;   unsigned char *tic_char;
    MacPacket packet; Payload pld;
/*
    unsigned char* gyrotemp;
	int zgyro;
    //int perpacket = 10;
	int perpacket = 2;
    int i;
    int packidx = 0;
    int dc1, dc2;
    //LED_RED = 1;
	int intwz;
	unsigned char gdata[6];
	unsigned char* gdatap = gdata;
	int* zp = (int*)(gdatap + 4);
    count = frame[0] + (frame[1] << 8);
    tic_char = (unsigned char*)&tic;
    swatchReset();

    while (count) {
        pld = payCreateEmpty(10*perpacket); 
        packidx = 0;
        for(i = 0; i < perpacket; i++){

            tic = swatchTic();
            //if (tic == 4) //testing why tic seems to reset at ~1.2 seconds
            //{
            //        asm volatile("nop");
            //        asm volatile("nop");
            //}
			gyroGetXYZ(gdata);
			intwz = *zp - offsz;

            payAppendData(pld, packidx, 4, tic_char); //time , long , 4 bytes
            packidx += 4;
            payAppendData(pld, packidx, 2, (unsigned char*)(&intwz) );  //gyro data , int, 2 bytes
            packidx += 2;
            dc1 = PDC1; dc2 = PDC2;
            payAppendData(pld, packidx, 2, (unsigned char*)(&dc1) );  //duty cycle 1, int, 2 bytes
            packidx += 2;
            payAppendData(pld, packidx, 2, (unsigned char*)(&dc2) );  //duty cycle 2 , int, 2 bytes
            packidx += 2;
            count--;
            delay_ms(4);
        }
		
        paySetStatus(pld, status);
        paySetType(pld, CMD_GET_IMU_LOOP_ZGYRO);

   //     radioSendPayload(macGetDestAddr(),pld);
         // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));

    }

    //LED_RED = 0;
*/
}

static void cmdSetMoveQueue(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
	unsigned int count;
	int idx = 0;
	count = (unsigned int)(*(frame+idx));
	idx += 2;
	moveCmdT move;
	int i;
	unsigned char durtemp[4];
	unsigned long* dur = (unsigned long*)durtemp; // requires for unsigned long read
	
	for(i = 0; i < count; i++){
		move = (moveCmdT)malloc(sizeof(moveCmdStruct));
		//move->inputL = (int)(*(frame+idx));
		move->inputL = frame[idx] + (frame[idx+1] << 8); idx+=2;
		//idx += 2;
		//move->inputR = (int)(*(frame+idx));
		move->inputR = frame[idx] + (frame[idx+1] << 8); idx+=2;
		//idx += 2;
		//apparently this is required to properly read the unsigned long.
		durtemp[0] = frame[idx];
		durtemp[1] = frame[idx+1];
		durtemp[2] = frame[idx+2];
		durtemp[3] = frame[idx+3];
		move->duration = dur[0];
		idx += 4;
		mqPush(moveq, move);
	}
}

static void cmdSetSteeringGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	int Kp, Ki, Kd, Kaw, ff;
	int idx = 0;
	MacPacket packet; Payload pld;
	Kp = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Ki = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kd = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kaw = frame[idx] + (frame[idx+1] << 8); idx+=2;
	ff = frame[idx] + (frame[idx+1] << 8); idx+=2;
	steeringSetGains(Kp,Ki,Kd,Kaw, ff);
	//Send confirmation packet
	pld = payCreateEmpty(10);
	pld->pld_data[0] = status;
    pld->pld_data[1] = CMD_SET_STEERING_GAINS;
	memcpy((pld->pld_data)+2, frame, 10);
//	radioSendPayload((WordVal)macGetDestAddr(), pld);
// ip2.5c radio command:

    // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));
}

static void cmdSoftwareReset(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
{
	asm volatile("reset");
}

static void cmdSpecialTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
/*	int count;
	unsigned long temp;
	//count = (int)(*(frame));
	count = frame[0] + (frame[1] << 8);
	if(count != -1){
		swatchReset();
		setSampleSaveCount(count);
	       temp = t1_ticks;  // need atomic read due to interrupt 
		TelemControl.start = temp; // start recording now
		TelemControl.skip = 1; // every other sample (150 Hz)
	} else{ //start a readback over the radio
		//shut down all movement before sending
		pidSetInput(0,0,0);
		pidSetInput(1,0,0);
		readDFMem();
	}
*/
}



static void cmdEraseSector(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
{
//	eraseDFMemSectors0a0b();
}

static void cmdFlashReadback(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	unsigned int count = frame[0] + (frame[1] << 8);
//	readDFMemBySample(count);
}


// set up velocity profile structure  - assume 4 set points for now, generalize later
static void cmdSetVelProfile(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	int interval[NUM_VELS], delta[NUM_VELS], vel[NUM_VELS];
	int idx = 0, i = 0;
	for(i = 0; i < NUM_VELS; i ++)
	{ interval[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2;	}
	for(i = 0; i < NUM_VELS; i ++)
	{ delta[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2; 	}
	for(i = 0; i < NUM_VELS; i ++)
	{ vel[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2; 	}
	setPIDVelProfile(0, interval, delta, vel);
	for(i = 0; i < NUM_VELS; i ++)
	{ interval[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2;	}
	for(i = 0; i < NUM_VELS; i ++)
	{ delta[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2; 	}
	for(i = 0; i < NUM_VELS; i ++)
	{ vel[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2; 	}
	setPIDVelProfile(1, interval, delta, vel);

	//Send confirmation packet
	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_SET_VEL_PROFILE, status, 48, frame);  
     return; //success
}

// send robot info when queried
void cmdWhoAmI(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) 
{   unsigned char i, string_length; unsigned char *version_string;
// maximum string length to avoid packet size limit
	version_string = (unsigned char *)versionGetString();
	i = 0;
	while((i < 127) && version_string[i] != '\0')
	{ i++;}
	string_length=i;     
	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_WHO_AM_I, status, string_length, version_string);  
      return; //success
}

// report motor position and  reset motor position (from Hall angular sensors)
// note motor_count is long (4 bytes) revs+frac rev
void cmdZeroPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) 
{ 	long motor_count[2]; 
	motor_count[0] = pidObjs[0].p_state;
	motor_count[1] = pidObjs[1].p_state;

	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_ZERO_POS,\
		 status, sizeof(motor_count), (unsigned char *)motor_count);  
     pidZeroPos(0); pidZeroPos(1);
}

// alternative telemetry which runs at 1 kHz rate inside PID loop
static void cmdStartTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	int idx=0;
       unsigned long temp;
     TelemControl.count = frame[idx] + (frame[idx+1] << 8); idx+=2;
   // start time is relative to current t1_ticks
	temp = t1_ticks; // need atomic read due to interrupts
	TelemControl.start = 
		(unsigned long) (frame[idx] + (frame[idx+1] << 8) )
						+ temp;
	idx+=2;
      samplesToSave = TelemControl.count; // **** this runs sample capture in T5 interrupt
	TelemControl.skip = frame[idx]+(frame[idx+1]<<8); 
	swatchReset();
	if(TelemControl.count > 0) 
	{ TelemControl.onoff = 1;   // use just steering servo sample capture
	 } // enable telemetry last 
}
