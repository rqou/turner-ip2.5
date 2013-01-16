/* Telemetry related functions */
/* since telemetry is called in steering.c */
#include "pid-ip2.5.h"
#include "utils.h"
#include "telemetry.h"
#include "stopwatch.h"
#include "dfmem.h"
#include "radio.h"
#include "../MyConsts/radio_settings.h"
#include "timer.h"
#include "cmd.h"
#include "adc_pid.h"

int samplesToSave;
extern int gdata[3];
extern int gyroAvg;
extern int xldata[3];  // accelerometer data 
extern int offsx, offsy, offsz;

// structure to keep track of telemetry recording
TelemStruct TelemControl;
extern pidT steeringPID;
extern pidPos pidObjs[NUM_PIDS];
extern int bemf[NUM_PIDS];

// record current state to telemU structure
void telemSaveSample(void)
{	telemU data;
//Stopwatch was already started in the cmdSpecialTelemetry function
			data.telemStruct.timeStamp = (long)swatchTic(); 

// since T1 has higher priority, these state readings might get interrupted 
	CRITICAL_SECTION_START  // need coherent sample without T1 int updates
//  save Hall encoder position instead of commanded thrust
		data.telemStruct.posL = pidObjs[0].p_state;
		data.telemStruct.posR = pidObjs[1].p_state;


	// save output instead of reading PWM (sync issue?)
			data.telemStruct.dcL = pidObjs[0].output;	// left
			data.telemStruct.dcR = pidObjs[1].output;	// right
			data.telemStruct.bemfL = bemf[0];
			data.telemStruct.bemfR = bemf[1];
	CRITICAL_SECTION_END
   
			data.telemStruct.gyroX = gdata[0] - offsx;
			data.telemStruct.gyroY = gdata[1] - offsy;
			data.telemStruct.gyroZ = gdata[2] - offsz; 
			data.telemStruct.gyroAvg = gyroAvg;

			data.telemStruct.accelX = xldata[0];
			data.telemStruct.accelY = xldata[1];
			data.telemStruct.accelZ = xldata[2];
			data.telemStruct.Vbatt = (int) adcGetVbatt();
			data.telemStruct.sOut = steeringPID.output;
// inside T5 interrupt, so don't need to DisableIntT5
			telemFlashSample(&data); 
}

/// write telemetry sample to Flash memory
void telemFlashSample(telemU* data)
{  dfmemSave((unsigned char *)data, sizeof(telemStruct_t));
	if(samplesToSave == 0) //Done sampling, commit last buffer
	{ dfmemSync(); }
			
}

void setSampleSaveCount(int count){
	samplesToSave = count;
}


//read telemetry from Flash Memory and send radio packets back
void telemFlashReadback(unsigned int count)
{  unsigned char status = 0;
	unsigned int sampLen = sizeof(telemStruct_t);
	unsigned long sampNum = 0;
	telemU data;
	DisableIntT5;		// prevent MPU access to SPI2
	for(sampNum = 0; sampNum < count; sampNum++)
	{ dfmemReadSample(sampNum, sampLen, (unsigned char *) &data); 
	   radioConfirmationPacket(RADIO_DEST_ADDR,
						     CMD_SPECIAL_TELEMETRY, 
						     status, sampLen, (unsigned char *) &data);  
	delay_ms(25);	// slow down for XBee 57.6 K
	}
	EnableIntT5;
	
}
