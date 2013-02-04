/* Telemetry related functions */
/* since telemetry is called in steering.c */

typedef struct {
		unsigned long sampleIndex;
	       unsigned long timeStamp; 
		long posL;  	// Hall angle position
		long posR;
		int dcL;		// PWM duty cycle
		int dcR;
		int gyroX;
		int gyroY;
		int gyroZ;
		int gyroAvg;
		int accelX;
		int accelY;
		int accelZ;
		int bemfL;
		int bemfR;
		int Vbatt; // battery voltage
		int sOut;	
	} telemStruct_t;

typedef union packedTelemUnion {
	telemStruct_t telemStruct;
	unsigned char dataArray[sizeof(telemStruct_t)];
} telemU;


// telemetry control structure 
typedef struct
{	char onoff;				// telemetry recording enabled 
	unsigned long start;	// recording start time   
	int count;				// count of samples to record
	int skip;				// samples to skip
} TelemConStruct;


void telemGetPID(unsigned long sampIdx);
void telemSaveSample(unsigned long sampIdx);
void telemFlashSample(telemU* data);
void setSampleSaveCount(int count);
void telemFlashReadback(unsigned int count);
