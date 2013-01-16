/* Telemetry related functions */
/* since telemetry is called in steering.c */

typedef struct {
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



void telemSaveSample(void);
void telemFlashSample(telemU* data);
void setSampleSaveCount(int count);
void telemFlashReadback(unsigned int count);
