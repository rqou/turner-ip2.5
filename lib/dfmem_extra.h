#ifndef __DFMEM_EXTRA_H
#define __DFMEM_EXTRA_H

//#include <dfmem.h>

// DataFlash Memory
#define MAXSECTOR 15
#define MAXPAGES 4096
#define BUFFERSIZE 264
//#define PACKETSIZE 	26	
		// My Packets (bytes)  , (4*1 + 2*11)
		//Done automatically below based on sizeof(telemStruct_t)
#define BUFFERSIZETHRESHOLD 264		// Modify

//Types
/* old type
typedef union packedTelemUnion {
	struct {
		unsigned long timeStamp; 
		int inputL;  
		int inputR;
		int dcL;
		int dcR;
		int gyroX;
		int gyroY;
		int gyroZ;
		int gyroAvg;
		int bemfL;
		int bemfR;
		int sOut;	
	} telemStruct;
	unsigned char dataArray[PACKETSIZE];
} telemU; */
//New type

typedef struct {
		unsigned long timeStamp; 
		int inputL;  
		int inputR;
		int dcL;
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
		int sOut;	
	} telemStruct_t;

typedef union packedTelemUnion {
	telemStruct_t telemStruct;
	unsigned char dataArray[sizeof(telemStruct_t)];
} telemU;

#define PACKETSIZE sizeof(telemStruct_t)

// Prototypes
void eraseDFMemSectors0a0b(void);
void readDFMem(void);
void readDFMemBySample(int);
void sendDataDelay(unsigned char, unsigned char*);
void saveTelemData(telemU *data);


#endif  // __DFMEM_EXTRA_H
