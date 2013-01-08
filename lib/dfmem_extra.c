#include "cmd.h"
#include "utils.h"
#include "dfmem.h"
#include "dfmem_extra.h"
#include "payload.h"
#include "radio.h"

unsigned char currentBuffer = 0;
unsigned int currentBufferLength = 0;
unsigned int nextPage = 0;

extern int samplesToSave;

//int flags[45];

// Erase entire DataFlash memory sector by sector (LED to signal erasing)
void eraseDFMemSectors0a0b(void)
{	
	//_LATB14 = 1;
	LED_2 = 1;
	//erase first three sectors
	//This erases enoough for 3584 samples
	dfmemEraseSector(0);
	dfmemEraseSector(8);
	dfmemEraseSector(256);
	while(!dfmemIsReady()){
		LED_2 = ~LED_2;
		delay_ms(100);
	}
	LED_2 = 0;
	currentBuffer = 0;
	currentBufferLength = 0;
	nextPage = 0;
	//for (page = 128; page < MAXPAGES; page += 128)
	//{
	//	dfmemEraseSector(page);
	//}
	//_LATB14 = 0;
}


// Read out contents of DataFlash memory, returning over radio (LED to signal reading)
void readDFMem(void)
{
	unsigned int page, bufferByte;
	unsigned char dataPacket[PACKETSIZE];
	LED_2 = 1;

	// Read out all written pages
	for (page = 0; page < nextPage; ++page)
	{
		for (bufferByte = 0; bufferByte < BUFFERSIZETHRESHOLD; bufferByte += PACKETSIZE)
		{
			dfmemRead(page, bufferByte, PACKETSIZE, dataPacket);
			sendDataDelay(PACKETSIZE, dataPacket);
		}
	}

	LED_2 = 0;
}

void readDFMemBySample(int num)
{
	unsigned int page, bufferByte;// maxpage;
	unsigned char dataPacket[PACKETSIZE];
	unsigned long bytesleft = PACKETSIZE * num;
	unsigned int i;

	//maxpage = (num * PACKETSIZE) / BUFFERSIZE + 1;
	bufferByte = 0;
	page = 0;


	LED_2 = 1;
	_T1IE = 0; _T5IE=0;
	while(!dfmemIsReady());
	

	// Read out all written pages
	//for (page = 0; page < maxpage; ++page)
	//{
	//	for (bufferByte = 0; bufferByte < BUFFERSIZETHRESHOLD; bufferByte += PACKETSIZE)
	//	{
	//		dfmemRead(page, bufferByte, PACKETSIZE, dataPacket);
	//		sendDataDelay(PACKETSIZE, dataPacket);
	//	}
	//}
	
	for(i = 0; i < num; i++){
		if((bufferByte + PACKETSIZE) >= BUFFERSIZE){
			bufferByte = 0;
			page++;
		}
		//flags[i] = 1;
		dfmemRead(page, bufferByte, PACKETSIZE, dataPacket);
		sendDataDelay(PACKETSIZE, dataPacket);
		bufferByte += PACKETSIZE;
	}
	_T1IE = 1; _T5IE=1;
	LED_2 = 0;
}

//#define CMD_SPECIAL_TELEMETRY       0x89
void sendDataDelay(unsigned char data_length, unsigned char* data)
{
	// Create Payload, set status and type (don't cares)
	MacPacket packet; Payload pld = payCreateEmpty(data_length);
	//////    FIX THIS //////////
    paySetType(pld, CMD_SPECIAL_TELEMETRY);			// Don't Care
    paySetStatus(pld, 0);		// Don't Care
    
	// Set Payload data
	paySetData(pld, data_length, data);
    
	// Send Payload WITH 15ms DELAY
	// Handles pld delete: Assigns pointer to payload in packet
	//    and radio command deletes payload, then packet.
	delay_ms(25); 	// Acommodate 57600 baud rate of XBee
//	radioSendPayload(macGetDestAddr(), pld);
    // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));
}

void saveTelemData(telemU *data){
	if (nextPage < MAXPAGES)
	{
		// If buffer full, write to next page
		if ((currentBufferLength + PACKETSIZE) >= BUFFERSIZETHRESHOLD)
		{
			// Transfer current buffer to next page (if available)
			dfmemWriteBuffer2MemoryNoErase(nextPage, currentBuffer);
			++nextPage;
	
			// Toggle and reset buffer
			currentBuffer = (currentBuffer) ? 0 : 1;
			currentBufferLength = 0;
		}

		dfmemWriteBuffer(data->dataArray, PACKETSIZE, currentBufferLength, currentBuffer);
		currentBufferLength += PACKETSIZE;
		
		if(samplesToSave == 0){
			//Done sampling, commit last buffer
			dfmemWriteBuffer2MemoryNoErase(nextPage, currentBuffer);
		}
	}
}
