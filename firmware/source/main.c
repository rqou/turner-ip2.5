/******************************************************************************
 * Name: main.c
 * Desc:
 * Date: 2010-07-08
 * Author: stanbaek
 *******************************************************************************/

#include "settings.h"
#include "Generic.h"
#include "p33Fxxxx.h"
#include "init_default.h"
#include "ports.h"
#include "battery.h"
#include "cmd.h"
#include "radio.h"
#include "utils.h"
#include "stopwatch.h"
#include "motor_ctrl.h"
#include "led.h"
#include "dfmem.h"
#include "pid.h"
#include "adc_pid.h"
#include "steering.h"
#include "telem.h"
#include "hall.h"
#include "tih.h"
#include "amsCtrl.h"
#include "ams-enc.h"

#include <stdlib.h>

extern unsigned char id[4];
extern ENCPOS encPos[NUM_ENC];
extern pidObj amsPID[nPIDS];

volatile unsigned long wakeTime;
extern volatile char g_radio_duty_cycle;
extern volatile char inMotion;
extern byte phyReadReg(byte addr);

int dcCounter;

int main(void) {

    SetupClock();
    SwitchClocks();
    SetupPorts();
    amsPIDSetup();
    encSetup();
    tiHSetup();
//  mpuSetup();
//    amsCtrlSetGains(0,1000,00,00,0,0);

    while(1){
        amsGetPos(1);
        amsCtrlSetInput(1, 2000);
        amsCtrlPIDUpdate(1, encPos[1].calibPOS);
        tiHSetDC(2, amsPID[1].output);
    }

    LED_GREEN = 0;
    LED_RED = 1;
    LED_YELLOW = 1;
    _LATG9 = 1;
    _LATC15 = 1;


    //_LATC15 = 0;
    //_LATG9 = 0;

    //_RC15 = 0;
    //_RG9 = 0;
    /*
    while(1)
    {
     delay_ms(1000);

    LED_RED = 1;
    _RC15 = 1;
    _RG9 = 1;
    //_LATC15 = 1;
    //_LATG9 = 1;
    delay_ms(1000);

    LED_RED = 0;
    _RC15 = 0;
    _RG9 = 0;
    //_LATC15 = 0;
    //_LATG9 = 0;

    }
   


    //testRadio();


/*
    LED_GREEN = 1;
    mpuSetup();
    LED_GREEN = 0;
    LED_RED = 1;
    LED_YELLOW = 1;

    wakeTime = 0;
    dcCounter = 0;

    WordVal src_addr_init = {RADIO_SRC_ADDR};
    WordVal src_pan_id_init = {RADIO_SRC_PAN_ID};
    WordVal dst_addr_init = {RADIO_DST_ADDR};


    //batSetup();

    //int old_ipl;
    //mSET_AND_SAVE_CPU_IP(old_ipl, 1)

    //swatchSetup();
    radioInit(src_addr_init, src_pan_id_init, RADIO_RXPQ_MAX_SIZE, RADIO_TXPQ_MAX_SIZE);
    radioSetChannel(RADIO_CHANNEL); //Set to my channel
    macSetDestAddr(dst_addr_init);

    //LED_YELLOW = 1;

    //dfmemSetup();
    //xlSetup();
    //gyroSetup();
    //tiHSetup();
    //mcSetup();
    //cmdSetup();
    //adcSetup();
    //telemSetup(); //Timer 5


    //mcSetDutyCycle(1,70.0);
    //mcSetDutyCycle(2,70.0);
    //mcSetDutyCycle(3,70.0);
    //mcSetDutyCycle(4,70.0);


#ifdef HALL_SENSORS
    //hallSetup();    // Timer 1, Timer 2
    //hallSteeringSetup(); //doesn't exist yet
#else //No hall sensors, standard BEMF control
    //legCtrlSetup(); // Timer 1
    //steeringSetup();  //Timer 5
#endif

    //tailCtrlSetup();

    //ovcamSetup();
    /*
    //radioReadTrxId(id);

    LED_RED = 1; //Red is use an "alive" indicator
    LED_GREEN = 0;
    LED_YELLOW = 0;


    //tiHSetFloat(1,50.0);
    //tiHSetFloat(2,75.0);

    //tiHSetup();

    //LED_GREEN = 1;

    //tiHSetFloat(1,.800);

    //LED_YELLOW = 1;
    //Radio startup verification
    //if(phyGetState() == 0x16)  { LED_GREEN = 1; }

    //Sleeping and low power options
    //_VREGS = 1;
    //gyroSleep();

    //tiHSetFloat(1,98.0);
    //tiHSetFloat(2,30.0);
    //tiHSetFloat(3,99.0);
    //tiHSetFloat(4,99.0);


    
    LED_GREEN = 0;
    LED_RED = 0;
    LED_YELLOW = 1;
    
    int i = 0;
    while (1)
    {
        delay_ms(2000);
        int i = i+1;
        LED_GREEN = i%2;
        
 //       cmdHandleRadioRxBuffer();


#ifndef __DEBUG //Idle will not work with debug
        //Simple idle:
        if (radioIsRxQueueEmpty()) {
            Idle();
            
            //_T1IE = 0;
        }
        
#endif

        //delay_ms(1000);
        //cmdEcho(0, 1 , (unsigned char*)(&i) );
        //i++;
        //if(radioIsRxQueueEmpty() && (t1_ticks >= wakeTime + 5000) ){
        //Idle();
        //LED_RED = 0;
        //gyroSleep();
        //Sleep();
        //}
    }
    
    
    /*
    if(g_radio_duty_cycle){
            if(dcCounter == 0){
                    //LED_GREEN = 1;
                    atSetRXAACKON();
            }else{
                    //LED_GREEN = 0;
                    atSetTRXOFF();
            }
    }
    else{
            //LED_GREEN = 1;
    }

    dcCounter = (dcCounter + 1) % 8;
		
    if(radioIsRxQueueEmpty() && !inMotion){
            //gyroSleep();
            LED_RED = 0;
            _SWDTEN = 1; //restart wdt
            Sleep();
            //Idle();
    }
		
    //should be asleep here, waiting for WTD wakeup
    ClrWdt(); //clear wdt
    _SWDTEN = 0; //software disable wdt
    LED_RED = 1;

    //spin up clock
    if(_COSC != 0b010){
            while(OSCCONbits.LOCK!=1);
    }
    //gyroWake();
    }
}


void testRadio(void)
{
    //designed to be used with testRadio.py.
    //test radio.py should produce many echoes that cycle through the ASCII characters
    //comment out all code in main and use test Radio to test the radio only.
    //This code does not initialize non-radio-nessisary components.
    wakeTime = 0;
    dcCounter = 0;

    WordVal src_addr_init = {RADIO_SRC_ADDR};
    WordVal src_pan_id_init = {RADIO_SRC_PAN_ID};
    WordVal dst_addr_init = {RADIO_DST_ADDR};

    SetupClock();
    SwitchClocks();
    SetupPorts();

    int old_ipl;
    mSET_AND_SAVE_CPU_IP(old_ipl, 1)

swatchSetup();
    radioInit(src_addr_init, src_pan_id_init, RADIO_RXPQ_MAX_SIZE, RADIO_TXPQ_MAX_SIZE);
    radioSetChannel(RADIO_CHANNEL); //Set to my channel
    macSetDestAddr(dst_addr_init);

    cmdSetup();
    LED_GREEN = ON;
    int i = 0;
    while (1) {
        i++;
        cmdHandleRadioRxBuffer();
    }
    LED_RED = OFF;
}*/
}