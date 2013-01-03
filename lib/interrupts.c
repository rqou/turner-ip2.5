/*
 * Interrupt handlers. All tests are handled through timer 2 interrupt.
 * Timer 2 runs at 100Hz and checks to see if a command packet has
 * been received. If so, it pushes the appropriate test routine onto
 * a function queue to be popped and executed in the main loop.
 *
 * Created: 2011-04-16
 * Author: AMH
 */
#include "interrupts.h"
#include "consts.h"
#include "tests.h"
#include "timer.h"
#include "radio.h"
#include <stdlib.h>
#include "led.h"

unsigned char* rxPacketData;
unsigned char type, status, length;

volatile unsigned int t2_ticks;

volatile PayQueue rx_pay_queue;

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void) {

    _INT0IF = 0;    // Clear the interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {

    _T1IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {
    MacPacket rx_packet;
    Payload rx_payload;

    if (!radioRxQueueEmpty())
    {
        // Check for unprocessed packet
        rx_packet = radioDequeueRxPacket();
        if(rx_packet == NULL) return;

        // Retrieve payload
        rx_payload = macGetPayload(rx_packet);

        // Switch on packet type
        Test* test = (Test*) malloc(sizeof(Test));
        if(!test) return;

        test->packet = rx_packet;
        switch(payGetType(rx_payload))
        {
            case RADIO_TEST:
                test->tf = &test_radio;
                queuePush(fun_queue, test);
                break;
            case GYRO_TEST:
                test->tf = &test_gyro;
                queuePush(fun_queue, test);
                break;
		case HALL_TEST:
 			test->tf = &test_hall;
                queuePush(fun_queue, test);
                break;
            case ACCEL_TEST:
                test->tf = &test_accel;
                queuePush(fun_queue, test);
                break;
            case DFLASH_TEST:
                test->tf = &test_dflash;
                queuePush(fun_queue, test);
                break;
            case MOTOR_TEST:
                test->tf = &test_motor;
                queuePush(fun_queue, test);
                break;
            case SMA_TEST:
                test->tf = &test_sma;
                queuePush(fun_queue, test);
                break;
            default:    // temporary to check out what is happening to packets
		     test->tf = &test_radio;
                queuePush(fun_queue, test);
                break;
        }
    }
    _T2IF = 0;
}

void setupTimer6(unsigned int fs) {

    unsigned int con_reg, period;

    con_reg =   T6_ON &         // Timer on
                T6_IDLE_STOP &  // Stop timer when idle
                T6_GATE_OFF &   // Gated mode off
                T6_PS_1_8 &     // Prescale 1:8
                T6_SOURCE_INT &  // Internal clock source
                T6_32BIT_MODE_OFF; // 16 bit mode

    // period value = Fcy/(prescale*Ftimer)
    period = FCY/(8*fs);

    OpenTimer6(con_reg, period);
    ConfigIntTimer6(T6_INT_PRIOR_3 & T6_INT_ON);

}

void __attribute__((interrupt, no_auto_psv)) _T6Interrupt(void) {
    radioProcess();
    _T6IF = 0;
}
