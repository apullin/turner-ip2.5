/*********************************************************************************************************
 * Name: main.c
 * Desc: A test suite for the ImageProc 2.2 system. These tests should not be
 * considered rigorous, exhaustive tests of the hardware, but rather
 * "smoke tests" - ie. turn on the functionality and make sure the
 * hardware/software doesn't start "smoking."
 *
 * The architecture is based on a function pointer queue scheduling model. The
 * meat of the testing logic resides in test.c. If the radio has received a
 * command packet during the previous timer interval for Timer2, the appropriate
 * function pointer is added to a queue in the interrupt service routine for
 * Timer2 (interrupts.c). The main loop simply pops the function pointer off
 * the top of the queue and executes it.
 *
 * Date: 2011-04-13
 * Author: AMH, Ryan Julian
 *********************************************************************************************************/
#include <xc.h>
#include "init_default.h"
#include "timer.h"
#include "utils.h"
#include "queue.h"
#include "radio.h"
#include "tests.h" 
#include "dfmem.h"
#include "interrupts.h"
#include "sclock.h"
#include "ams-enc.h"
#include "tih.h"
#include "blink.h"
#include <stdlib.h>
#include "cmd.h"
#include "pid-ip2.5.h"
#include "steering.h"
#include "consts.h"
#include "adc_pid.h"
#include "init.h"
#include "settings.h"

Payload rx_payload;
MacPacket rx_packet;

volatile MacPacket uart_tx_packet;
volatile unsigned char uart_tx_flag;

Test* test;


unsigned int error_code;

int main() {
    fun_queue = queueInit(FUN_Q_LEN);
    test_function tf;
    error_code = ERR_NONE;

    /* Initialization */
    SetupClock();
    SwitchClocks();
    SetupPorts();

    SetupInterrupts();
    //   SetupADC(); old A/D
    adcSetup(); // DMA A/D
    //    SetupTimer1(); setup in pidSetup
    SetupTimer2();
    sclockSetup();
    mpuSetup();
    /*   if Hall not present will hang */
#ifdef HALL_SENSOR
    amsEncoderSetup();
#endif
    dfmemSetup();
    tiHSetup(); // set up H bridge drivers
    cmdSetup(); // setup command table
    pidSetup(); // setup PID control

    // Radio setup
    blink_leds(4, 200); // blink LEDs 4 times at half sec
    radioInit(RADIO_RXPQ_MAX_SIZE, RADIO_TXPQ_MAX_SIZE);
    radioSetChannel(RADIO_CHANNEL);
    radioSetSrcAddr(RADIO_SRC_ADDR);
    radioSetSrcPanID(RADIO_PAN_ID);
    setupTimer6(RADIO_FCY); // Radio and buffer loop timer
    /**** set up steering last - so dfmem can finish ****/
    steeringSetup(); // steering and Timer5 Int
    blink_leds(4, 500); // blink LEDs 4 times at half sec
    char j;
    for (j = 0; j < 3; j++) {
        LED_2 = ON;
        delay_ms(250);
        LED_2 = OFF;
        delay_ms(250);
    }

    LED_2 = ON;

    EnableIntT2;
    while (1) {
        while (!queueIsEmpty(fun_queue)) {
            test = queuePop(fun_queue);
            rx_payload = macGetPayload(test->packet);
            tf = test->tf;
            (*tf)(payGetType(rx_payload), // old commands don't use packet type
                    payGetStatus(rx_payload),
                    payGetDataLength(rx_payload),
                    payGetData(rx_payload));
            radioReturnPacket(test->packet);
            free(test);
        }
    }
    return 0;
}
