/*
 * Copyright (c) 2011, Franklin W. Olin College of Engineering
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the University of California, Berkeley nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Series of test functions to test the functionality of the
 * ImageProc/CrawlerProc class of microcontroller-based robotics boards.
 *
 * by Aaron M. Hoover
 *
 * v.0.1
 *
 * Revisions:
 *  Aaron M. Hoover      2011-05-02    Initial release
 *
 * Notes:
 *  A basic set of tests for testing all the functionality of the board.
 *  Every test function must take four arguments - char type, unsigned char
 *  status, unsigned char length, and unsigned char* data. This is because
 *  pointers to these functions are stored in the function pointer queue
 *  that is the primary queue that's serviced by the main loop.
 *  For simplicity, if each function is called with the same type and number of
 *  arguments, we can always use the same call structure in the main loop.
 */

#include "tests.h"
#include "consts.h"
#include "utils.h"
#include "radio.h"
#include "pwm.h"
//#include "mpu6000.h"
#include "xl.h"
#include "dfmem.h"
#include <string.h>
#include <stdlib.h>
#include "radio_settings.h"
#include "ams-enc.h"
#include "sclock.h"
#include "tih.h"

volatile Queue fun_queue;
extern mpuObj mpu_data;


/*****************************************************************************
* Function Name : test_radio
* Description   : Send out a packet containing the data in the array pointed to
*                 by the 'data' argument passed in.
* Parameters    : type - The type field of the radio test packet
*                 status - Status field of radio test packet (not yet used)
*                 length - The length of the payload data array
*                 data - Pointer to the character array containing the payload
*                 data to send back
* Return Value  : success indicator - 0 for failed, 1 for succeeded
*****************************************************************************/
unsigned char test_radio(unsigned char type, unsigned char status,\
                         unsigned char length, unsigned char* data)
{
    MacPacket packet;
    Payload pld;

    // Get a new packet from the pool
    packet = radioRequestPacket(length);
    if(packet == NULL) return 0;
    macSetDestAddr(packet, RADIO_DEST_ADDR);

    // Prepare the payload
    pld = packet->payload;
    paySetType(pld, type);
    paySetStatus(pld, status);
    paySetData(pld, length, data);

    // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));

    return 1; //success
}

/*****************************************************************************
* Function Name : test_gyro
* Description : Create and send out over the radio a number of test packets that
* contain the three X,Y, and Z values read from the gyro.
* Parameters : type - The type field of the gyro test packet
* status - Status field of gyro test packet (not yet used)
* length - The length of the payload data array
* data - not used
* Return Value : success indicator - 0 for failed, 1 for succeeded
*****************************************************************************/
unsigned char test_gyro(unsigned char type, unsigned char status,\
                         unsigned char length, unsigned char* data)
{  	MacPacket packet;
     Payload pld;
	// refresh MPU reading
	mpuUpdate();

    // Get a new packet from the pool
    	packet = radioRequestPacket(sizeof(mpu_data));
    	if(packet == NULL) return 0;
    	macSetDestAddr(packet, RADIO_DEST_ADDR);

     // Prepare the payload
     	pld = packet->payload;
     	paySetStatus(pld, STATUS_UNUSED);
     	paySetType(pld, type);
 
	// Read gyro data into the payload
	memcpy(payGetData(pld), & mpu_data, sizeof(mpu_data)); // copy gyro data to packet

   	// Enqueue the packet for broadcast
  	while(!radioEnqueueTxPacket(packet));
      return 1; //success
}

/*****************************************************************************
* Function Name : test_hall
* Description   : send out over the radio a the current position readings from the
				Austria Microsystems AS5048B absolute Hall sensors
* Parameters    : type - The type field of the hall test packet
*                 status - Status field of Hall test packet (not yet used)
*                 length - The length of the payload data array
*                 data - not used
* Return Value  : success indicator - 0 for failed, 1 for succeeded
*****************************************************************************/
unsigned char test_hall(unsigned char type, unsigned char status,\
                         unsigned char length, unsigned char* data)
{   int i;
	MacPacket packet;
    Payload pld;
// refresh Hall reading
	for(i = 0; i< NUM_ENC; i++)
	{ amsGetPos(i); }

    // Get a new packet from the pool
    packet = radioRequestPacket(sizeof(encPos));
    if(packet == NULL) return 0;
    macSetDestAddr(packet, RADIO_DEST_ADDR);

   // Prepare the payload
    pld = packet->payload;
    paySetStatus(pld, STATUS_UNUSED);
    paySetType(pld, type);
 
// Read Hall data into the payload
	memcpy(payGetData(pld),  & encPos, sizeof(encPos)); // copy Hall data to packet

   // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));
  
    return 1; //success
}



/*****************************************************************************
* Function Name : test_accel
* Description   : Create and send out over the radio a number of test packets that
*                 contain the three X,Y, and Z values read from the
*                 accelerometer.
* Parameters    : type - The type field of the accelerometer test packet
*                 status - Status field of the accelerometer test packet (not yet used)
*                 length - The length of the payload data array
*                 data - not used
* Return Value  : success indicator - 0 for failed, 1 for succeeded
*****************************************************************************/
unsigned char test_accel(unsigned char type, unsigned char status,\
                         unsigned char length, unsigned char* data)
{    /*
    MacPacket packet;
    Payload pld;
    for(int i=0; i < data[0]; i++) {
        // Get a new packet from the pool
        packet = radioRequestPacket(6);
        if(packet == NULL) return;
        macSetDestAddr(packet, RADIO_DEST_ADDR);
        // Toggle LED
        LED_1 = ~LED_1;
        // Fill the payload
        pld = packet->payload;
        paySetType(pld, type);
        paySetStatus(pld, 0);
        paySetData(pld, 6, xlReadXYZ());
        // Enqueue the packet for broadcast
        while(!radioEnqueueTxPacket(packet));
        // Wait around for a while
        delay_ms(TEST_PACKET_INTERVAL_MS);
    }
    LED_1 = OFF;
    */
    return 1; //success
}

/*****************************************************************************
* Function Name : test_dflash
* Description   : Write four different strings to a page in the data flash,
*                 then read them back and send their contents out over the
*                 radio. Bonus points if you can identify the film without
*                 reverting to the internet.
* Parameters    : type - The type field of the dflash test packet
*                 status - Status field of the dflash test packet (not yet used)
*                 length - The length of the payload data array
*                 data - not used
* Return Value  : success indicator - 0 for failed, 1 for succeeded
*****************************************************************************/
unsigned char test_dflash(unsigned char type, unsigned char status,
                          unsigned char length, unsigned char* data)
{
    MacPacket packet;
    Payload pld;

    char mem_data[256] = {};
    
    //char* str1 = "You must be here to fix the cable.";  // 38+1
    char str1[] = "D";  // 38+1
    char str2[] = "Lord. You can imagine where it goes from here.";  //46+1
    char str3[] = "He fixes the cable?"; //19+1
    char str4[] = "Don't be fatuous, Jeffrey."; //26+1
    int  page  = 0x100;

    strcpy(mem_data, str1);
    strcpy(mem_data + strlen(str1), str2);
    strcpy(mem_data + strlen(str1) + strlen(str2), str3);
    strcpy(mem_data + strlen(str1) + strlen(str2) + strlen(str3), str4);

    // Write into dfmem
    dfmemWrite((unsigned char *)(mem_data), sizeof(mem_data), page, 0, 1);

    // ---------- string 1 -----------------------------------------------------
    // Get a new packet from the pool
    packet = radioRequestPacket(strlen(str1));
    if(packet == NULL) return 0;
    macSetDestAddr(packet, RADIO_DEST_ADDR);

    // Prepare the payload
    pld = packet->payload;
    paySetStatus(pld, STATUS_UNUSED);
    paySetType(pld, type);

    // Read out dfmem into the payload
    dfmemRead(page, 0, strlen(str1), payGetData(pld));

    // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));

    // ---------- string 2 -----------------------------------------------------
    // Get a new packet from the pool
    packet = radioRequestPacket(strlen(str2));
    if(packet == NULL) return 0;
    macSetDestAddr(packet, RADIO_DEST_ADDR);

    // Prepare the payload
    pld = packet->payload;
    paySetStatus(pld, STATUS_UNUSED);
    paySetType(pld, type);

    // Read out dfmem into the payload
    dfmemRead(page, strlen(str1), strlen(str2), payGetData(pld));

    // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));

    // ---------- string 3 -----------------------------------------------------
    // Get a new packet from the pool
    packet = radioRequestPacket(strlen(str3));
    if(packet == NULL) return 0;
    macSetDestAddr(packet, RADIO_DEST_ADDR);

    // Prepare the payload
    pld = packet->payload;
    paySetStatus(pld, STATUS_UNUSED);
    paySetType(pld, type);

    // Read out dfmem into the payload
    dfmemRead(page, strlen(str1) + strlen(str2), strlen(str3),
            payGetData(pld));

    // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));

    // ---------- string 4 -----------------------------------------------------
    // Get a new packet from the pool
    packet = radioRequestPacket(strlen(str4));
    if(packet == NULL) return 0;
    macSetDestAddr(packet, RADIO_DEST_ADDR);

    // Prepare the payload
    pld = packet->payload;
    paySetStatus(pld, STATUS_UNUSED);
    paySetType(pld, type);

    // Read out dfmem into the payload
    dfmemRead(page, strlen(str1) + strlen(str2) + strlen(str3), strlen(str4),
            payGetData(pld));

    // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));

    return 1; //success
}

/*****************************************************************************
* Function Name : test_motor
* Description   : Turns on a specified motor for a specified period of time
*                 and duty cycle
* Parameters    : type - The type field of the motor test packet
*                 status - Status field of the motor test packet (not yet used)
*                 length - The length of the payload data array
*                 data - data[0] = motor number
*                        data[1:2] = on time (milli secs)
*                        data[3:4] = duty cycle (percent)
* Return Value  : success indicator - 0 for failed, 1 for succeeded
*****************************************************************************/
unsigned char test_motor(unsigned char type, unsigned char status, \
                          unsigned char length, unsigned char* data)
{ unsigned int motor_id; int on_time; int dutycycle;
	char ack_string[40]="motor OK\n";
	 MacPacket packet;
   	 Payload pld;

	 motor_id = (unsigned int) data[0];
	 on_time = (unsigned long)( (data[3] << 8) + data[2]);
	 dutycycle = (int)((data[5] << 8) + data[4]);
	 tiHSetDC(motor_id, dutycycle);
	swatchDelayMs(on_time);
  	tiHSetDC(motor_id, 0);
// send an ack packet back - could have data later...
  // Get a new packet from the pool
    packet = radioRequestPacket(sizeof(ack_string));
    if(packet == NULL) return 0;
    macSetDestAddr(packet, RADIO_DEST_ADDR);

   // Prepare the payload
    pld = packet->payload;
    paySetStatus(pld, STATUS_UNUSED);
    paySetType(pld, type);
 // Read message data into the payload
	memcpy(payGetData(pld),  & ack_string, sizeof(ack_string)); // copy ack_string to packet

   // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));
  
      return 1; //success
} 



/**********************************/
unsigned char test_sma(unsigned char type, unsigned char status, \
                          unsigned char length, unsigned char* data)
{
    /*
    WordVal dest_addr;
    dest_addr = radioGetDestAddr();

    unsigned char chan_id, on_time, duty_cycle;
    chan_id = data[0];
    on_time = data[1];
    duty_cycle = data[2];


    if(chan_id == 1)
    {
        P1OVDCONbits.POVD3H = 1;
        P1OVDCONbits.POVD3L = 0;
        P1OVDCONbits.POUT3L = 0;
    }else
    {
        P1OVDCONbits.POVD3L = 1;
        P1OVDCONbits.POVD3H = 0;
        P1OVDCONbits.POUT3H = 0;
    }

    SetDCMCPWM(3, (2 * (long)PTPERvalue * (long)duty_cycle)/100, 0);
    if (chan_id == SMA_1)
    {
        MD_LED_1 = 1;
    } else if (chan_id == SMA_2)
    {
        MD_LED_2 = 1;
    }

    int i;
    for (i=0; i < on_time; i++)
    {
        delay_ms(1000);
    }

    SetDCMCPWM(3, 0, 0);
    MD_LED_1 = 0;
    MD_LED_2 = 0;

    */
    return 1;
}

/*
 * This version is for controlling the Freescale motor controller. The aim is
 * to phase the controller out for the Toshiba TB6612FNG.
 */
/*
unsigned char set_motor_direction(unsigned char chan_num, unsigned char\
                            direction)
{
    switch(chan_num){
        case 1:
            //Braking case: override both and set both to low
            if (direction == BRAKE){
                P1OVDCONbits.POVD1L = 0;
                P1OVDCONbits.POUT1L = 0;
                P1OVDCONbits.POVD1H = 0;
                P1OVDCONbits.POUT1H = 0;
            //Reverse case: set 1L to PWM, override 1H and set high (to enable high impedance during off times)
            }else if (direction == REVERSE){
                P1OVDCONbits.POVD1L = 1;
                P1OVDCONbits.POVD1H = 0;
                P1OVDCONbits.POUT1H = 1;
            //Forward case: set 1H to PWM, override 1L and set high
            }else if(direction == FORWARD){
                P1OVDCONbits.POVD1L = 0;
                P1OVDCONbits.POVD1H = 1;
                P1OVDCONbits.POUT1L = 1;
            }
           break;
        case 2:
            //Braking case: override both and set both to low
            if (direction == BRAKE){
                P1OVDCONbits.POVD2L = 0;
                P1OVDCONbits.POUT2L = 0;
                P1OVDCONbits.POVD2H = 0;
                P1OVDCONbits.POUT2H = 0;
            //Reverse case: set 1L to PWM, override 1H and set high
            }else if (direction == REVERSE){
                P1OVDCONbits.POVD2L = 1;
                P1OVDCONbits.POVD2H = 0;
                P1OVDCONbits.POUT2H = 1;
            //Forward case: set 1H to PWM, override 1L and set low
            }else if (direction == FORWARD){
                P1OVDCONbits.POVD2L = 0;
                P1OVDCONbits.POVD2H = 1;
                P1OVDCONbits.POUT2L = 1;
            }
            break;
        default:
            return 0;
    }
    return 1;
}
*/


/*
 * This version is for the TB6612 hardware which has a very different
 * configuration for controlling the motor direction.
 *
 * Forward = CW: PWM1H = High, PWM1L = Low.
 * Reverse = CCW: PWM1H = Low, PWM1L = High.
 */
/*
unsigned char set_motor_direction(unsigned char chan_num, unsigned char\
                            direction)
{
    switch(chan_num){
        case 1:
            //Reverse case: set 1L to PWM, override 1H and set  (to enable high impedance during off times)
            if (direction == REVERSE){
                P1OVDCONbits.POVD1L = 1;
                P1OVDCONbits.POVD1H = 0;
                P1OVDCONbits.POUT1H = 0;
            //Forward case: set 1H to PWM, override 1L and set high
            }else if(direction == FORWARD){
                P1OVDCONbits.POVD1L = 0;
                P1OVDCONbits.POVD1H = 1;
                P1OVDCONbits.POUT1L = 0;
            }
           break;
        case 2:
            //Braking case: override both and set both to low
            if (direction == BRAKE){
                P1OVDCONbits.POVD2L = 0;
                P1OVDCONbits.POUT2L = 0;
                P1OVDCONbits.POVD2H = 0;
                P1OVDCONbits.POUT2H = 0;
            //Reverse case: set 2L to PWM, override 2H and set high
            }else if (direction == REVERSE){
                P1OVDCONbits.POVD2L = 1;
                P1OVDCONbits.POVD2H = 0;
                P1OVDCONbits.POUT2H = 1;
            //Forward case: set 2H to PWM, override 2L and set low
            }else if (direction == FORWARD){
                P1OVDCONbits.POVD2L = 0;
                P1OVDCONbits.POVD2H = 1;
                P1OVDCONbits.POUT2L = 1;
            }
            break;
        default:
            return 0;
    }
    return 1;
}
*/


