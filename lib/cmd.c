/***************************************************************************
* Name: cmd.c
* Desc: Receiving and transmitting queue handler
* Date: 2010-07-10
* Author: stanbaek
**************************************************************************/
#include "utils.h"
#include "cmd.h"
#include "cmd_const.h"
#include "cmd-motor.h"
#include "cmd-aux.h"
#include "dfmem.h"
#include "utils.h"
#include "ports.h"
#include "sclock.h"
#include "blink.h"
#include "payload.h"
#include "mac_packet.h"
#include "dfmem.h"
#include "pid-ip2.5.h"
#include "radio.h"
#include "move_queue.h"
#include "steering.h"
#include "dfmem.h"
//  #include "dfmem_extra.h" replace with telemetry.h
#include "tests.h"
#include "queue.h"
#include "version.h"
#include "tiH.h"
#include "timer.h"
#include "telemetry.h"
#include "battery.h"
#include "settings.h"


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

unsigned char tx_frame_[127];

extern MoveQueue moveq;
extern int offsz;
extern pidPos pidObjs[NUM_PIDS];
extern telemU telemPIDdata;
extern TelemConStruct TelemControl;
extern unsigned long t1_ticks;
extern int samplesToSave;
unsigned long packetNum = 0; // sequential packet number

extern moveCmdT currentMove, idleMove, manualMove;
// updated version string to identify robot
//extern static char version[];

// use an array of function pointer to avoid a number of case statements
// MAX_CMD_FUNC_SIZE is defined in cmd_const.h
// arg to all commands are: type, status, length, *frame
void (*cmd_func[MAX_CMD_FUNC_SIZE])(unsigned char, unsigned char, unsigned char, unsigned char*);

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/

static void cmdSteer(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdGetImuData(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdGetImuLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdStartImuDataSave(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdStopImuDataSave(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdTxSavedImuData(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdEraseMemSector(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);



//User commands
static void cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdGetPIDTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSetCtrldTurnRate(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdGetImuLoopZGyro(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSetMoveQueue(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSetSteeringGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSpecialTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdEraseSector(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdFlashReadback(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdStartTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);

//auxiliary commands defined in cmd-aux.c
/*
void cmdEcho(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdSoftwareReset(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
static void cmdWhoAmI(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
void cmdError(void);
*/

// commands defined in cmd-motor.c:
//static void cmdSetThrust(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
//static void cmdSetThrustClosedLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
//static void cmdSetPIDGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
//static void cmdSetVelProfile(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);
//static void cmdZeroPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame);


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
	cmd_func[CMD_ESTOP] = &cmdEStop;
// setup battery low voltage callback
	 batSetCallback(&cmdEStopSend);
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

/*-----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * The functions below are intended for internal use, i.e., private methods.
 * Users are recommended to use functions defined above.
 * ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
 *         User function
-----------------------------------------------------------------------------*/
/* keyboard_telem2.5.py uses , 
 * cmd.c: FLASH_READBACK, ERASE_SECTORS, START_TELEM, GET_PID_TELEMETRY
* cmd-motor.c: SET_PID_GAINS,  SET_THRUST, SET_VEL_PROFILE, ZERO_POS, SET_THRUST_CLOSED_LOOP
*  cmd-aux.c: WHO_AM_I, ECHO, SOFTWARE_RESET
*/


static void cmdEraseSector(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
{
//	eraseDFMemSectors0a0b();
/// updated for IP2.5
// hard code for now 300 samples at 300 Hz
    //TODO: This entire command needs to be overhauled (pullin, ronf)

//	CRITICAL_SECTION_START   //  can't have interrupt process grabbing SPI2
//	dfmemEraseSectorsForSamples( (unsigned long) 300, sizeof(telemStruct_t));
//	CRITICAL_SECTION_END
//	mpuBeginUpdate(); // make sure we can still use SPI2
}


// telemetry is saved at 300 Hz inside steering servo
// alternative telemetry which runs at 1 kHz rate inside PID loop
static void cmdStartTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	int idx=0;
       unsigned long temp;
     TelemControl.count = frame[idx] + (frame[idx+1] << 8); idx+=2;
   // start time is relative to current t1_ticks
       CRITICAL_SECTION_START
	temp = t1_ticks; // need atomic read due to interrupts
	CRITICAL_SECTION_END
	TelemControl.start = 
		(unsigned long) (frame[idx] + (frame[idx+1] << 8) )
						+ temp;
	idx+=2;
      samplesToSave = TelemControl.count; // **** this runs sample capture in T5 interrupt
	TelemControl.skip = frame[idx]+(frame[idx+1]<<8); 
//	swatchReset(); //TODO: swatch module no longer exists.
	if(TelemControl.count > 0) 
	{ TelemControl.onoff = 1;   // use just steering servo sample capture
	 } // enable telemetry last 
}

//send single radio packet with current PID state
static void cmdGetPIDTelemetry(unsigned char type, unsigned char status, unsigned char length,
								 unsigned char *frame)
{ 	unsigned int sampLen = sizeof(telemStruct_t);
	telemGetPID(packetNum);  // get current state
	radioConfirmationPacket(RADIO_DST_ADDR, CMD_SPECIAL_TELEMETRY,
						     status, sampLen, (unsigned char *) &telemPIDdata);  
	packetNum++;
    // delay_ms(25);	// slow down for XBee 57.6 K
	blink_leds(1,20); // wait 20 ms to give plenty of time to send packets
}


static void cmdFlashReadback(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
{	unsigned int count;
	count = frame[0] + (frame[1] << 8);
	telemFlashReadback(count);	
/**********  will need to disable mpuUpdate to read flash... ******/
//	readDFMemBySample(count);
}



//#include "cmd-motor.c"  // ZeroPos, SetThrust, SetVelProfile, SetPIDGains
// #include "cmd-aux.c"   // auxiliary functions Echo, WhoAmI, Error
#include "cmd-pt2.c"
