/* basic motor control functions */

#include "cmd.h"
#include "cmd_const.h"
#include "cmd-motor.h"
#include "utils.h"
#include "pid-ip2.5.h"
#include "radio.h"
#include "../MyConsts/radio_settings.h"
#include "tiH.h"
#include "timer.h"
#include "telemetry.h"
#include "sclock.h"
#include "blink.h"

extern pidPos pidObjs[NUM_PIDS];

// set PWM values for short duration for each motor
// throttle[0], throttle[1], duration
void cmdSetThrust(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
 {	int thrust1 = frame[0] + (frame[1] << 8);
	int thrust2 = frame[2] + (frame[3] << 8);
	unsigned int run_time_ms = frame[4] + (frame[5] << 8);

	DisableIntT1;	// since PID interrupt overwrites PWM values
// wiring was scrambled Jan. 9, 2013 on RSF velociRoACH robot
// unscramble Feb. 6, 2013
  	tiHSetDC(1, thrust1);
	tiHSetDC(2, thrust2); 
	delay_ms(run_time_ms);
	tiHSetDC(1,0);
	tiHSetDC(2,0);
	EnableIntT1;
 }  


// 	stop motors and PD loop
void cmdEStop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
{ 	CRITICAL_SECTION_START; // Disable interrupts
	EmergencyStop(); // stop pid loop
	CRITICAL_SECTION_END; // Re-enable interrupts
	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_ESTOP, status, length, frame);  
    	return; //success     
}


#define MESSAGE "Emergency Stop - low Battery!"
void cmdEStopSend(void)
{	 unsigned char string_length; 
	unsigned char message_string[sizeof(MESSAGE)] = MESSAGE;
	unsigned char status = 1;
	CRITICAL_SECTION_START; // Disable interrupts
	EmergencyStop(); // stop pid loop
	 CRITICAL_SECTION_END; // Re-enable interrupts
//	message_string = MESSAGE;
	string_length = sizeof(MESSAGE);
// note packet won't be sent until return from battery interrupt
	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_ESTOP, status, string_length, message_string);  

	blink_leds(30,150); // also alert that we have had a battery brown out
	return;
}


// report motor position and  reset motor position (from Hall angular sensors)
// note motor_count is long (4 bytes) revs+frac rev
void cmdZeroPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) 
{ 	long motor_count[2]; 
	motor_count[0] = pidObjs[0].p_state;
	motor_count[1] = pidObjs[1].p_state;

	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_ZERO_POS,\
		 status, sizeof(motor_count), (unsigned char *)motor_count);  
     pidZeroPos(0); pidZeroPos(1);
}

void cmdSetThrustClosedLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame)
{	int thrust1 = frame[0] + (frame[1] << 8);
	unsigned int run_time_ms1 = frame[2] + (frame[3] << 8);
	int thrust2 = frame[4] + (frame[5] << 8);
	unsigned int run_time_ms2 = frame[6] + (frame[7] << 8);
	//currentMove = manualMove;
	pidSetInput(0 ,thrust1, run_time_ms1);
	pidOn(0);
	pidSetInput(1 ,thrust2, run_time_ms2);
	pidOn(1);
}

void cmdSetPIDGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	int Kp, Ki, Kd, Kaw, ff;
	int idx = 0;

	Kp = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Ki = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kd = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kaw = frame[idx] + (frame[idx+1] << 8); idx+=2;
	ff = frame[idx] + (frame[idx+1] << 8); idx+=2;
	pidSetGains(0,Kp,Ki,Kd,Kaw, ff);
	Kp = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Ki = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kd = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kaw = frame[idx] + (frame[idx+1] << 8); idx+=2;
	ff = frame[idx] + (frame[idx+1] << 8); idx+=2;
	pidSetGains(1,Kp,Ki,Kd,Kaw, ff);

	//Send confirmation packet
	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_SET_PID_GAINS, status, 20, frame);  
      return; //success
}


// set up velocity profile structure  - assume 4 set points for now, generalize later
// packet = motor0: intervals[0..3], delta[0...3], vel[0...3], motor1: intervals[0..3], delta[0...3], vel[0...3]
void cmdSetVelProfile(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	int interval[NUM_VELS], delta[NUM_VELS], vel[NUM_VELS];
	int idx = 0, i = 0;
	for(i = 0; i < NUM_VELS; i ++)
	{ interval[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2;	}
	for(i = 0; i < NUM_VELS; i ++)
	{ delta[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2; 	}
	for(i = 0; i < NUM_VELS; i ++)
	{ vel[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2; 	}
	setPIDVelProfile(0, interval, delta, vel);
	for(i = 0; i < NUM_VELS; i ++)
	{ interval[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2;	}
	for(i = 0; i < NUM_VELS; i ++)
	{ delta[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2; 	}
	for(i = 0; i < NUM_VELS; i ++)
	{ vel[i] = frame[idx]+ (frame[idx+1]<<8);
	 	idx+=2; 	}
	setPIDVelProfile(1, interval, delta, vel);

	//Send confirmation packet
	radioConfirmationPacket(RADIO_DEST_ADDR, CMD_SET_VEL_PROFILE, status, 48, frame);  
     return; //success
}


