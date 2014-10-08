#include "pid-ip2.5.h"
#include "timer.h"
#include "mpu6000.h"
#include "steering.h"
#include "sclock.h"
//#include "dfmem_extra.h" replace with telemetry.c
#include "move_queue.h"
#include "utils.h" 
#include "telemetry.h"
#define TIMER_FREQUENCY     200                 // 400 Hz
#define TIMER_PERIOD        1/TIMER_FREQUENCY
#define SKIP_COUNT          2

long gyro_accum;
int gyroAvg;
int gdata[3];
int xldata[3];  // accelerometer data 
pidT steeringPID;
int steeringIsOn;

int telemSkip;

// structure to keep track of telemetry recording
extern TelemConStruct TelemControl;
extern int samplesToSave;

static int gyroWindow[GYRO_AVG_SAMPLES];
int windowIdx;

//extern int offsx, offsy, offsz;
int offsx, offsy, offsz;
extern mpuObj mpu_data; // gyro data structure

extern pidPos pidObjs[NUM_PIDS];
extern int bemf[NUM_PIDS];

extern moveCmdT currentMove;

extern unsigned long t1_ticks; //needed to calculate new runtimes

void steeringSetup(void) 
{   int i; long calib[3];
    gyro_accum = 0;
     setSampleSaveCount(0);   // turn off sampling by default unless enabled
    telemSkip = 1;  
	DisableIntT5; // make sure interrupt is off
//set up telemetry structure
	TelemControl.onoff = 0; 
	TelemControl.start = 0x0;
	TelemControl.count = 0x0;
	TelemControl.skip = 0x01;

// get gyro offset 
	calib[0] = 0; calib[1]=1; calib[2]=0;
	for( i =0; i < GYRO_AVG_SAMPLES; i++)
	{ mpuUpdate();
	   calib[0] += mpu_data.gyro_data[0];
 	   calib[1] += mpu_data.gyro_data[1];
	   calib[2] += mpu_data.gyro_data[2];
	} 
	offsx = (int)(calib[0]/GYRO_AVG_SAMPLES);
	offsy = (int)(calib[1]/GYRO_AVG_SAMPLES);
	offsz = (int)(calib[2]/GYRO_AVG_SAMPLES);

    initPIDObj(&steeringPID, STEERING_KP, STEERING_KI, STEERING_KD, STEERING_KAW, 0);
    setSteeringAngRate(0);

    // period value = Fcy/(prescale*Ftimer)  Fcy = 40 MHz
    unsigned int con_reg, period;
    // prescale 1:64
    con_reg = T5_ON & T5_IDLE_STOP & T5_GATE_OFF & T5_PS_1_64 & T5_SOURCE_INT;
    // Period is set so that period = 5ms (200Hz), MIPS = 40
    //period = 3125; // 200Hz
	period = 2083; // ~300Hz

    //con_reg = T3_ON & T3_IDLE_STOP & T3_GATE_OFF & T3_PS_1_64 & T3_SOURCE_INT;
    // Period is set so that period = 1.25ms (800Hz), MIPS = 40
    //period = 6250; // 100Hz

    OpenTimer5(con_reg, period);

    // interrupt for reading gyro
    ConfigIntTimer5(T5_INT_PRIOR_3 & T5_INT_ON);
//	DisableIntT5;   // DEBUG enabling interrupt seems to cause problem
    //offs = (float*)(gyroGetCalibParam());
	steeringIsOn = 1;
	windowIdx = 0;
}

/////  Main steering PID loop, gyro update, and telemetry saving
// note that flash and gyro share same SPI, thus only one can run at a time
// T5 int should be only place mpuUpdate is called, and only place writes to Flash occur
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) 
{
// unsigned long time_start, time_end; 
// 	time_start =  swatchTic(); 
    //int gyroAvg;
	int left, right;
	int i;
 
// this is only place where mpuUpdate can be called as it shares SPI2 with dfmem
  	mpuUpdate(); // read mpu6000 gyro + accelerometer

// for now just copy data from structure
	for(i = 0; i<3; i++)
	{ gdata[i] = mpu_data.gyro_data[i];
	   xldata[i] = mpu_data.xl_data[i];
	}

// get moving average of z axis
	gyroWindow[windowIdx] = gdata[2];
	windowIdx = (windowIdx + 1) % GYRO_AVG_SAMPLES;
	gyro_accum = 0;
	for( i =0; i < GYRO_AVG_SAMPLES; i++){
		gyro_accum += gyroWindow[i];
	}
	gyroAvg = (gyro_accum - GYRO_AVG_SAMPLES*offsz) / GYRO_AVG_SAMPLES;


	//Update the setpoints
	if((currentMove->inputL != 0) && (currentMove->inputR != 0)){  
		//Only update steering controller if we are in motion
		steeringPID.error = steeringPID.input - gyroAvg;
       	UpdatePIDSteering(&steeringPID , gyroAvg);

		left = currentMove->inputL;
		right = currentMove->inputR;
		
		// Depending on which way the bot is turning, choose which side to add correction to
		if( steeringPID.input <= 0){
			right = right - steeringPID.output;
			if( right < 0){ right = 1; }  //clip right channel to zero (one, actually)
		} else //if(steeringPID.input > 0)
		{
			left = left + steeringPID.output;
			if( left < 0){ left = 1; }  //clip right channel to zero (one, actually)
		}
		pidObjs[0].v_input = left;
		pidObjs[1].v_input = right;
	}

	// Section for saving telemetry data to flash
	if (TelemControl.onoff)
	{	if(t1_ticks >= TelemControl.start)
		{	if( telemSkip == 0)
			{	if(samplesToSave > 0)
				{samplesToSave--;
				// save current smaple in FLASH
				  telemSaveSample((unsigned long)(TelemControl.count - samplesToSave));
				  telemSkip = TelemControl.skip;  // reset skip
			  	 }
				else  {TelemControl.onoff = 0;}	// turn off telemetry if no more counts
			}   
			else  { telemSkip--;}  
             }         
       }     
//	time_end =  swatchToc();	
    _T5IF = 0;  // clear Interrupt flag so won't re-interrupt

}



void setSteeringAngRate(int angRate)
{  
	CRITICAL_SECTION_START  // don't want gains changing in middle of cycle
	steeringPID.input = angRate;
	steeringPID.p = 0;
	steeringPID.i = 0;
	steeringPID.d = 0;
	steeringPID.iState = 0;
	CRITICAL_SECTION_END
}

//I need a better solution than this
void UpdatePIDSteering(pidT *pid, int y)
{
    pid->p = (long)pid->Kp * pid->error;
    pid->i = (long)pid->Ki * pid->iState;
    //Filtered derivative action applied directly to measurement
    pid->d = ((long)pid->Kd * (long)pid->d * (long)STEERING_GAIN_SCALER) / ((long)pid->Kd + (long)pid->Kp * (long)pid->N) -
        ((long)pid->Kd * (long)pid->Kp * (long)pid->N * ((long)y - (long)pid->y_old)) /
        ((long)pid->Kd + (long)pid->Kp * (long)pid->N);

    pid->preSat = (pid->p + pid->i + pid->d) / (long)STEERING_GAIN_SCALER ;

    if (pid->preSat > SATVAL)
    {
        pid->output = SATVAL;
    }else
    {
        pid->output = pid->preSat;
    }

    pid->iState += (long)(pid->error) + ((long)(pid->Kaw) * 
			((long)(pid->output) - (long)(pid->preSat)))/(long)GAIN_SCALER;
    pid->y_old = y;
}

void steeringSetGains(int Kp, int Ki, int Kd, int Kaw, int ff){
    steeringPID.Kp  = Kp;
    steeringPID.Ki  = Ki;
    steeringPID.Kd  = Kd;
    steeringPID.Kaw = Kaw;
	steeringPID.feedforward = ff;
}

/*
void getSteeringTelem(unsigned char* ptr){
	steeringPID.p;
	
}
*/
