/* Telemetry related functions */
/* since telemetry is called in steering.c */
#include "pid-ip2.5.h"
#include "utils.h"
#include "telemetry.h"
#include "sclock.h"
#include "dfmem.h"
#include "radio.h"
#include "../MyConsts/radio_settings.h"
#include "timer.h"
#include "cmd.h"
#include "adc_pid.h"
#include "spi_controller.h"
#include "blink.h"

int samplesToSave;
extern int gdata[3];
extern int gyroAvg;
extern int xldata[3];  // accelerometer data 
extern int offsx, offsy, offsz;
extern SpicStatus port_status[SPIC_NUM_PORTS];  

// structure to keep track of telemetry recording
TelemConStruct TelemControl;  // structure for telemetry control
extern pidT steeringPID;
extern pidPos pidObjs[NUM_PIDS];
extern int bemf[NUM_PIDS];

/* static struct piddata {
    int output[NUM_PIDS];
    unsigned int measurements[NUM_PIDS];
	long p[2],i[2],d[2];
} PIDTelemData;
*/

/* data structure for telemetry */
telemU telemPIDdata;


// store current PID info into structure. Used by telemSaveSample and CmdGetPIDTelemetry
void telemGetPID(unsigned long sampIdx)
{
	telemPIDdata.telemStruct.sampleIndex = sampIdx;
//Stopwatch was already started in the cmdSpecialTelemetry function
	telemPIDdata.telemStruct.timeStamp = (long)swatchTic(); 

// since T1 has higher priority, these state readings might get interrupted 
	CRITICAL_SECTION_START  // need coherent sample without T1 int updates
//  save Hall encoder position instead of commanded thrust
		telemPIDdata.telemStruct.posL = pidObjs[0].p_state;
		telemPIDdata.telemStruct.posR = pidObjs[1].p_state;
	// save output instead of reading PWM (sync issue?)
		telemPIDdata.telemStruct.dcL = pidObjs[0].output;	// left
		telemPIDdata.telemStruct.dcR = pidObjs[1].output;	// right
		telemPIDdata.telemStruct.bemfL = bemf[0];
		telemPIDdata.telemStruct.bemfR = bemf[1];
	CRITICAL_SECTION_END

   		telemPIDdata.telemStruct.gyroX = gdata[0] - offsx;
		telemPIDdata.telemStruct.gyroY = gdata[1] - offsy;
		telemPIDdata.telemStruct.gyroZ = gdata[2] - offsz; 
		telemPIDdata.telemStruct.gyroAvg = gyroAvg;
		telemPIDdata.telemStruct.accelX = xldata[0];
		telemPIDdata.telemStruct.accelY = xldata[1];
		telemPIDdata.telemStruct.accelZ = xldata[2];
		telemPIDdata.telemStruct.Vbatt = (int) adcGetVbatt();
		telemPIDdata.telemStruct.sOut = steeringPID.output;
		return;
}



// record current state to telemU structure
void telemSaveSample(unsigned long sampIdx)
{	
/* telemU data;
			data.telemStruct.sampleIndex = sampIdx;
//Stopwatch was already started in the cmdSpecialTelemetry function
			data.telemStruct.timeStamp = (long)swatchTic(); 

// since T1 has higher priority, these state readings might get interrupted 
	CRITICAL_SECTION_START  // need coherent sample without T1 int updates
//  save Hall encoder position instead of commanded thrust
		data.telemStruct.posL = pidObjs[0].p_state;
		data.telemStruct.posR = pidObjs[1].p_state;
	// save output instead of reading PWM (sync issue?)
			data.telemStruct.dcL = pidObjs[0].output;	// left
			data.telemStruct.dcR = pidObjs[1].output;	// right
			data.telemStruct.bemfL = bemf[0];
			data.telemStruct.bemfR = bemf[1];
	CRITICAL_SECTION_END
   
			data.telemStruct.gyroX = gdata[0] - offsx;
			data.telemStruct.gyroY = gdata[1] - offsy;
			data.telemStruct.gyroZ = gdata[2] - offsz; 
			data.telemStruct.gyroAvg = gyroAvg;

			data.telemStruct.accelX = xldata[0];
			data.telemStruct.accelY = xldata[1];
			data.telemStruct.accelZ = xldata[2];
			data.telemStruct.Vbatt = (int) adcGetVbatt();
			data.telemStruct.sOut = steeringPID.output;
*/
			telemGetPID(sampIdx);
// inside T5 interrupt, so don't need to DisableIntT5
			telemFlashSample(&telemPIDdata); 
}

/// write telemetry sample to Flash memory
void telemFlashSample(telemU* data)
{  dfmemSave((unsigned char *)data, sizeof(telemStruct_t));
	if(samplesToSave == 0) //Done sampling, commit last buffer
	{ dfmemSync(); }
			
}

void setSampleSaveCount(int count){
	samplesToSave = count;
}


//read telemetry from Flash Memory and send radio packets back
void telemFlashReadback(unsigned int count)
{  unsigned char status = 0;
	unsigned int sampLen = sizeof(telemStruct_t);
	unsigned long sampNum = 0;
	telemU data;
	DisableIntT5;		// prevent MPU access to SPI2
	for(sampNum = 0; sampNum < count; sampNum++)
	{ dfmemReadSample(sampNum, sampLen, (unsigned char *) &data);
	   if ((sampNum+1) != data.telemStruct.sampleIndex)
		while(1) // hang here if bad read
		{ blink_leds(1,200); }
	   radioConfirmationPacket(RADIO_DEST_ADDR,
						     CMD_SPECIAL_TELEMETRY, 
						     status, sampLen, (unsigned char *) &data);  
	// delay_ms(25);	// slow down for XBee 57.6 K
	blink_leds(1,20); // wait 40 ms to give plenty of time to send packets
	}
	// wait for DMA to finish and release SPI2
	while(port_status[1] == STAT_SPI_BUSY)
	{ blink_leds(1,300);   // waste some time
	}
	EnableIntT5;
}

