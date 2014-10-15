/*  splitting command file to make more manageable
    old, unused or obsolete commands can go in here 
*/

static void cmdGetImuLoopZGyro(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
    //unsigned int count;     unsigned long tic;   unsigned char *tic_char;
    //MacPacket packet; Payload pld;
/*
    unsigned char* gyrotemp;
	int zgyro;
    //int perpacket = 10;
	int perpacket = 2;
    int i;
    int packidx = 0;
    int dc1, dc2;
    //LED_RED = 1;
	int intwz;
	unsigned char gdata[6];
	unsigned char* gdatap = gdata;
	int* zp = (int*)(gdatap + 4);
    count = frame[0] + (frame[1] << 8);
    tic_char = (unsigned char*)&tic;
    swatchReset();

    while (count) {
        pld = payCreateEmpty(10*perpacket); 
        packidx = 0;
        for(i = 0; i < perpacket; i++){

            tic = swatchTic();
            //if (tic == 4) //testing why tic seems to reset at ~1.2 seconds
            //{
            //        asm volatile("nop");
            //        asm volatile("nop");
            //}
			gyroGetXYZ(gdata);
			intwz = *zp - offsz;

            payAppendData(pld, packidx, 4, tic_char); //time , long , 4 bytes
            packidx += 4;
            payAppendData(pld, packidx, 2, (unsigned char*)(&intwz) );  //gyro data , int, 2 bytes
            packidx += 2;
            dc1 = PDC1; dc2 = PDC2;
            payAppendData(pld, packidx, 2, (unsigned char*)(&dc1) );  //duty cycle 1, int, 2 bytes
            packidx += 2;
            payAppendData(pld, packidx, 2, (unsigned char*)(&dc2) );  //duty cycle 2 , int, 2 bytes
            packidx += 2;
            count--;
            delay_ms(4);
        }
		
        paySetStatus(pld, status);
        paySetType(pld, CMD_GET_IMU_LOOP_ZGYRO);

   //     radioSendPayload(macGetDestAddr(),pld);
         // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));

    }

    //LED_RED = 0;
*/
}

/*-----------------------------------------------------------------------------
 *          IMU functions
-----------------------------------------------------------------------------*/
static void cmdGetImuData(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {

//	senGetIMUData(status, CMD_GET_IMU_DATA);


    /*unsigned char *xl_data, *gyro_data;
    unsigned char i;
    Payload pld = payCreateEmpty(14);  // data length = 12

    xlReadXYZ();
    xl_data = xlGetsXYZ();
    gyroReadXYZ();
    gyro_data = gyroGetsXYZ();
    for(i = 0; i < 6; ++i) {
        pld->pld_data[i] = xl_data[i];
        pld->pld_data[i+6] = gyro_data[i];
    }

    pld->status = status;
    pld->type = CMD_GET_IMU_DATA;
    radioTxPayload(pld);
    */
}


// return packet format:
// 4 bytes for time
// 6 bytes for xl data
// 6 bytes for gyro data
static void cmdGetImuLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
/*
    unsigned int count;
    unsigned long tic;
    unsigned char *tic_char;
    Payload pld;
  
    LED_RED = 1;

    count = frame[0] + (frame[1] << 8);

    tic_char = (unsigned char*)&tic;
    swatchReset();
    tic = swatchTic();
    
    while (count) {
        pld = payCreateEmpty(16);  // data length = 16
        paySetData(pld, 4, tic_char);
        payAppendData(pld, 4, 6, xlReadXYZ());
        payAppendData(pld,10, 6, gyroReadXYZ());
        paySetStatus(pld, status);
        paySetType(pld, CMD_GET_IMU_DATA);

        radioSendPayload(macGetDestAddr(),pld);
        count--;
		payDelete(pld);
        delay_ms(4);  
        tic = swatchTic();
    }
    LED_RED = 0;
*/
}


static void cmdStartImuDataSave(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
//    senSetImuDataSave(1);  
}

static void cmdStopImuDataSave(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
  //  senSetImuDataSave(0);  
}

static void cmdTxSavedImuData(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
 /*  
    unsigned int page, byte;
    unsigned int i, j;
    Payload pld;

    //senGetMemLocIndex(&page, &byte);
    page = 0x0200;
    byte = 0;

    LED_RED = 1;

    dfmemEraseSector(0x0100);   // erase Sector 1 (page 256 - 511)

    for (i = 0x0100; i < 0x0200; ++i) {
        j = 0;
        while (j < 512) {
            pld = payCreateEmpty(18);  // data length = 16
            dfmemRead(i, j, 16, pld->pld_data);
            paySetStatus(pld, status);
            paySetType(pld, CMD_GET_IMU_DATA);
            while(!radioReceivePayload());
            j += 16;
        }
        delay_ms(200);
    }
    LED_RED = 0;
*/
}



static void cmdSetMoveQueue(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
	unsigned int count;
	int idx = 0;
	count = (unsigned int)(*(frame+idx));
	idx += 2;
	moveCmdT move;
	int i;
	unsigned char durtemp[4];
	unsigned long* dur = (unsigned long*)durtemp; // requires for unsigned long read
	
	for(i = 0; i < count; i++){
		move = (moveCmdT)malloc(sizeof(moveCmdStruct));
		//move->inputL = (int)(*(frame+idx));
		move->inputL = frame[idx] + (frame[idx+1] << 8); idx+=2;
		//idx += 2;
		//move->inputR = (int)(*(frame+idx));
		move->inputR = frame[idx] + (frame[idx+1] << 8); idx+=2;
		//idx += 2;
		//apparently this is required to properly read the unsigned long.
		durtemp[0] = frame[idx];
		durtemp[1] = frame[idx+1];
		durtemp[2] = frame[idx+2];
		durtemp[3] = frame[idx+3];
		move->duration = dur[0];
		idx += 4;
		mqPush(moveq, move);
	}
}

static void cmdSetSteeringGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	int Kp, Ki, Kd, Kaw, ff;
	int idx = 0;
	MacPacket packet; Payload pld;
	Kp = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Ki = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kd = frame[idx] + (frame[idx+1] << 8); idx+=2;
	Kaw = frame[idx] + (frame[idx+1] << 8); idx+=2;
	ff = frame[idx] + (frame[idx+1] << 8); idx+=2;
	steeringSetGains(Kp,Ki,Kd,Kaw, ff);
	//Send confirmation packet
	pld = payCreateEmpty(10);
	pld->pld_data[0] = status;
    pld->pld_data[1] = CMD_SET_STEERING_GAINS;
	memcpy((pld->pld_data)+2, frame, 10);
//	radioSendPayload((WordVal)macGetDestAddr(), pld);
// ip2.5c radio command:

    // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));
}

static void cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	//unsigned char chan1[4], chan2[4];
    //float *duty_cycle1 = (float*)chan1;
	//float *duty_cycle2 = (float*)chan2;
	int dc1, dc2;

	dc1 = *(int*)(frame);
	dc2 = *(int*)(frame + sizeof(int) );

    /*chan2[0] = frame[0];
    chan2[1] = frame[1];
    chan2[2] = frame[2];
    chan2[3] = frame[3];

	chan1[4] = frame[4];
    chan1[5] = frame[5];
    chan1[6] = frame[6];
    chan1[7] = frame[7];*/
    
	PDC1 = dc1;
	PDC2 = dc2;

    //mcSetDutyCycle(MC_CHANNEL_PWM1, duty_cycle1[0]);
	//mcSetDutyCycle(MC_CHANNEL_PWM2, duty_cycle2[0]);
	//mcSetDutyCycle(1, duty_cycle[0]);
}


static void cmdSteer(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
    
    unsigned char chr_test[4];
    float *steer_value = (float*)chr_test;

    chr_test[0] = frame[0];
    chr_test[1] = frame[1];
    chr_test[2] = frame[2];
    chr_test[3] = frame[3];
/*
    mcSteer(steer_value[0]);
*/
}

static void cmdSetCtrldTurnRate(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
	int rate;
	int idx = 0;
	MacPacket packet; Payload pld;
	rate = frame[0] + (frame[1] << 8);
	setSteeringAngRate(rate);
	
	//Send confirmation packet
	pld = payCreateEmpty(sizeof(int));
	pld->pld_data[0] = status;
    pld->pld_data[1] = CMD_SET_CTRLD_TURN_RATE;
	memcpy((pld->pld_data)+2, frame, sizeof(int));
//	radioSendPayload((WordVal)macGetDestAddr(), pld);	
// ip2.5c radio command:

    // Enqueue the packet for broadcast
    while(!radioEnqueueTxPacket(packet));
}


static void cmdSpecialTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame){
/*	int count;
	unsigned long temp;
	//count = (int)(*(frame));
	count = frame[0] + (frame[1] << 8);
	if(count != -1){
		swatchReset();
		setSampleSaveCount(count);
	       temp = t1_ticks;  // need atomic read due to interrupt 
		TelemControl.start = temp; // start recording now
		TelemControl.skip = 1; // every other sample (150 Hz)
	} else{ //start a readback over the radio
		//shut down all movement before sending
		pidSetInput(0,0,0);
		pidSetInput(1,0,0);
		readDFMem();
	}
*/
}



static void cmdEraseMemSector(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame) {
    unsigned int page;
	page = frame[0] + (frame[1] << 8);
    LED_RED = 1;
    dfmemEraseSector(0x0100);   // erase Sector 1 (page 256 - 511)
    LED_RED = 0;
}




