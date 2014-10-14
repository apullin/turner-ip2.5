#ifndef __INIT_H
#define __INIT_H

void SetupADC(void);
//void SetupI2C(void);
void SetupInterrupts(void);
void SetupPorts(void);
void SetupTimer1(void);  // PID
void SetupTimer2(void);  // check radio queue
void SetupUART2(void);
// void SetupPWM(void);
extern unsigned int PTPERvalue;

#endif
