// Contents of this file are copyright Andrew Pullin, 2013

/******************************************************************************
* Name: settings.h
* Desc: Constants used by Andrew P. are included here.
* Author: pullin
******************************************************************************/
#ifndef __SETTINGS_H
#define __SETTINGS_H

//Which set of radio settings to use
#define RONF
//#define OCTOROACH
//#define MOTILE

#ifdef OCTOROACH
#define RADIO_CHANNEL         0x11
#define RADIO_PAN_ID          0x2110
#define RADIO_DST_ADDR       0x2111
#define RADIO_SRC_ADDR        0x2112
#endif

#ifdef RONF
#define RADIO_CHANNEL 0x13
#define RADIO_PAN_ID 0x2060
#define RADIO_DST_ADDR 0x2011
#define RADIO_SRC_ADDR 0x2052
#endif

#ifdef MOTILE
#define RADIO_CHANNEL 0x0e
#define RADIO_PAN_ID 0x3000
#define RADIO_DST_ADDR 0x3001
#define RADIO_SRC_ADDR 0x2052
#endif

#define RADIO_RXPQ_MAX_SIZE 	8
#define RADIO_TXPQ_MAX_SIZE	8


// Choose if Hall encoder present
#define HALL_SENSOR

#endif //__SETTINGS_H
