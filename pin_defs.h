/*
	pin_defs.h 

	Contains all the pin definitions.

	This code is released under the terms of the LGPLv3 licence.
 */

#ifndef PIN_DEF_H
#define PIN_DEF_H

// XBee
#define _TX_1 	11
#define _RX_1 	12

// GPS
#define _TX_2 	4
#define _RX_2 	9

#define _TX_3 	7
#define _RX_3 	10

#define _TX_4 	A3
#define _RX_4 	8


#define _SV_1 	5
#define _SV_2 	6 

#ifdef ARDUINO_YUN
// I2C (Compass)
#define _SDA	2
#define _SCL	3
#else
#define _SDA	A4
#define _SCL	A5
#endif

#endif
