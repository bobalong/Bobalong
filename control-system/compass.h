/*
	compass.h 

	Provides functions for interacting with Bobalong's HMC6343 electronic compass

	This code is released under the terms of the LGPLv3 licence.
 */

#ifndef COMPASS_H
#define COMPASS_H

#include "type_defs.h"

// I2C device address
#define HMC6343_ADDRESS 			0x19

// HMC6343 registers
#define HMC6343_BEARING_REG 		0x50

namespace Compass
{
	/**********************************************************************************
	 * Returns the boat's bearing
	 * 
	 *********************************************************************************/
	bool GetBearing(BearingData& bearing);

	/**********************************************************************************
	 * Tries to read a register from the compass and puts the result in v0, v1, and v2.
	 * 
	 * @param  reg 				The register address to read.
	 * @param  v0  				The first value.
	 * @param  v1  				The second value.
	 * @param  v2  				The third value.
	 * 
	 * @return     				Returns true if the read was succesful.
	 *********************************************************************************/
	static bool Read(byte reg, int& v0, int& v1, int& v2 );

	/**********************************************************************************
	 * Reads a single value from the compass and returns the result.
	 * 
	 *********************************************************************************/
	static int ReadValue();

	/**********************************************************************************
	 * Combines a high and low byte into a single integer.
	 * 
	 *********************************************************************************/
	static int CombineByte(byte high, byte low);
}

#endif
