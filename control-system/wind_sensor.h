/*
	wind_sensor.h

	Provides functions for interacting with Bobalong's Rowind wind sensor. The
	wind sensor uses a software serial to communicate with the device.

	This code is released under the terms of the LGPLv3 licence.
 */

#ifndef WIND_SENSOR_H
#define WIND_SENSOR_H

#include "type_defs.h"

namespace WindSensor
{
	/**********************************************************************************
	 * Initialises the wind sensor. Creates and opens a software serial to the rowind.
	 * 
	 *********************************************************************************/
	void Initialise();

	/**********************************************************************************
	 * Listens to the wind sensor's software serial and fills a wind data struct with
	 * wind data.
	 * 
	 * @param  wind_data 			The struct to fill with wind data.
	 * 
	 * @return           			Returns true if the read operation was successful
	 * 
	 *********************************************************************************/
	bool Read(WindData& wind_data);

	/**********************************************************************************
	 * Returns a Rowind's NMEA string.
	 * 
	 * @return 						A char array of 30 characters, in the format:
	 *                 				$IIMWV,315.0,R,000.00,N,A
	 * 
	 *********************************************************************************/
	static char* GetNMEA();
}

#endif
