/*
	gps.h 

	Contains functions for dealing with the boat's GPS. To use the 
	GPS it must first be initialised. The GPS uses TinyGPS to parse the GPS NMEA strings

	This code is released under the terms of the LGPLv3 licence.
 */

#ifndef GPS_H
#define GPS_H

#include "Arduino.h"
#include "type_defs.h"

namespace GPS
{
	/**********************************************************************************
	 * Initialises the GPS software serial.
	 * 
	 *********************************************************************************/
	void Initialise();

	/**********************************************************************************
	 *  Determines if the GPS has a fix and returns true if it does.
	 *  
	 *********************************************************************************/
	bool HasFix();

	/**********************************************************************************
	 * Returns the current position of the boat
	 * 
	 * @return 				Returns a GPSPosition struct containing latitude and longitude.
	 * 
	 *********************************************************************************/
	GPSPosition GetPosition();

	/**********************************************************************************
	 * Returns the data and time from the GPS.
	 * 
	 * @return 				Returns a GPSDataTime struct containing time and date
	 * 
	 *********************************************************************************/
	GPSDateTime GetDateTime();

	/**********************************************************************************
	 * Returns a struct containing the boat's GPS course and speed.
	 * 
	 * @return 				Returns a GPSCourse struct.
	 * 
	 *********************************************************************************/
	GPSCourse GetCourse();

	/**********************************************************************************
	 * Updates all the GPS data.
	 * 
	 *********************************************************************************/
	static void UpdateData();
}

#endif
