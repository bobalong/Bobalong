/*
	gps.c

	Contains functions for dealing with the boat's GPS. To use the 
	GPS it must first be initialised. The GPS uses TinyGPS to parse the GPS NMEA strings

	This code is released under the terms of the LGPLv3 licence.
 */

#include "gps.h"
#include "TinyGPS.h"
#include <SoftwareSerial.h>
#include "Arduino.h"
#include "pin_defs.h"

#define GPS_FIX_UPDATE 2500

SoftwareSerial serial(_RX_2, _TX_2);
TinyGPS gps;

//////////////////////////////////////////////////////////////////////////
void GPS::Initialise()
{
	serial.begin(4800);
}

//////////////////////////////////////////////////////////////////////////
bool GPS::HasFix()
{
	long lat, lon;
	unsigned long fix_age;

	gps.get_position(&lat, &lon, &fix_age);
	if(fix_age == TinyGPS::GPS_INVALID_AGE) {
		Serial.println("No fix detected!");
		return false;
	} else if(fix_age > 5000) {
		Serial.println("Warning: GPS data possibly old!");
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////
GPSPosition GPS::GetPosition()
{
	GPSPosition gps_pos;
	unsigned long fix_age;

	gps.get_position(&gps_pos.latitude, &gps_pos.longitude, &fix_age);

	if(fix_age > GPS_FIX_UPDATE) {
		UpdateData();
	}

	return gps_pos;
}

//////////////////////////////////////////////////////////////////////////
GPSDateTime GPS::GetDateTime()
{
	//TODO: Should look at breaking this down into seperate components:
	// hour, minute, second, day, month, year

	GPSDateTime gps_date_time;
	unsigned long fix_age;

	gps.get_datetime(&gps_date_time.date, &gps_date_time.time, &fix_age);

	if(fix_age > GPS_FIX_UPDATE) {
		UpdateData();
	}

	return gps_date_time;
}

//////////////////////////////////////////////////////////////////////////
GPSCourse GPS::GetCourse()
{
	GPSCourse gps_course;

	gps_course.speed = gps.speed();
	gps_course.bearing = gps.course();

	return gps_course;
}

//////////////////////////////////////////////////////////////////////////
void GPS::UpdateData()
{
	serial.listen();

	while(serial.available()) {
		gps.encode(serial.read());
	}
}
