/*
	type_defs.h 

	Contains all the type defs and structs used in the project.

	This code is released under the terms of the LGPLv3 licence.
 */

#ifndef TYPES_H
#define TYPES_H

typedef unsigned int uint;
typedef unsigned char byte;

struct BearingData {
	int heading;
	int roll;
	int pitch;
};

struct WindData {
	float speed;
	float direction;
};

struct GPSPosition {
	long latitude;
	long longitude;
};

struct GPSCourse {
	unsigned long speed;
	unsigned long bearing;
};

struct GPSDateTime {
	// ddmmyy
	unsigned long date;
	//hhmmss
	unsigned long time;
};

struct Boat {
	BearingData bearing;
	WindData	wind;
	GPSPosition	position;
	GPSDateTime	date_time;
	GPSCourse	course;
	bool		has_fix;
};

#endif
