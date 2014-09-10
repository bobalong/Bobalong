// Arduino Libraries
#include <Servo.h>
#include <Wire.h>
#include <stdint.h>
#include <SoftwareSerial.h>
#include <FileIO.h>
#include <Time.h>

/////////////////////////////////////////////////////////////////
// Boat libraries
#include "type_defs.h"
#include "pin_defs.h"
#include "compass.h"
#include "wind_sensor.h"
#include "gps.h"

#include "navigation.h"
#include "waypoint_mgr.h"

/////////////////////////////////////////////////////////////////
// Defines
#define SAIL_ROTATE_AMOUNT		25

// Set once right, so surely only two possible values for the sail exist or am I going crazy???
#define SAIL_LEFT				45
#define SAIL_RIGHT				135

#define TACK_AMOUNT				45
#define TACK_DISTANCE			25 // In metres
#define RUDDER_CHANGE_FREG		5 // The number of loops before we update the rudder
#define RUDDER_P_COEF			0.5f
#define HOLDING_DISTANCE_MIN	1; // In metres
#define HOLDING_DISTANCE_MAX	5; // In metres

/////////////////////////////////////////////////////////////////
// Variables

Boat boat;
Servo rudder;
Servo sail;

enum SailingMode {
	SM_NORMAL,
	SM_HOLD,
	SM_TACKING
};

SailingMode current_mode;
int rudder_freq_counter;

// Sailing
int desired_heading;
int relative_wind;

// Tacking
bool change_tack;
bool tack_left;
int tack_heading;
GPSPosition tack_pos;

// Holding 
bool reversing;

//////////////////////////////////////////////////////////////////////////
void setup() {
	WindSensor::Initialise();
	GPS::Initialise();
    Bridge.begin(); 
    Serial.begin(9600);
    FileSystem.begin();

    rudder.attach(_SV_1);
    sail.attach(_SV_2);
        
	// Wait for everything to get running.
	delay(100);

	// Light up debugger LED
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

	// Hold untill we tell it to sail
	current_mode = SM_HOLD;
	change_tack = true;
	tack_left = true;
    desired_heading = 0;
	rudder_freq_counter = RUDDER_CHANGE_FREG;
	reversing = false;

	// Reset servos
	sail.write(90);
	rudder.write(90);
}

//////////////////////////////////////////////////////////////////////////
void loop() {
	// LED debugger
	digitalWrite(13, HIGH);
	// Update data
	UpdateCompass();
	UpdateGPS();
	delay(77);
	UpdateWind();
					
	// Log it
	LogData();

	desired_heading = Navigation::GetDesiredHeading(boat.position, WaypointMgr::GetCurrentWaypoint());
	relative_wind = Navigation::GetHeadingDiff(boat.bearing.heading, boat.wind.direction);

	switch(current_mode) {
		case SM_NORMAL:
			SailMode_Normal();
		break;
		case SM_HOLD:
			SailingMode_Hold();
		break;
		case SM_TACKING:
			SailingMode_Tack();
		break;
	}

	if(rudder_freq_counter >= RUDDER_CHANGE_FREG) {
		rudder_freq_counter = 0;
		KeepHeading();
	} else {
		rudder_freq_counter++;
	}

	// Causes the debug LED to flash, so we know the code hasn't gotten
	// stuck somewhere.
	digitalWrite(13,LOW);
	
	// Don't update or log for 1 second
	delay(100);
}

//////////////////////////////////////////////////////////////////////////
void UpdateCompass() {
	BearingData bearing;

	if(Compass::GetBearing(bearing)) {
		boat.bearing = bearing;
	} else {
		Serial.println("Warning: Failed to read compass!");
	}
}

//////////////////////////////////////////////////////////////////////////
void UpdateWind() {
	WindData wind;
	if(WindSensor::Read(wind)) {
		boat.wind = wind;
	} else {
		Serial.println("Warning: Failed to read wind sensor");
	}
}

//////////////////////////////////////////////////////////////////////////
void UpdateGPS(){
	boat.has_fix = GPS::HasFix();
	if(boat.has_fix) {
		boat.position = GPS::GetPosition();
		boat.course = GPS::GetCourse();
	}
	boat.date_time = GPS::GetDateTime();
	setTime(boat.date_time.time);
}

//////////////////////////////////////////////////////////////////////////
void LogData() {
	// Time
	Serial1.print("time="); Serial1.print(now()); Serial1.print(" ");
	// Boat Heading
	Serial1.print("bhead="); Serial1.print(boat.bearing.heading); Serial1.print(" ");
	// GPS Heading
	Serial1.print("gpshead="); Serial1.print(boat.course.bearing); Serial.print(" ");
	// Speed
	Serial1.print("knots="); Serial1.print(boat.course.speed); Serial1.print(" ");
	// Boat Wind Dir
	Serial1.print("wind="); Serial1.print(boat.wind.direction); Serial1.print(" ");
	// Wind speed
	Serial1.print("windSpd="); Serial1.print(boat.wind.speed); Serial1.print(" ");
	// Lat
	Serial1.print("lat="); Serial1.print(boat.position.latitude,5); Serial1.print(" ");
	// Long
	Serial1.print("lon="); Serial1.print(boat.position.longitude,5); Serial1.println(" ");
}

//////////////////////////////////////////////////////////////////////////
void SailMode_Normal()
{
	if(WaypointMgr::GetWaypointCount() == 0) {
		current_mode = SM_HOLD;
	} else {
		// Determine if we are at the waypoint and if so advance it
		if(WaypointMgr::CheckProximity(boat.position)) {
			WaypointMgr::Advance();
			return;
		}
		
		// Decide if we need to tack
		if(ShouldTack()) {
			current_mode = SM_TACKING;
			change_tack = true;
		} else {
			// set the sails
			if(relative_wind < 180) {
				sail.write(SAIL_RIGHT);
			} else {
				sail.write(SAIL_LEFT);
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
void SailingMode_Hold()
{
	// When in hold mode, the boat will try to hold its current position	
	if(!reversing) {/*
		if(Navigation::GetDistance(boat.position, WaypointMgr::GetCurrentWaypoint()) <= HOLDING_DISTANCE_MIN) {
			reversing = !reversing;
		}*/

		// set the sails
		if(relative_wind < 180) {
			sail.write(SAIL_RIGHT);
		} else {
			sail.write(SAIL_LEFT);
		}
	} else {/*
		if(Navigation::GetDistance(boat.position, WaypointMgr::GetCurrentWaypoint()) >= HOLDING_DISTANCE_MAX) {
			reversing = !reversing;
		}*/
		// set the sails
		if(relative_wind > 180) {
			sail.write(SAIL_RIGHT);
		} else {
			sail.write(SAIL_LEFT);
		}
	}
}

//////////////////////////////////////////////////////////////////////////
void SailingMode_Tack()
{
	// Check if we should still tack
	if(!ShouldTack()) {
		current_mode = SM_NORMAL;
		return;
	}

	// Set the initial tack if we haven't set it yet
	if(change_tack) {
		change_tack = false;

		tack_pos = boat.position;

		// Set the target tacking heading
		if(tack_left) {
			desired_heading += TACK_AMOUNT;
			tack_left = false;
			sail.write(SAIL_LEFT);
		} else {
			desired_heading -= TACK_AMOUNT;
			tack_left = true;
			sail.write(SAIL_RIGHT);
		}
		Navigation::ClampAngle(desired_heading);
	}

	// Check if we should change tack
	if(Navigation::GetDistance(boat.position, tack_pos) >= TACK_DISTANCE) {
		change_tack = true;
	}
}

//////////////////////////////////////////////////////////////////////////
bool ShouldTack()
{
	int relative_wind = Navigation::GetHeadingDiff(boat.bearing.heading, boat.wind.direction);
	if(abs(relative_wind >= TACK_AMOUNT)) {
		return true;
	} else {
		return false;
	}
}

//////////////////////////////////////////////////////////////////////////
void KeepHeading() {
	int headingOff = Navigation::GetHeadingDiff(boat.bearing.heading, desired_heading);
	if(headingOff > -1 && headingOff < 1) {
		rudder.write(90);
		return;
	}
	int rot = 90 + (int)(headingOff * RUDDER_P_COEF);
	rudder.write(rot);
}
