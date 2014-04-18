// Arduino Libraries
#include <Servo.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <stdint.h>

/////////////////////////////////////////////////////////////////
// Boat libraries
#include "Types.h"
#include "HMC6343.h"
#include "TinyGPSPlus.h"
#include "Rowind.h"

/////////////////////////////////////////////////////////////////
// Variables

#define GPS_Enable 7
#define GPS_RXPIN 2
#define GPS_TXPIN 3

#define ROWIND_PIN 4

// Servo pins
#define RUDDER_PIN 9 // Blue wire
#define SAIL_PIN 10 // Gray wire

//Define pid coefs
#define RUDDER_P_COEF 0.5f


HMC6343 Compass;
BoatData Boat;
TinyGPSPlus gps;
Rowind rowind(ROWIND_PIN,5);
SoftwareSerial ss(GPS_RXPIN,GPS_TXPIN);


Servo rudder;
// Autonomous sailing
int m_DestHeading;

/////////////////////////////////////////////////////////////////
void setup() {
	Serial.begin(9600);
	ss.begin(4800);

	// Light up debugger LED
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

	// Check all components are working
	if(!Compass.IsFunctioning()) {
		Serial.println("Error with compass!");
	}
	if(!rowind.IsFunctioning()) {
		Serial.println("Error with Rowind!");
	}

        
        // GPS enable
        pinMode(GPS_Enable, OUTPUT);
        digitalWrite(GPS_Enable, HIGH);

	// Get starting boat heading
	UpdateCompass();
	m_DestHeading = Boat.Heading;
	Serial.println(m_DestHeading);
 
	rudder.attach(RUDDER_PIN);
	rudder.write(90);
}

/////////////////////////////////////////////////////////////////
void loop() {
	// LED debugger
	digitalWrite(13, HIGH);
	ss.listen();
	// Update data
	UpdateCompass();
	UpdateGPS();
	delay(77);

	//rowind.GetData(Boat.WindDirection, Boat.WindSpeed);

	KeepHeading();

	// Log it
	LogData();

	digitalWrite(13,LOW);

	// Don't update or log for 5 seconds
	delay(500);
}

/////////////////////////////////////////////////////////////////
// Update data from hardware
/////////////////////////////////////////////////////////////////

void UpdateCompass() {
	Compass.GetBearing(Boat.Heading, Boat.Pitch, Boat.Roll);
}


void UpdateGPS(){
	while (ss.available()>0)
	{  
		delay(1);
		if (gps.encode(ss.read()))
		{
			// Debugger LED
			digitalWrite(13, LOW);

			// Store GPS data
			Boat.Latitude = gps.location.lat();
			Boat.Longitude = gps.location.lng();

			Boat.DateDay = gps.date.day();
                       	Boat.DateMonth = gps.date.month(); 

			Boat.TimeHours = gps.time.hour();
                        Boat.TimeMinutes = gps.time.minute();
                        Boat.TimeSeconds = gps.time.second();

                        Boat.Course = gps.course.deg();
                        Boat.Speed = gps.speed.knots();
		}
	}   
}
/////////////////////////////////////////////////////////////
// Logs all the data to the openlogger
void LogData() {
	// Boat Heading
	Serial.print("bhead="); Serial.print(Boat.Heading); Serial.print(" ");
	// GPS Heading
	Serial.print("gpshead="); Serial.print(Boat.Course); Serial.print(" ");
	// Speed
	Serial.print("knots="); Serial.print(Boat.Speed); Serial.print(" ");
	// Boat Wind Dir
	Serial.print("wind="); Serial.print(Boat.WindDirection); Serial.print(" ");
	// Wind speed
	Serial.print("windSpd="); Serial.print(Boat.WindSpeed); Serial.print(" ");
	// Lat
	Serial.print("lat="); Serial.print(Boat.Latitude,5); Serial.print(" ");
	// Long
	Serial.print("lon="); Serial.print(Boat.Longitude,5); Serial.print(" ");
	// Date
	Serial.print("date="); Serial.print(Boat.DateDay); Serial.print(":"); Serial.print(Boat.DateMonth); Serial.print(" ");
	// TIme
	Serial.print("time="); Serial.print(Boat.TimeHours); Serial.print(":"); Serial.print(Boat.TimeMinutes); Serial.print(":"); Serial.print(Boat.TimeSeconds); Serial.println(" ");
}

int getHeadingDifference(int heading1, int heading2){
  int result = heading1 - heading2;
  
  if (result < -180){
   return 360 + result;
  }
  if ( result > 180 ){
   return 0 - (360 - result);
  }
  return result;
}

/*setWaypoint(latitude, longitude){
    
}*/

/////////////////////////////////////////////////////////////
// Tries to keep the boat heading in the direction it was 

// facing when started.
void KeepHeading() {
	int headingOff = getHeadingDifference(Boat.Heading, m_DestHeading);

	if(headingOff > -1 && headingOff < 1) {
		rudder.write(90);
		return;
	}

        int rot = 90 + (int)(headingOff * RUDDER_P_COEF);
        Serial.print("Rudder: "); Serial.println(rot);
        rudder.write(rot);
}