/*
	wind_sensor.cpp

	Provides functions for interacting with Bobalong's Rowind wind sensor. The
	wind sensor uses a software serial to communicate with the device.

	This code is released under the terms of the LGPLv3 licence.
 */

#include "wind_sensor.h"
#include "Arduino.h"
#include "pin_defs.h"
#include <SoftwareSerial.h>

SoftwareSerial wind_ss(_RX_4, _TX_4);

//////////////////////////////////////////////////////////////////////////
void WindSensor::Initialise()
{
	wind_ss.begin(4800);
}

//////////////////////////////////////////////////////////////////////////
bool WindSensor::Read(WindData& wind_data)
{
	wind_ss.listen();
  	char* line = GetNMEA();
  	// Make sure nothing went wrong and that we have a valid line
  	if(line == 0) {
  		return false;
  	}

	// Parse line
	char *s = line;
	char *str;
	int i = 0;

	//  Splits the string into tokens whic hare seperated by ','. Returns null when at the end
	while ((str = strtok_r( s, ",", &s )) != NULL ) {
		// Second token contains the wind direction
		if ( i == 1 ) {
			wind_data.direction = atof( str );

		// fourth token contains wind speed
		} else if ( i == 3 ) {
			wind_data.speed = atof( str );
		}
		i++;
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////
char* WindSensor::GetNMEA()
{
	char line[50];
	bool gotData = false;
	unsigned long endTime = millis() + 5000;

	// Search for the correct rowind data line, this might could be 
	// cleaned up I think
	while(!gotData && millis() < endTime) {
		char c = wind_ss.read();
		delay(3);
		// Start of a rowind sentence
		if(c == '$') {
			Serial.println();
			Serial.print("Roind NMEA: ");
			int i = 0;

	       	// Reads a line
			while(c != '\n' && i < 80) {
				line[i] = c;
				Serial.print(c);
				c = wind_ss.read();
				delay(3);
				i++;
			}

			// Data we want starts with $IIMWV
			if(line[1] == 'I') {
				gotData = true;
			}
		}
	}

	Serial.println();
        
    if(!gotData) {
    	Serial.println("Rowind: Failed to find the right or any NMEA ")
        return 0;
    }
        
	return line;
}
