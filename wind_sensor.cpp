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

#define DEBUG_ROWIND

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
	char nmea_buffer[50];
	bool got_data = false;
	unsigned long timeout = millis() + 5000;

#ifdef DEBUG_ROWIND
	Serial.println("Reading RoWind serial...");
	delay(1000);
#endif

	// Keep going until we either get the correct data or timeout after
	// 5 seconds
	while(!got_data & millis() <= timeout) {
		// Gives the rowind enough time to send a char
		delay(3);
		char c = wind_ss.read();

		#ifdef DEBUG_ROWIND
		Serial.print(c);
		#endif

		// Check if the char is the start of a nmea string
		if(c == '$') {
			#ifdef DEBUG_ROWIND
			// Starts a new line on the debug terminal, for better viewing
			Serial.println();
			#endif

			// Once we know we are at the start of a nmea string we read
			// until we hit the end of the string and store it in a buffer
			int i = 0;
			while(c != '\n' & i < 50) {
				nmea_buffer[i] = c;

				#ifdef DEBUG_ROWIND
				Serial.print(c);
				#endif

				c = wind_ss.read();
				i++;
			}

			// Now check if we have a $IIMWV nmea string
			if(nmea_buffer[1] == 'I') {
				got_data = true;
				#ifdef DEBUG_ROWIND
				Serial.println("Found $IIMWV");
				#endif
			}
		}
	}

    if(!got_data) {
    	Serial.println("Rowind timeout");
        return 0;
    }
        
	return nmea_buffer;
}
