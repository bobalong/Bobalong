/*
	compass.cpp

	Provides functions for interacting with Bobalong's HMC6343 electronic compass

	This code is released under the terms of the LGPLv3 licence.
 */

#include "compass.h"
#include "Arduino.h"
#include "Wire.h"


//////////////////////////////////////////////////////////////////////////
bool Compass::GetBearing(BearingData& bearing)
{
	return Compass::Read(HMC6343_BEARING_REG, bearing.heading, bearing.roll, bearing.pitch);
}

//////////////////////////////////////////////////////////////////////////
bool Compass::Read(byte reg, int& v0, int& v1, int& v2 )
{
	byte high, low;
	unsigned long timeout = 1000;
	unsigned long last_time;
        
	// Start the communication with the I2C device
	Wire.beginTransmission(HMC6343_ADDRESS);
    Serial.println("Compass: BEGIN");
	// Send the address of the registers we want to read
	Wire.write(reg);
	Serial.println("Compass: WRITE");
	Wire.endTransmission();
    Serial.println("Compass: END TRANSMISSION");

	// Requests 6 bytes from the compass and then a stop condition is
	// sent releaing the bus.
	Wire.requestFrom(HMC6343_ADDRESS, 6, true);
    Serial.println("Compass: REQUEST 6 bytes");
	 // Wait for the data, return false marking the read as failed if
	 // we go over the waiting time
	last_time = millis();
	while(Wire.available() < 6) {
                Serial.print("COMPASS: Timeout left="); Serial.println(millis() - last_time);
		if(last_time + timeout > millis()) {
			Serial.println("Compass: TIMEOUT");
			return false;
		}
	}

	// Read the data
	v0 = ReadValue();
	v1 = ReadValue();
	v2 = ReadValue();

	return true;
}

//////////////////////////////////////////////////////////////////////////
int Compass::ReadValue()
{
	byte high, low;

	// read in the high and low bytes
	high = Wire.read();
	low = Wire.read();

	// Combine them into a single int so we can get one value from the 
	// compass.
	return CombineByte(high, low);
}

//////////////////////////////////////////////////////////////////////////
int Compass::CombineByte(byte high, byte low)
{
	return ((high << 8)+ low);
}
