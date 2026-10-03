/******************************************************************************
SerialDemo.ino
Basic serial demo script for the Walrus pressure and temperature sensor, made by Northern Widget LLC.
Bobby Schulz @ Northern Widget LLC
2/28/2020
Hardware info located at: https://github.com/NorthernWidget-Skunkworks/Project-Walrus

This code reads temperature and pressure data from the Walrus over I2C and displays it via serial

"Size matters not. Look at me. Judge me by my size do you? And well you should not. For my ally is the
Force, and a powerful ally it is"
-Yoda

Distributed as-is; no warranty is given.
******************************************************************************/

#include <Walrus_I2C.h>

Walrus Sensor; //Instantiate Walrus instance 

const unsigned long UpdatePeriod = 5000; //Update every 5 seconds

void setup()
{
	Serial.begin(9600); //Initialize serial for basic communication 
	Serial.print("Begin Walrus Demo...\n\n"); 
	if(!Sensor.begin()) { //Initialize sensor; say why if it refuses
		Serial.print("Walrus not found: ");
		Sensor.printNote(Serial, true);
		Serial.println();
	}
	Sensor.printDataHeader(Serial); //Print header from Walrus to identify values
	Serial.println();
}

void loop()
{
	static unsigned long LocalTime = millis(); //Initialize local time keeping
	if((millis() - LocalTime) > UpdatePeriod) { //If a period has passed, print new values
		Sensor.updateMeasurements(); //Take the readings
		Sensor.printDataRow(Serial); //Print them straight to the port
		Serial.println();
	}
}