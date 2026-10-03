// Walrus_Demo: one row per second from a Walrus pressure and temperature
// sensor over I2C. Header once, then a reading and a row each loop.
#include <Walrus_I2C.h>

Walrus sensor;

void setup() {
  Serial.begin(9600);
  if (!sensor.begin()) {
    Serial.print("Walrus not found: ");
    sensor.printNote(Serial, true);  // NotAnswering, NotSchema1, WrongName, OldFirmware
    Serial.println();
  }
  sensor.printDataHeader(Serial);  // straight to the port: no row is built in RAM
  Serial.println();
}

void loop() {
  sensor.updateMeasurements();     // take the readings; printDataRow() prints what they left
  sensor.printDataRow(Serial);     // -9999.00 where a reading failed
  Serial.println();
  if (sensor.anyFault()) {
    sensor.printReport(Serial);  // e.g. "MS5803: not answering"
    Serial.println();
  }
  delay(1000);
}
