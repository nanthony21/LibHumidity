#include <LibHumidity.h>

/****************************
 * GetRelativeHumidity
 *  An example sketch that reads the sensor and prints the
 *  relative humidity to the PC's serial port
 *
 *  Tested with the HTU21 Breakout Board
 *****************************/
#include <Wire.h>
#include <LibHumidity.h>

LibHumidity humidity = LibHumidity(eSensorHTU21);

void setup() {
  Serial.begin(9600);
  humidity.ResetSensor();
  delay(5000);
}

void loop() {
  Serial.print("RH: ");
  Serial.print(humidity.GetHumidity());
  Serial.print(" Temp: ");
  Serial.println(humidity.GetTemperatureC());
  delay(2000);
}

