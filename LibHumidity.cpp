/*
  LibHumidity - A Humidity Library for Arduino.

  Supported Sensor modules:
    SHT21-Breakout Module - http://www.moderndevice.com/products/sht21-humidity-sensor

  Created by Christopher Ladden at Modern Device on December 2009.
  modified by Paul Badger March 2010

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <inttypes.h>
#ifdef LIBHUMIDITY_USE_I2C_T3
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif

//#include <wiring.h>
#include "LibHumidity.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/******************************************************************************
 * Constructors
 ******************************************************************************/

/**********************************************************
 * Initialize the sensor based on the specified type.
 **********************************************************/
LibHumidity::LibHumidity(uint8_t sensorType) {
    Wire.begin();

    switch (sensorType) {
        case eSensorHTU21:
            SetReadDelay(50, 16);
            break;
        case eSensorSHT21:
        case eSensorUnknown:
            SetReadDelay(85, 29);
            break;
    }
}

#ifdef LIBHUMIDITY_USE_I2C_T3

LibHumidity::LibHumidity(uint8_t sensorType, i2c_pins pins, i2c_rate rate) {
    Wire.begin(I2C_MASTER, 0, pins, I2C_PULLUP_EXT, rate);

    switch (sensorType) {
        case eSensorHTU21:
            SetReadDelay(50, 16);
            break;
        case eSensorSHT21:
        case eSensorUnknown:
            SetReadDelay(85, 29);
            break;
    }
}

#endif

/******************************************************************************
 * Global Functions
 ******************************************************************************/

/**********************************************************
 * GetHumidity
 *  Gets the current humidity from the sensor.
 *
 * @return float - The relative humidity in %RH
 **********************************************************/
float LibHumidity::GetHumidity(void) {

    float humidity;

    humidity = calculateHumidity(readSensor(eRHumidityHoldCmd));

    return humidity;
}

/**********************************************************
 * GetTemperatureC
 *  Gets the current temperature from the sensor.
 *
 * @return float - The temperature in Deg C
 **********************************************************/
float LibHumidity::GetTemperatureC(void) {

    float temperature;

    temperature = calculateTemperatureC(readSensor(eTempHoldCmd));

    return temperature;
}

void LibHumidity::ResetSensor() {
	Wire.beginTransmission(eSHT21Address);
	Wire.write(eSoftResetCmd);
	delay(15);
	Wire.endTransmission();
}

/**********************************************************
 * SetReadDelay
 *  Set the I2C Read delay from the sensor.
 *
 *  The SHT21 humidity sensor datasheet says:
 *  Parameter Resolution typ max Units
 *    14 bit      66        85      ms
 *    13 bit      33        43      ms
 *    12 Bit      17        22      ms
 *    11 bit       8        11      ms
 *    10 bit       4         6      ms
 *
 *      Measurement time
 *      (max values for -40°C
 *        125°C.)
 *      8 bit 1 3 ms
 *
 **********************************************************/
void LibHumidity::SetReadDelay(uint16_t readTemperatureDelay, uint16_t readHumidityDelay) {
    this->readTemperatureDelay = readTemperatureDelay;
    this->readHumidityDelay = readHumidityDelay;
}

/******************************************************************************
 * Private Functions
 ******************************************************************************/

uint16_t LibHumidity::getDelay(uint8_t command) {
    static const int holdBitMask = 0x10;

    command &= ~holdBitMask;
    if (command == eTempHoldCmd) {
        return readTemperatureDelay;
    }
    if (command == eRHumidityHoldCmd) {
        return readHumidityDelay;
    }

    return 0;
}

uint16_t LibHumidity::readSensor(uint8_t command) {
    uint16_t result;

    Wire.beginTransmission(eSHT21Address);   //begin
    Wire.write(command);                      //send the pointer location

    uint8_t readDelay = getDelay(command);
    delay(readDelay);

    Wire.endTransmission();                  //end

    Wire.requestFrom(eSHT21Address, 3);
    while(Wire.available() < 3) {
      ; //wait
    }

    //Store the result
    result = (Wire.read()) << 8;
    result |= Wire.read();
    result &= ~0x0003;   // clear two low bits (status bits)
    return result;
}

float LibHumidity::calculateTemperatureC(uint16_t analogTempValue) {
    return 175.72 / 65536.0 * (float)analogTempValue - 46.85; //T= -46.85 + 175.72 * ST/2^16
}

float LibHumidity::calculateHumidity(uint16_t analogHumValue) {
  return -6.0 + 125.0/65536.0 * analogHumValue;       // RH= -6 + 125 * SRH/2^16
}

