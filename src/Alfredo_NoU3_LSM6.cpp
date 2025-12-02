/*
  This file is part of the Arduino_LSM6DS library.
  Copyright (c) 2021 Arduino SA. All rights reserved.

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
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Alfredo_NoU3_LSM6.h"

LSM6Class::LSM6Class()
{
}

LSM6Class::~LSM6Class()
{
}

int LSM6Class::begin(TwoWire &wire)
{
  _wire = &wire;

  // TODO: Supporting these chips will be easy to do: DSR, DSV, DSL, DS3TR-C

  //LSM6DSOX, LSM6DSOW
  _slaveAddress = 0x6B;
  if (readRegister(LSM6DS_WHO_AM_I_REG) == 0x6C)
  {
    //Serial.println("LSM6DSOW/LSM6DSOX Detected");
    setupLSM6();
    return 1;
  }

  //LSM6DS3
  _slaveAddress = 0x6A;
  if (readRegister(LSM6DS_WHO_AM_I_REG) == 0x69)
  {
    //Serial.println("LSM6DS3 Detected");
    setupLSM6();
    return 1;
  }

  //LSM6DSD
  _slaveAddress = 0x6A;
  if (readRegister(LSM6DS_WHO_AM_I_REG) == 0x6A)
  {
    //Serial.println("LSM6DSD Detected");
    setupLSM6();
    return 1;
  }
  

  return 0;
}

int LSM6Class::setupLSM6(void)
{
  // disable I3C interface to prevent bus glitches
  writeRegister(LSM6DS_CTRL9_XL, 0xE2); //default 0xE0

  // enable BDU to prevent data corruption during reads
  writeRegister(LSM6DS_CTRL9_XL, 0x44); //default 0x40

  // set the gyroscope control register to work at 104 Hz, 2000 dps and in bypass mode
  // writeRegister(LSM6DS_CTRL2_G, 0x4C); //0100 1100
  // set the gyroscope control register to work at 26 Hz, 2000 dps and in bypass mode
  // writeRegister(LSM6DS_CTRL2_G, 0x2C); //0010 1100
  // set the gyroscope control register to work at 104 Hz, 500 dps and in bypass mode
  writeRegister(LSM6DS_CTRL2_G, 0x44); // 0100 0100

  // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass mode and enable ODR/4
  // low pass filter (check figure9 of LSM6DS's datasheet)
  writeRegister(LSM6DS_CTRL1_A, 0x4A);

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  writeRegister(LSM6DS_CTRL7_G, 0x00); // 0000 0000

  // Set the ODR config register to ODR/4
  writeRegister(LSM6DS_CTRL8_A, 0x09);

  return 1;
}

int LSM6Class::readAcceleration(float *x, float *y, float *z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DS_OUTX_L_A, (uint8_t *)data, sizeof(data)))
  {
    *x = NAN;
    *y = NAN;
    *z = NAN;

    return 0;
  }

  // REORDERED FOR NOU3 CONVENTION
  *x = 1 * data[1] * 4.0 / 32768.0;
  *y = -1 * data[0] * 4.0 / 32768.0;
  *z = 1 * data[2] * 4.0 / 32768.0;

  return 1;
}

int LSM6Class::accelerationAvailable()
{
  if (readRegister(LSM6DS_STATUS_REG) & 0x01)
  {
    return 1;
  }

  return 0;
}

float LSM6Class::accelerationSampleRate()
{
  return 104.0F;
}

int LSM6Class::readGyroscope(float *x, float *y, float *z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DS_OUTX_L_G, (uint8_t *)data, sizeof(data)))
  {
    *x = NAN;
    *y = NAN;
    *z = NAN;

    return 0;
  }

  // REORDERED FOR NOU3 CONVENTION
  *x = 1 * float(data[1]) * (PI / 180.0) * 500.0 / 32768.0;
  *y = -1 * float(data[0]) * (PI / 180.0) * 500.0 / 32768.0;
  *z = 1 * float(data[2]) * (PI / 180.0) * 500.0 / 32768.0;

  return 1;
}

int LSM6Class::gyroscopeAvailable()
{
  if (readRegister(LSM6DS_STATUS_REG) & 0x02)
  {
    return 1;
  }

  return 0;
}

int LSM6Class::readTemperature(int &temperature_deg)
{
  float temperature_float = 0;
  readTemperatureFloat(temperature_float);

  temperature_deg = static_cast<int>(temperature_float);

  return 1;
}

int LSM6Class::readTemperatureFloat(float &temperature_deg)
{
  /* Read the raw temperature from the sensor. */
  int16_t temperature_raw = 0;

  if (readRegisters(LSM6DS_OUT_TEMP_L, reinterpret_cast<uint8_t *>(&temperature_raw), sizeof(temperature_raw)) != 1)
  {
    return 0;
  }

  /* Convert to °C. */
  static int const TEMPERATURE_LSB_per_DEG = 256;
  static int const TEMPERATURE_OFFSET_DEG = 25;

  temperature_deg = (static_cast<float>(temperature_raw) / TEMPERATURE_LSB_per_DEG) + TEMPERATURE_OFFSET_DEG;

  return 1;
}

int LSM6Class::temperatureAvailable()
{
  if (readRegister(LSM6DS_STATUS_REG) & 0x04)
  {
    return 1;
  }

  return 0;
}

float LSM6Class::gyroscopeSampleRate()
{
  return 104.0F;
}

void LSM6Class::enableInterrupt()
{
  writeRegister(LSM6DS_INT1_CTRL, 0x03); // 0000 0011
}

int LSM6Class::readRegister(uint8_t address)
{

  if (_slaveAddress == -1)
  {
    return -1;
  }

  uint8_t value;

  if (readRegisters(address, &value, sizeof(value)) != 1)
  {
    return -1;
  }

  return value;
}

int LSM6Class::readRegisters(uint8_t address, uint8_t *data, size_t length)
{
  if (_slaveAddress == -1)
  {
    return -1;
  }

  _wire->beginTransmission(_slaveAddress);
  _wire->write(address);

  if (_wire->endTransmission(false) != 0)
  {
    return -1;
  }

  if (_wire->requestFrom(_slaveAddress, length) != length)
  {
    return 0;
  }

  for (size_t i = 0; i < length; i++)
  {
    *data++ = _wire->read();
  }

  return 1;
}

int LSM6Class::writeRegister(uint8_t address, uint8_t value)
{
  if (_slaveAddress == -1)
  {
    return -1;
  }

  _wire->beginTransmission(_slaveAddress);
  _wire->write(address);
  _wire->write(value);
  if (_wire->endTransmission() != 0)
  {
    return 0;
  }

  return 1;
}