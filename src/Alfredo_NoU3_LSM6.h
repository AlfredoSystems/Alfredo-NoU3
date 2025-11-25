/*
  This file is part of the Arduino_LSM6DSOX library.
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

#ifndef ALFREDO_NOU3_LSM6_H
#define ALFREDO_NOU3_LSM6_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define LSM6DSOX_ADDRESS            0x6B

#define LSM6DSOX_INT1_CTRL          0X0D
#define LSM6DSOX_WHO_AM_I_REG       0X0F
#define LSM6DSOX_CTRL1_A            0X10
#define LSM6DSOX_CTRL2_G            0X11

#define LSM6DSOX_CTRL6_C            0X15
#define LSM6DSOX_CTRL7_G            0X16
#define LSM6DSOX_CTRL8_A            0X17

#define LSM6DSOX_STATUS_REG         0X1E

#define LSM6DSOX_OUT_TEMP_L         0X20
#define LSM6DSOX_OUT_TEMP_H         0X21

#define LSM6DSOX_OUTX_L_G           0X22
#define LSM6DSOX_OUTX_H_G           0X23
#define LSM6DSOX_OUTY_L_G           0X24
#define LSM6DSOX_OUTY_H_G           0X25
#define LSM6DSOX_OUTZ_L_G           0X26
#define LSM6DSOX_OUTZ_H_G           0X27

#define LSM6DSOX_OUTX_L_A           0X28
#define LSM6DSOX_OUTX_H_A           0X29
#define LSM6DSOX_OUTY_L_A           0X2A
#define LSM6DSOX_OUTY_H_A           0X2B
#define LSM6DSOX_OUTZ_L_A           0X2C
#define LSM6DSOX_OUTZ_H_A           0X2D

//------------- LSM6DSD --------------//

#define LSM6DSD_ADDRESS            0x6A

#define LSM6DSD_FUNC_CFG_ACCESS        0x01
#define LSM6DSD_SENSOR_SYNC_TIME_FRAME 0x04
#define LSM6DSD_SENSOR_SYNC_RES_RATIO  0x05

#define LSM6DSD_FIFO_CTRL1             0x06
#define LSM6DSD_FIFO_CTRL2             0x07
#define LSM6DSD_FIFO_CTRL3             0x08
#define LSM6DSD_FIFO_CTRL4             0x09
#define LSM6DSD_FIFO_CTRL5             0x0A

#define LSM6DSD_DRDY_PULSE_CFG_G       0x0B

#define LSM6DSD_INT1_CTRL              0x0D
#define LSM6DSD_INT2_CTRL              0x0E

#define LSM6DSD_WHO_AM_I               0x0F

#define LSM6DSD_CTRL1_XL               0x10
#define LSM6DSD_CTRL2_G                0x11
#define LSM6DSD_CTRL3_C                0x12
#define LSM6DSD_CTRL4_C                0x13
#define LSM6DSD_CTRL5_C                0x14
#define LSM6DSD_CTRL6_C                0x15
#define LSM6DSD_CTRL7_G                0x16
#define LSM6DSD_CTRL8_XL               0x17
#define LSM6DSD_CTRL9_XL               0x18
#define LSM6DSD_CTRL10_C               0x19

#define LSM6DSD_MASTER_CONFIG          0x1A
#define LSM6DSD_WAKE_UP_SRC            0x1B
#define LSM6DSD_TAP_SRC                0x1C
#define LSM6DSD_D6D_SRC                0x1D
#define LSM6DSD_STATUS_REG             0x1E

#define LSM6DSD_OUT_TEMP_L             0x20
#define LSM6DSD_OUT_TEMP_H             0x21

#define LSM6DSD_OUTX_L_G               0x22
#define LSM6DSD_OUTX_H_G               0x23
#define LSM6DSD_OUTY_L_G               0x24
#define LSM6DSD_OUTY_H_G               0x25
#define LSM6DSD_OUTZ_L_G               0x26
#define LSM6DSD_OUTZ_H_G               0x27

#define LSM6DSD_OUTX_L_XL              0x28
#define LSM6DSD_OUTX_H_XL              0x29
#define LSM6DSD_OUTY_L_XL              0x2A
#define LSM6DSD_OUTY_H_XL              0x2B
#define LSM6DSD_OUTZ_L_XL              0x2C
#define LSM6DSD_OUTZ_H_XL              0x2D


class LSM6Class {
  public:
    LSM6Class();
    ~LSM6Class();

    int begin(TwoWire& wire);

    // Accelerometer
    int readAcceleration(float *x, float *y, float *z); // Results are in g (earth gravity).
    float accelerationSampleRate(); // Sampling rate of the sensor.
    int accelerationAvailable(); // Check for available data from accelerometer

    // Gyroscope
    int readGyroscope(float *x, float *y, float *z); // Results are in degrees/second.
    float gyroscopeSampleRate(); // Sampling rate of the sensor.
    int gyroscopeAvailable(); // Check for available data from gyroscope

    // Temperature
    int readTemperature(int& temperature_deg);
    int readTemperatureFloat(float& temperature_deg);
    int temperatureAvailable();
	
	// Interrupt
	void enableInterrupt();

  private:
    int readRegister(uint8_t address);
    int readRegisters(uint8_t address, uint8_t* data, size_t length);
    int writeRegister(uint8_t address, uint8_t value);

    TwoWire* _wire = nullptr;
    uint8_t _slaveAddress;
    int _irqPin;
};

#endif