/*
  This file is part of the Arduino_LSM6DS3 library.
  Copyright (c) 2019 Arduino SA. All rights reserved.
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

#include "LSM6DS3.h"

#define LSM6DS3_ADDRESS            0x6A

#define LSM6DS3_WHO_AM_I_REG       0X0F
#define LSM6DS3_CTRL1_XL           0X10
#define LSM6DS3_CTRL2_G            0X11

#define LSM6DS3_STATUS_REG         0X1E

#define LSM6DS3_CTRL6_C            0X15
#define LSM6DS3_CTRL7_G            0X16
#define LSM6DS3_CTRL8_XL           0X17

#define LSM6DS3_OUT_TEMP_L         0X20

#define LSM6DS3_OUTX_L_G           0X22
#define LSM6DS3_OUTX_H_G           0X23
#define LSM6DS3_OUTY_L_G           0X24
#define LSM6DS3_OUTY_H_G           0X25
#define LSM6DS3_OUTZ_L_G           0X26
#define LSM6DS3_OUTZ_H_G           0X27

#define LSM6DS3_OUTX_L_XL          0X28
#define LSM6DS3_OUTX_H_XL          0X29
#define LSM6DS3_OUTY_L_XL          0X2A
#define LSM6DS3_OUTY_H_XL          0X2B
#define LSM6DS3_OUTZ_L_XL          0X2C
#define LSM6DS3_OUTZ_H_XL          0X2D

#define LSM6DS3_WAKE_UP_SRC		   0x1B

#ifndef IMU_FS
	#define IMU_FS 104
#endif


LSM6DS3Class::LSM6DS3Class(TwoWire& wire, uint8_t slaveAddress) :
  _wire(&wire),
  _spi(NULL),
  _slaveAddress(slaveAddress)
{
}

LSM6DS3Class::LSM6DS3Class(SPIClass& spi, int csPin, int irqPin) :
  _wire(NULL),
  _spi(&spi),
  _csPin(csPin),
  _irqPin(irqPin),
  _spiSettings(10E6, MSBFIRST, SPI_MODE0)
{
}

LSM6DS3Class::~LSM6DS3Class()
{
}

int LSM6DS3Class::begin()
{
	return begin(104);
}


int LSM6DS3Class::begin(int fs)
{
  _fs = fs;
  if (_spi != NULL) {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi->begin();
  } else {
    _wire->begin();
  }

  if (readRegister(LSM6DS3_WHO_AM_I_REG) != 0x69) {
    end();
    return 0;
  }

  //set the gyroscope control register to work at 104 Hz, 2000 dps and in bypass mode
  writeRegister(LSM6DS3_CTRL2_G, 0x2C); // Configurado a 26 Hz y 2000dps

  // Set the Accelerometer control register to work at 104 Hz, 4G,and in bypass mode and enable ODR/4
  // low pass filter(check figure9 of LSM6DS3's datasheet)
  switch(_fs)
  {
	  case 208:
		writeRegister(LSM6DS3_CTRL1_XL, 0x5B);
		break;
	  case 104:
	    writeRegister(LSM6DS3_CTRL1_XL, 0x4B);
		break;
	  case 26:
	    writeRegister(LSM6DS3_CTRL1_XL, 0x2B);
		break;
  }

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  writeRegister(LSM6DS3_CTRL7_G, 0x00);

  // Set the ODR config register to ODR/4
  writeRegister(LSM6DS3_CTRL8_XL, 0x01);

  return 1;
}

void LSM6DS3Class::end()
{
  if (_spi != NULL) {
    _spi->end();
    digitalWrite(_csPin, LOW);
    pinMode(_csPin, INPUT);
  } else {
    writeRegister(LSM6DS3_CTRL2_G, 0x00);
    writeRegister(LSM6DS3_CTRL1_XL, 0x00);
    _wire->end();
  }
}

int LSM6DS3Class::readAcceleration(int16_t& x, int16_t& y, int16_t& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DS3_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0];// *4.0 / 32768.0;
  y = data[1];// *4.0 / 32768.0;
  z = data[2];// *4.0 / 32768.0;

  return 1;
}

int LSM6DS3Class::freeFall()
{
  uint8_t data;

  if (!readRegisters(LSM6DS3_WAKE_UP_SRC, &data, sizeof(data))) {


    return 0;
  }

  return data&0x20;
}

int LSM6DS3Class::accelerationAvailable()
{
    return readRegister(LSM6DS3_STATUS_REG) & 0x01;
  /*if (readRegister(LSM6DS3_STATUS_REG) & 0x01) {
    return 1;
  }

  return 0;*/
}

float LSM6DS3Class::accelerationSampleRate()
{
  return (float)_fs;
}

int LSM6DS3Class::readGyroscope(int16_t& x, int16_t& y, int16_t& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DS3_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0];// *2000.0 / 32768.0;
  y = data[1];// * 2000.0 / 32768.0;
  z = data[2];// * 2000.0 / 32768.0;

  return 1;
}

int LSM6DS3Class::gyroscopeAvailable()
{
    return readRegister(LSM6DS3_STATUS_REG) & 0x02;
  /*if (readRegister(LSM6DS3_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;*/
}

float LSM6DS3Class::gyroscopeSampleRate()
{
  return 52.0F;
}

int LSM6DS3Class::readTemperature(int16_t& t)
{
  int16_t data[1];

  if (!readRegisters(LSM6DS3_OUT_TEMP_L, (uint8_t*)data, sizeof(data))) {
    t = NAN;

    return 0;
  }

  t = data[0];// / 16.0 + 25;

  return 1;
}

int LSM6DS3Class::temperatureAvailable()
{
    return readRegister(LSM6DS3_STATUS_REG) & 0x04;
  /*if (readRegister(LSM6DS3_STATUS_REG) & 0x04) {
    return 1;
  }

  return 0;*/
}

float LSM6DS3Class::temperatureSampleRate()
{
  return 52.0F;
}

int LSM6DS3Class::readRegister(uint8_t address)
{
  uint8_t value;

  if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
  }

  return value;
}

int LSM6DS3Class::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
  if (_spi != NULL) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(0x80 | address);
    _spi->transfer(data, length);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
  } else {
    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);

    if (_wire->endTransmission(false) != 0) {
      return -1;
    }

    if (_wire->requestFrom(_slaveAddress, length) != length) {
      return 0;
    }

    for (size_t i = 0; i < length; i++) {
      *data++ = _wire->read();
    }
  }
  return 1;
}

int LSM6DS3Class::writeRegister(uint8_t address, uint8_t value)
{
  if (_spi != NULL) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(address);
    _spi->transfer(value);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
  } else {
    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);
    _wire->write(value);
    if (_wire->endTransmission() != 0) {
      return 0;
    }
  }
  return 1;
}

#ifdef ARDUINO_AVR_UNO_WIFI_REV2
LSM6DS3Class IMU(SPI, SPIIMU_SS, SPIIMU_INT);
#else
LSM6DS3Class IMU(Wire, LSM6DS3_ADDRESS);
#endif