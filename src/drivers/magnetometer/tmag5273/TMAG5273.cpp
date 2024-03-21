/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "TMAG5273.hpp"

using namespace time_literals;

// static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
// {
// 	return (msb << 8u) | lsb;
// }

TMAG5273::TMAG5273(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_mag(get_device_id())
{
}

TMAG5273::~TMAG5273()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int TMAG5273::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool TMAG5273::Reset()
{
	ScheduleClear();
	ScheduleNow();
	return true;
}

void TMAG5273::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

int TMAG5273::probe()
{
	if (!isConnected()){
		DEVICE_DEBUG("TMAG5273 not connected");
		return PX4_ERROR;
	}

	return PX4_OK;
}

void TMAG5273::RunImpl()
{
  const hrt_abstime now = hrt_absolute_time();

  switch (_state)
  {
    case STATE::CONFIGURE:
      if (Configure())
      {
        _state = STATE::MEASURE;
        ScheduleDelayed(20_ms);
      }
      else
      {
        // CONFIGURE not complete
        PX4_DEBUG("Configure failed, retrying");
        ScheduleDelayed(100_ms);
      }
      break;

    case STATE::MEASURE:
  		const hrt_abstime tic = hrt_absolute_time();
      const float x = getXData();
      const float y = getYData();
      const float z = getZData();
  		const hrt_abstime toc = hrt_absolute_time();
			_px4_mag.update(now, x, y, z);

			PX4_DEBUG("%llu", toc-tic);
      // PX4_DEBUG("%llu Read from magnetometer: %f %f %f", now, (double)x, (double)y, (double)z);

      // initiate next measurement
      // ScheduleDelayed(20_ms);  // Wait at least 6ms. (minimum waiting time for 16 times internal average setup)
      ScheduleDelayed(5_ms);
      break;
  }
}

bool TMAG5273::Configure()
{
	bool success = true;

	if (!isConnected()){
		return false;	
	}

	// Following the Detailed Design Prodedure on page 42 of the datasheet
    setMagneticChannel(TMAG5273_X_Y_Z_ENABLE);
    setTemperatureEn(true);
    setOperatingMode(TMAG5273_CONTINUOUS_MEASURE_MODE);

    // Set the axis ranges for the device to be the largest
    setXYAxisRange(TMAG5273_RANGE_80MT);
    setZAxisRange(TMAG5273_RANGE_80MT);

    // Check if there is any issue with the device status register
    if (getError() != 0)
    {
        return 0;
    }

    // Check the low active current mode (0)
    if (getLowPower() != TMAG5273_LOW_ACTIVE_CURRENT_MODE)
    {
        return 0;
    }

    // Check the operating mode to make sure it is set to continuous measure (0X2)
    if (getOperatingMode() != TMAG5273_CONTINUOUS_MEASURE_MODE)
    {
        return 0;
    }

    // Check that all magnetic channels have been enables(0X7)
    if (getMagneticChannel() != TMAG5273_X_Y_Z_ENABLE)
    {
        return 0;
    }

    // Check that the temperature data acquisition has been enabled
    if (getTemperatureEn() != TMAG5273_TEMPERATURE_ENABLE)
    {
        return 0;
    }

    // Check that X and Y angle calculation is disabled
    if (getAngleEn() != TMAG5273_NO_ANGLE_CALCULATION)
    {
        return 0;
    }

	// _px4_mag.set_scale(1.f / 1320.f); // 1320 LSB/Gauss
	_px4_mag.set_scale(1.f);

	return success;
}

bool TMAG5273::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint8_t TMAG5273::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	transfer(&cmd, 1, &buffer, 1);
	return buffer;
}

void TMAG5273::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	transfer(buffer, sizeof(buffer), nullptr, 0);
}

void TMAG5273::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

/// @brief Reads the register byte from the sensor when called upon.
/// @param regAddress Register's address to read from
/// @return Value of the register chosen to be read from
uint8_t TMAG5273::readRegister(Register regAddress)
{
    /* uint8_t regVal = 0;
    readRegisters(regAddress, &regVal, 2);
    return regVal; */
		return RegisterRead(regAddress);
}

/// @brief Reads a register region from a device.
/// @param regAddress I2C address of device
/// @param data Value to fill register with
/// @return Error code (0 is success, negative is failure, positive is warning)
uint8_t TMAG5273::writeRegister(Register regAddress, uint8_t data)
{
    /* // Write 1 byte to writeRegisters()
    writeRegisters(regAddress, &data, 1);
    */
		RegisterWrite(regAddress, data);
		return data;
}

/// @brief Returns the 8-Bit Manufacturer ID. There are two
///  registers, LSB and MSB.
///     MANUFACTURER_ID_LSB
///     MANUFACTURER_ID_MSB
/// @return 16-Bit Manufacturer ID
uint16_t TMAG5273::getManufacturerID()
{
    uint16_t deviceIDReg = 0;
    uint8_t databuffer[2];

    databuffer[0] = readRegister(TMAG5273_REG_MANUFACTURER_ID_LSB);
    databuffer[1] = readRegister(TMAG5273_REG_MANUFACTURER_ID_MSB);

    deviceIDReg = (databuffer[1] << 8) | (databuffer[0]);

    return deviceIDReg;
}

/// @brief This function will make sure the TMAG5273 acknowledges
///  over I2C, along with checking the Device ID to ensure proper
///  connection.
/// @return Error code (0 is success, negative is failure)
bool TMAG5273::isConnected()
{
    if (getManufacturerID() != TMAG5273_DEVICE_ID_VALUE)
    {
        return false;
    }

    return true;
}

/// @brief Sets the data aquisition from the following magnetic
///  axis channels listed below
/// @param channelMode Value that sets the channel for data aquisition
///     0X0 = All magnetic channels off, DEFAULT
///     0X1 = X Channel Enabled
///     0X2 = Y Channel Enabled
///     0X3 = X, Y Channel Enabled
///     0X4 = Z Channel Enabled
///     0X5 = Z, X Channel Enabled
///     0X6 = Y, Z Channel Enabled
///     0X7 = X, Y, Z Channel Enabled
///     0X8 = XYX Channel Enabled
///     0X9 = YXY Channel Enabled
///     0XA = YZY Channel Enabled
///     0XB = XZX Channel Enabled
///     TMAG5273_REG_SENSOR_CONFIG_1 - bits 7-4
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setMagneticChannel(uint8_t channelMode)
{
    uint16_t mode = 0;
    mode = readRegister(TMAG5273_REG_SENSOR_CONFIG_1);

    // If-Else statement for writing values to the register, bit by bit
    if (channelMode == 0X0) // 0b0000
    {
        // Writes 0 to bit 4 of the register
        bitWrite(mode, 4, 0);
        // Writes 0 to bit 5 of the register
        bitWrite(mode, 5, 0);
        // Writes 0 to bit 6 of the register
        bitWrite(mode, 6, 0);
        // Writes 0 to bit 7 of the register
        bitWrite(mode, 7, 0);
        // Writes the new register value back into TMAG5273_REG_SENSOR_CONFIG_1
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
        // writeRegisterRegion(TMAG5273_REG_SENSOR_CONFIG_1, mode, 0);
    }
    else if (channelMode == 0X1) // 0x0001
    {
        bitWrite(mode, 4, 1);
        bitWrite(mode, 5, 0);
        bitWrite(mode, 6, 0);
        bitWrite(mode, 7, 0);
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    }
    else if (channelMode == 0X2) // 0x0010
    {
        bitWrite(mode, 4, 0);
        bitWrite(mode, 5, 1);
        bitWrite(mode, 6, 0);
        bitWrite(mode, 7, 0);
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    }
    else if (channelMode == 0X3) // 0x0011
    {
        bitWrite(mode, 4, 1);
        bitWrite(mode, 5, 1);
        bitWrite(mode, 6, 0);
        bitWrite(mode, 7, 0);
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    }
    else if (channelMode == 0X4) // 0x0100
    {
        bitWrite(mode, 4, 0);
        bitWrite(mode, 5, 0);
        bitWrite(mode, 6, 1);
        bitWrite(mode, 7, 0);
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    }
    else if (channelMode == 0X5) // 0x0101
    {
        bitWrite(mode, 4, 1);
        bitWrite(mode, 5, 0);
        bitWrite(mode, 6, 1);
        bitWrite(mode, 7, 0);
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    }
    else if (channelMode == 0X6) // 0x0110
    {
        bitWrite(mode, 4, 0);
        bitWrite(mode, 5, 1);
        bitWrite(mode, 6, 1);
        bitWrite(mode, 7, 0);
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    }
    else if (channelMode == 0X7) // 0x0111
    {
        bitWrite(mode, 4, 1);
        bitWrite(mode, 5, 1);
        bitWrite(mode, 6, 1);
        bitWrite(mode, 7, 0);
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    }
    else if (channelMode == 0X8) // 0x1000
    {
        bitWrite(mode, 4, 0);
        bitWrite(mode, 5, 0);
        bitWrite(mode, 6, 0);
        bitWrite(mode, 7, 1);
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    }
    else if (channelMode == 0X9) // 0x1001
    {
        bitWrite(mode, 4, 1);
        bitWrite(mode, 5, 0);
        bitWrite(mode, 6, 0);
        bitWrite(mode, 7, 1);
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    }
    else if (channelMode == 0XA) // 0x1010
    {
        bitWrite(mode, 4, 0);
        bitWrite(mode, 5, 1);
        bitWrite(mode, 6, 0);
        bitWrite(mode, 7, 1);
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    }
    else if (channelMode == 0XB) // 0x1011
    {
        bitWrite(mode, 4, 1);
        bitWrite(mode, 5, 1);
        bitWrite(mode, 6, 0);
        bitWrite(mode, 7, 1);
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_1, mode);
    }

    return getError();
}

/// @brief Sets the enable bit that determines the data acquisition of the
///  temperature channel.
/// @param temperatureEnable Value to determine enable or disable
///     0x0 = Temp Channel Disabled
///     0x1 = Temp Channel Enabled
///     TMAG5273_REG_T_CONFIG - bit 0
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setTemperatureEn(bool temperatureEnable)
{
    uint16_t mode = 0;
    mode = readRegister(TMAG5273_REG_T_CONFIG);

    // If-Else statement for writing values to the register, bit by bit
    if (temperatureEnable == 0) // 0b0
    {
        // Write 0 to bit 7 of the register value
        bitWrite(mode, 0, 0);
        // Writes mode to the T_CONFIG register
        writeRegister(TMAG5273_REG_T_CONFIG, mode);
    }
    else if (temperatureEnable == 1) // 0b1
    {
        bitWrite(mode, 0, 1);
        writeRegister(TMAG5273_REG_T_CONFIG, mode);
    }

    return getError();
}

/// @brief Sets the operating mode from one of the 4 modes:
///  stand-by mode, sleed mode, continuous measure mode, and
///  wake-up and sleep mode.
/// @param opMode value to set the operating mode of the device
///     0X0 = Stand-by mode (starts new conversion at trigger event)
///     0X1 = Sleep mode
///     0X2 = Continuous measure mode
///     0X3 = Wake-up and sleep mode (W&S Mode)
///     TMAG5273_REG_DEVICE_CONFIG_2 - bit 1-0
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setOperatingMode(uint8_t opMode)
{
    uint16_t mode = 0;
    mode = readRegister(TMAG5273_REG_DEVICE_CONFIG_2);

    // If-Else statement for writing values to the register, bit by bit
    if (opMode == 0) // 0b00
    {
        // Write 0 to bit 0
        bitWrite(mode, 0, 0);
        // Write 0 to bit 1
        bitWrite(mode, 1, 0);
        writeRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    }
    else if (opMode == 0X1) // 0b01
    {
        // Write 1 to bit 0
        bitWrite(mode, 0, 1);
        // Write 0 to bit 1
        bitWrite(mode, 1, 0);
        writeRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    }
    else if (opMode == 0X2) // 0b10
    {
        bitWrite(mode, 0, 0);
        bitWrite(mode, 1, 1);
        writeRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    }
    else if (opMode == 0X3) // 0b11
    {
        bitWrite(mode, 0, 1);
        bitWrite(mode, 1, 1);
        writeRegister(TMAG5273_REG_DEVICE_CONFIG_2, mode);
    }

    return getError();
}

/// @brief Sets the X and Y axes magnetic range from 2 different options
/// @param xyAxisRange Value to choose the magnetic range
///     0X0 = ±40mT, DEFAULT
///     0X1 = ±80mT
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 1
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setXYAxisRange(uint8_t xyAxisRange)
{
    uint16_t mode = 0;
    mode = readRegister(TMAG5273_REG_SENSOR_CONFIG_2);

    // If-Else statement for writing values to the register, bit by bit
    if (xyAxisRange == 0X0) // 0b0
    {
        // Write 0 to bit 1 of the register value
        bitWrite(mode, 1, 0);
        // Writes mode to the CONFIG_2 register
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    }
    else if (xyAxisRange == 0X1) // 0b1
    {
        bitWrite(mode, 1, 1);
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    }

    return getError();
}

/// @brief Sets the Z magnetic range from 2 different options
/// @param zAxisRange Value to set the range from either 40mT or 80mT
///     0X0 = ±40mT, DEFAULT
///     0X1 = ±80mT
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 0
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::setZAxisRange(uint8_t zAxisRange)
{
    uint16_t mode = 0;
    mode = readRegister(TMAG5273_REG_SENSOR_CONFIG_2);

    // If-Else statement for writing values to the register, bit by bit
    if (zAxisRange == 0X0) // 0b0
    {
        // Write 0 to bit 0 of the register value
        bitWrite(mode, 0, 0);
        // Writes mode to the CONFIG_2 register
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    }
    else if (zAxisRange == 0X1) // 0b1
    {
        bitWrite(mode, 0, 1);
        writeRegister(TMAG5273_REG_SENSOR_CONFIG_2, mode);
    }

    return getError();
}

/// @brief This function returns the device status register as its
///  raw hex value. This value can be taken and compared to the main
///  register as seen in the datasheet.
///  The errors include an oscillator error, INT pin error detected,
///  OTP CRC errors, or undervoltage resistors.
///     TMAG5273_REG_DEVICE_STATUS
/// @return Device Status Regsiter as a raw value.
uint8_t TMAG5273::getDeviceStatus()
{
    // Check for undervoltage, OTP CRC, Int Pin, and Oscillator errors
    uint8_t deviceStatusReg = readRegister(TMAG5273_REG_DEVICE_STATUS);

    return deviceStatusReg;
}

/// @brief This function will return the error code, if there is any
///  at the time when the function is called.
///  If any of the Error pins are raised to 1 then there is an error.
///  For more information on the specific error, checkout the
///  getDeviceStatus() function and compare to the datasheet.
/// @return Error code (0 is success, negative is failure, positive is warning)
int8_t TMAG5273::getError()
{
    // Pull in the device status register to compare to the error codes
    uint8_t statusReg = getDeviceStatus();
    uint8_t undervoltageError = bitRead(statusReg, 0);
    uint8_t otpCrcError = bitRead(statusReg, 1);
    uint8_t intPinError = bitRead(statusReg, 2);
    uint8_t oscillatorError = bitRead(statusReg, 3);

    // If there is any error with the error codes, return -1. Otherwise, success and return 0
    if ((undervoltageError != 0) && (otpCrcError != 0) && (intPinError != 0) && (oscillatorError != 0))
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

/// @brief Returns if the device is operating in low power
///  or low noise mode.
///     0X0 = Low active current mode
///     0X1 = Low noise mode
///     TMAG5273_REG_DEVICE_CONFIG_2 - bit 4
/// @return Low power (0) or low noise (1) mode
uint8_t TMAG5273::getLowPower()
{
    uint8_t lowPowerMode = 0;
    lowPowerMode = readRegister(TMAG5273_REG_DEVICE_CONFIG_2);

    uint8_t lowPowerModeBit = bitRead(lowPowerMode, 4);

    return lowPowerModeBit;
}

/// @brief Returns the operating mode from one of the 4 listed below:
///     0X0 = Stand-by mode (starts new conversion at trigger event)
///     0X1 = Sleep mode
///     0X2 = Continuous measure mode
///     0X3 = Wake-up and sleep mode (W&S Mode)
///     TMAG5273_REG_DEVICE_CONFIG_2 - bit 1-0
/// @return Operating mode: stand-by, sleep, continuous, or wake-up and sleep
uint8_t TMAG5273::getOperatingMode()
{
    uint8_t opMode = 0;
    opMode = readRegister(TMAG5273_REG_DEVICE_CONFIG_2);

    uint8_t opMode0 = bitRead(opMode, 0);
    uint8_t opMode1 = bitRead(opMode, 1);

    if ((opMode0 == 0) && (opMode1 == 0)) // 0b00
    {
        // Stand-by mode
        return 0;
    }
    else if ((opMode0 == 1) && (opMode1 == 0)) // 0b01
    {
        // Sleep mode
        return 1;
    }
    else if ((opMode0 == 0) && (opMode1 == 1)) // 0b10
    {
        // Continuous measure mode
        return 2;
    }
    else if ((opMode0 == 1) && (opMode1 == 1)) // 0b11
    {
        // Wake-up and sleep mode (W&S Mode)
        return 3;
    }

    return 0;
}

/// @brief Returns data acquisition from the following magnetic axis channels:
///     0X0 = All magnetic channels off, DEFAULT
///     0X1 = X Channel Enabled
///     0X2 = Y Channel Enabled
///     0X3 = X, Y Channel Enabled
///     0X4 = Z Channel Enabled
///     0X5 = Z, X Channel Enabled
///     0X6 = Y, Z Channel Enabled
///     0X7 = X, Y, Z Channel Enabled
///     0X8 = XYX Channel Enabled
///     0X9 = YXY Channel Enabled
///     0XA = YZY Channel Enabled
///     0XB = XZX Channel Enabled
///     TMAG5273_REG_SENSOR_CONFIG_1 - bit 7-4
/// @return Code for the magnetic channel axis being read
uint8_t TMAG5273::getMagneticChannel()
{
    uint8_t magChannel = 0;
    magChannel = readRegister(TMAG5273_REG_SENSOR_CONFIG_1);

    uint8_t magMode4 = bitRead(magChannel, 4);
    uint8_t magMode5 = bitRead(magChannel, 5);
    uint8_t magMode6 = bitRead(magChannel, 6);
    uint8_t magMode7 = bitRead(magChannel, 7);

    if ((magMode4 == 0) && (magMode5 == 0) && (magMode6 == 0) && (magMode7 == 0)) // 0b0000
    {
        return 0; // All mag channels off
    }
    else if ((magMode4 == 1) && (magMode5 == 0) && (magMode6 == 0) && (magMode7 == 0)) // 0b0001
    {
        return 1; // X channel enabled
    }
    else if ((magMode4 == 0) && (magMode5 == 1) && (magMode6 == 0) && (magMode7 == 0)) // 0b0010
    {
        return 2; // Y channel enabled
    }
    else if ((magMode4 == 1) && (magMode5 == 1) && (magMode6 == 0) && (magMode7 == 0)) // 0b0011
    {
        return 3; // X, Y channel enabled
    }
    else if ((magMode4 == 0) && (magMode5 == 0) && (magMode6 == 1) && (magMode7 == 0)) // 0b0100
    {
        return 4; // Z channel enabled
    }
    else if ((magMode4 == 1) && (magMode5 == 0) && (magMode6 == 1) && (magMode7 == 0)) // 0b0101
    {
        return 5; // Z, X channel enabled
    }
    else if ((magMode4 == 0) && (magMode5 == 1) && (magMode6 == 1) && (magMode7 == 0)) // 0b0110
    {
        return 6; // Y, Z channel enabled
    }
    else if ((magMode4 == 1) && (magMode5 == 1) && (magMode6 == 1) && (magMode7 == 0)) // 0b0111
    {
        return 7; // X, Y, Z channel enabled
    }
    else if ((magMode4 == 0) && (magMode5 == 0) && (magMode6 == 0) && (magMode7 == 1)) // 0b1000
    {
        return 8; // XYX channel enabled
    }
    else if ((magMode4 == 1) && (magMode5 == 0) && (magMode6 == 0) && (magMode7 == 1)) // 0b1001
    {
        return 9; // YXY channel enabled
    }
    else if ((magMode4 == 0) && (magMode5 == 1) && (magMode6 == 0) && (magMode7 == 1)) // 0b1010
    {
        return 10; // YZY channel enabled
    }
    else if ((magMode4 == 1) && (magMode5 == 1) && (magMode6 == 0) && (magMode7 == 1)) // 0b1011
    {
        return 11; // XZX channel enabled
    }
    else
    {
        return 0; // DEFAULT
    }
}

/// @brief Returns the enable bit that determines the data
///  acquisition of the temperature channel.
////    0x0 = Temp Channel Disabled
///     0x1 = Temp Channel Enabled
///     TMAG5273_REG_T_CONFIG - bit 0
/// @return Enable bit that determines if temp channel is enabled or disabled
uint8_t TMAG5273::getTemperatureEn()
{
    uint8_t tempENreg = 0;
    tempENreg = readRegister(TMAG5273_REG_T_CONFIG);

    uint8_t tempEN = bitRead(tempENreg, 0);

    return tempEN;
}

/// @brief Returns angle calculation, magnetic gain, and offset
///  corrections between two selected magnetic channels.
///     0X0 = No angle calculation, magnitude gain, and offset
///           correction enabled
///     0X1 = X 1st, Y 2nd
///     0X2 = Y 1st, Z 2nd
///     0X3 = X 1st, Z 2nd
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 3-2
/// @return Angle calculation and associated channel order
uint8_t TMAG5273::getAngleEn()
{
    uint8_t angleReg = 0;
    angleReg = readRegister(TMAG5273_REG_SENSOR_CONFIG_2);

    uint8_t angleDir2 = bitRead(angleReg, 2);
    uint8_t angleDir3 = bitRead(angleReg, 3);

    if ((angleDir2 == 0) && (angleDir3 == 0)) // 0b00
    {
        // NO angle calculation
        return 0;
    }
    else if ((angleDir2 == 1) && (angleDir3 == 0)) // 0b01
    {
        // X 1st, Y 2nd
        return 1;
    }
    else if ((angleDir2 == 0) && (angleDir3 == 1)) // 0b10
    {
        // Y 1st, Z 2nd
        return 2;
    }
    else if ((angleDir2 == 1) && (angleDir3 == 1)) // 0b11
    {
        // X 1st, Z 2nd
        return 3;
    }

    return 0;
}

/// @brief Readcs back the X-Channel data conversion results, the
/// MSB 8-Bit and LSB 8-Bits. This reads from the following registers:
///     X_MSB_RESULT and X_LSB_RESULT
/// @return X-Channel data conversion results
float TMAG5273::getXData()
{
    int8_t xLSB = readRegister(TMAG5273_REG_X_LSB_RESULT);
    int8_t xMSB = readRegister(TMAG5273_REG_X_MSB_RESULT);

    // Variable to store full X data
    int16_t xData = 0;
    // Combines the two in one register where the MSB is shifted to the correct location
    xData = xLSB + (xMSB << 8);

    // Reads to see if the range is set to 40mT or 80mT
    uint8_t rangeValXY = getXYAxisRange();
    uint8_t range = 0;
    if (rangeValXY == 0)
    {
        range = 40;
    }
    else if (rangeValXY == 1)
    {
        range = 80;
    }

    // 16-bit data format equation
    float div = 32768;
    float xOut = -(range * xData) / div;

    return xOut;
}

/// @brief Reads back the Y-Channel data conversion results, the
///  MSB 8-Bits and LSB 8-Bits. This reads from the following registers:
///     Y_MSB_RESULT and Y_LSB_RESULT
/// @return Y-Channel data conversion results
float TMAG5273::getYData()
{
    int8_t yLSB = 0;
    int8_t yMSB = 0;

    yLSB = readRegister(TMAG5273_REG_Y_LSB_RESULT);
    yMSB = readRegister(TMAG5273_REG_Y_MSB_RESULT);

    // Variable to store full Y data
    int16_t yData = 0;
    // Combines the two in one register where the MSB is shifted to the correct location
    yData = yLSB + (yMSB << 8);

    // Reads to see if the range is set to 40mT or 80mT
    uint8_t rangeValXY = getXYAxisRange();
    uint8_t range = 0;
    if (rangeValXY == 0)
    {
        range = 40;
    }
    else if (rangeValXY == 1)
    {
        range = 80;
    }

    // 16-bit data format equation
    float div = 32768;
    float yOut = (range * yData) / div;

    return yOut;
}

/// @brief Reads back the Z-Channel data conversion results, the
///  MSB 8-Bits and LSB 8-Bits. This reads from the following registers:
///     Z_MSB_RESULT and Z_LSB_RESULT
/// @return Z-Channel data conversion results.
float TMAG5273::getZData()
{
    int8_t zLSB = 0;
    int8_t zMSB = 0;

    zLSB = readRegister(TMAG5273_REG_Z_LSB_RESULT);
    zMSB = readRegister(TMAG5273_REG_Z_MSB_RESULT);

    // Variable to store full X data
    int16_t zData = 0;
    // Combines the two in one register where the MSB is shifted to the correct location
    zData = zLSB + (zMSB << 8); 

    // Reads to see if the range is set to 40mT or 80mT
    uint8_t rangeValZ = getZAxisRange();
    uint8_t range = 0;
    if (rangeValZ == 0)
    {
        range = 40;
    }
    else if (rangeValZ == 1)
    {
        range = 80;
    }

    // div = (2^16) / 2    (as per the datasheet equation 10)
    // 16-bit data format equation
    float div = 32768;
    float zOut = (range * zData) / div;

    return zOut;
}

/// @brief Returns the X and Y axes magnetic range from the
///  two following options:
///     0X0 = ±40mT, DEFAULT
///     0X1 = ±80mT
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 1
/// @return X and Y axes magnetic range (0 or 1)
uint8_t TMAG5273::getXYAxisRange()
{
    uint8_t XYrangeReg = 0;
    XYrangeReg = readRegister(TMAG5273_REG_SENSOR_CONFIG_2);

    uint8_t axisRange = bitRead(XYrangeReg, 1);

    if (axisRange == 0)
    {
        return 0;
    }
    else if (axisRange == 1)
    {
        return 1;
    }

    return 0;
}

/// @brief Returns the Z axis magnetic range from the
///  two following options:
///     0X0 = ±40mT, DEFAULT
///     0X1 = ±80mT
///     TMAG5273_REG_SENSOR_CONFIG_2 - bit 0
/// @return Z axis magnetic range from ±40mT or ±80mT
uint8_t TMAG5273::getZAxisRange()
{
    uint8_t ZrangeReg = 0;
    ZrangeReg = readRegister(TMAG5273_REG_SENSOR_CONFIG_2);

    uint8_t ZaxisRange = bitRead(ZrangeReg, 0);

    if (ZaxisRange == 0)
    {
        return 0;
    }
    else if (ZaxisRange == 1)
    {
        return 1;
    }

    return 0;
}