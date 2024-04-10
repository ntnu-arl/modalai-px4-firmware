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

/**
 * @file TMAG5273Mult.hpp
 *
 * Driver for the TI TMAG5273Mult connected via I2C.
 *
 */

#pragma once

#include "TI_TMAG5273_registers.hpp"
#include "ADA_PCA9546_registers.hpp"
#include "PCA9546.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace TI_TMAG5273;
#define NUMBER_OF_TMAG5273 4

class TMAG5273Mult : public device::I2C, public I2CSPIDriver<TMAG5273Mult>
{
public:
	TMAG5273Mult(const I2CSPIDriverConfig &config);
	~TMAG5273Mult() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	// Sensor Configuration
	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterReadMultiple(Register reg, uint8_t* buffer, uint8_t bytes);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

  // compatibility
	uint8_t readRegister(Register regAddress);
	uint8_t writeRegister(Register regAddress, uint8_t data);
	// porting from Arduino.h
	#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
	#define bitSet(value, bit) ((value) |= (1UL << (bit)))
	#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
	#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
	// porting from SparkFun driver
	uint16_t getManufacturerID();
	bool isConnected();
	int8_t setMagneticChannel(uint8_t channelMode);
	int8_t setTemperatureEn(bool temperatureEnable);
	int8_t setOperatingMode(uint8_t opMode);
	int8_t setXYAxisRange(uint8_t xyAxisRange);
	int8_t setZAxisRange(uint8_t zAxisRange);
	uint8_t getDeviceStatus();
	int8_t getError();
	uint8_t getLowPower();
	uint8_t getOperatingMode();
	uint8_t getMagneticChannel();
	uint8_t getTemperatureEn();
	uint8_t getAngleEn();

	float getXData();
	float getYData();
	float getZData();
  void getXYZData(float* xyz);
  uint8_t getXYAxisRange();
	uint8_t getZAxisRange();

	PX4Magnetometer _px4_mag;
	PCA9546 _mux;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};

	enum class STATE: uint8_t {
		CONFIGURE,
		MEASURE
	} _state{STATE::CONFIGURE};
};
