/**
 * @file PCA9546.hpp
 *
 * Driver for the PCA9546 connected via I2C.
 *
 */

#pragma once

#include <memory>

#include "ADA_PCA9546_registers.hpp"
#include "TI_TMAG5273_registers.hpp"
// #include "TMAG5273.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace TI_TMAG5273;

class TMAG5273Small : public device::I2C
{
public:
  TMAG5273Small(const I2CSPIDriverConfig& config) : I2C(config)
  {
  }
  ~TMAG5273Small() override;

  int init() override
  {
    int ret = I2C::init();

    return ret;
  }

  uint16_t getManufacturerID()
  {
    uint16_t deviceIDReg = 0;
    uint8_t databuffer[2];

    databuffer[0] = RegisterRead(TMAG5273_REG_MANUFACTURER_ID_LSB);
    databuffer[1] = RegisterRead(TMAG5273_REG_MANUFACTURER_ID_MSB);

    deviceIDReg = (databuffer[1] << 8) | (databuffer[0]);

    return deviceIDReg;
  }

private:
  int probe() override
  {
    if (getManufacturerID() != TMAG5273_DEVICE_ID_VALUE)
    {
      DEVICE_DEBUG("TMAG5273 not connected");
      return PX4_ERROR;
    }

    return PX4_OK;
  }

  uint8_t RegisterRead(Register reg)
  {
    const uint8_t cmd = static_cast<uint8_t>(reg);
    uint8_t buffer{};
    transfer(&cmd, 1, &buffer, 1);
    return buffer;
  }
};

class PCA9546 : public device::I2C, public I2CSPIDriver<PCA9546>
{
public:
  PCA9546(const I2CSPIDriverConfig& config);
  ~PCA9546() override;

  static void print_usage();
  void RunImpl();
  int init() override;
  void print_status() override;

private:
  int probe() override;
  bool Reset();
  bool Configure();

  // std::unique_ptr<TMAG5273Small> _tmag5273;
  // TMAG5273Small* _tmag5273 = nullptr;
  PX4Magnetometer _px4_mag;

  perf_counter_t _bad_register_perf{ perf_alloc(PC_COUNT, MODULE_NAME ": bad register") };
  perf_counter_t _bad_transfer_perf{ perf_alloc(PC_COUNT, MODULE_NAME ": bad transfer") };
  perf_counter_t _reset_perf{ perf_alloc(PC_COUNT, MODULE_NAME ": reset") };

  hrt_abstime _reset_timestamp{ 0 };
  hrt_abstime _last_config_check_timestamp{ 0 };
  int _failure_count{ 0 };

  enum class STATE : uint8_t
  {
    CONFIGURE,
    MEASURE
  } _state{ STATE::CONFIGURE };
};