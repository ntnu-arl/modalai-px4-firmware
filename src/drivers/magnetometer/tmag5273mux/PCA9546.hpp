#pragma once

#include "ADA_PCA9546_registers.hpp"

#include <lib/drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>

class PCA9546 : public device::I2C
{
public:
  PCA9546(const I2CSPIDriverConfig& config, const uint32_t& frequency, const uint8_t& number_of_sensors)
    : I2C(DRV_MAG_DEVTYPE_PCA9546, "pca9546", config.bus, ADA_PCA9546::I2C_ADDRESS_DEFAULT, frequency)
    , _number_of_sensors(number_of_sensors)
  {
  }

  bool select(uint8_t i)
  {
    if (i >= _number_of_sensors)
    {
      PX4_ERR("select for index > _number_of_sensors: %i", _number_of_sensors);
      return false;
    }
		
		const uint8_t msg = 1 << i;
    this->transfer(&msg, 1, nullptr, 0);
    return true;
  }
private:
  int probe() override
  {
    return PX4_OK; // mux has no return
  }

  uint8_t _number_of_sensors;
};