#include "PCA9546.hpp"

using namespace time_literals;

PCA9546::PCA9546(const I2CSPIDriverConfig& config)
  : I2C(config), I2CSPIDriver(config), _px4_mag(get_device_id(), config.rotation)
{
  // _tmag5273 = std::make_unique<TMAG5273Small>(config);
  // _tmag5273 = new TMAG5273Small(config);
}

PCA9546::~PCA9546()
{
  // delete _tmag5273;

  perf_free(_reset_perf);
  perf_free(_bad_register_perf);
  perf_free(_bad_transfer_perf);
}

int PCA9546::init()
{
  int ret = I2C::init();

  if (ret != PX4_OK)
  {
    DEVICE_DEBUG("I2C::init failed (%i)", ret);
    return ret;
  }

  return Reset() ? 0 : -1;
}

bool PCA9546::Reset()
{
  ScheduleClear();
  ScheduleNow();
  return true;
}

void PCA9546::print_status()
{
  I2CSPIDriverBase::print_status();

  perf_print_counter(_reset_perf);
  perf_print_counter(_bad_register_perf);
  perf_print_counter(_bad_transfer_perf);
}

int PCA9546::probe()
{
  return PX4_OK;
}

void PCA9546::RunImpl()
{
  [[maybe_unused]] const hrt_abstime now = hrt_absolute_time();
  PX4_DEBUG("impl, state=%i", _state);

  switch (_state)
  {
    case STATE::CONFIGURE:
      // if (_tmag5273->init() == PX4_ERROR)
      if (true)
      {
        PX4_DEBUG("tmag5273 didn't initialize properly");
        ScheduleDelayed(500_ms);
      }
      else
      {
        _state = STATE::MEASURE;
      }
      break;

    case STATE::MEASURE:
      uint8_t select = 0;
      transfer(&select, sizeof(select), nullptr, 0);
      // probe
      // uint16_t id = _tmag5273->getManufacturerID();
      // PX4_DEBUG("TMAG5273 id: %i", id);
      PX4_DEBUG("some stuff");

      ScheduleDelayed(20_ms);
      break;
  }

}

bool PCA9546::Configure()
{
  bool success = true;
  return success;
}
