#include "ActuatorEffectivenessMCFlexible.hpp"

ActuatorEffectivenessMCFlexible::ActuatorEffectivenessMCFlexible(ModuleParams* parent)
	: ModuleParams(parent),
	_mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, false, true)
{
	for (int i = 0; i < NUM_SENSORS_MAX; ++i) {
		char buffer[17];
		// sensor position wrt COG
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_PX", i);
		_param_handles[i].position_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_PY", i);
		_param_handles[i].position_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_PZ", i);
		_param_handles[i].position_z = param_find(buffer);
	}

	updateParams();
}

void ActuatorEffectivenessMCFlexible::updateParams()
{
	ModuleParams::updateParams();

	_geometry.num_sensors = NUM_SENSORS_MAX;

	for (int i=0; i<_geometry.num_sensors; ++i){
		matrix::Vector3f &position = _geometry.sensors[i].position;
		param_get(_param_handles[i].position_x, &position(0));
		param_get(_param_handles[i].position_y, &position(1));
		param_get(_param_handles[i].position_z, &position(2));
	}
}

bool
ActuatorEffectivenessMCFlexible::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Motors
	_mc_rotors.updateRotorsFromSensors(_hall_effect);
	const bool rotors_added_successfully = _mc_rotors.addActuators(configuration);

	return rotors_added_successfully;
}

bool ActuatorEffectivenessMCFlexible::updateHallEffect(const matrix::Vector3f* measurements, const int& count)
{
	if (count > NUM_SENSORS_MAX)
	{
		return false;
	}

	static int msg_cnt = 0;
	static Calibration calib_buffer[NUM_SENSORS_MAX];
	static bool center_calib_set = false;
	if (msg_cnt < _calibration_count) {
		for (int i=0; i<count; ++i){
			calib_buffer[i].center += measurements[i];
		}
	} else {
		if (!center_calib_set) {
			for (int i=0; i<count; ++i){
				_calib[i].center = calib_buffer[i].center / _calibration_count;
				_calib[i].center(2) = 0.0f; // setting z to 0, assuming magnet in nominal configuration points along z
			}
			center_calib_set = true;
		}
	}
	msg_cnt++;

  for (int i = 0; i < count; ++i)
  {
		_hall_effect[i] = measurements[i] - _calib[i].center;
  }
	// PX4_INFO("raw: %f %f %f", (double)measurements[0](0), (double)measurements[0](1), (double)measurements[0](2));
	// PX4_INFO("center buffer: %f %f %f", (double)calib_buffer[0].center(0), (double)calib_buffer[0].center(1), (double)calib_buffer[0].center(2));
	// PX4_INFO("center: %f %f %f", (double)_calib[0].center(0), (double)_calib[0].center(1), (double)_calib[0].center(2));
	// PX4_INFO("result: %f %f %f", (double)_hall_effect[0](0), (double)_hall_effect[0](1), (double)_hall_effect[0](2));

	return true;
}
