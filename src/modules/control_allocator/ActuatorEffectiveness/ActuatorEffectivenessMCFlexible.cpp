#include "ActuatorEffectivenessMCFlexible.hpp"

ActuatorEffectivenessMCFlexible::ActuatorEffectivenessMCFlexible(ModuleParams* parent)
	: ModuleParams(parent),
	_mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, false, true)
{
	for (int i = 0; i < NUM_SENSORS_MAX; ++i) {
		char buffer[17];
		// sensor position wrt COG
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_MAXX", i);
		_param_handles[i].cal_max_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_MAXY", i);
		_param_handles[i].cal_max_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_MAXZ", i);
		_param_handles[i].cal_max_z = param_find(buffer);
	}

	updateParams();
}

void ActuatorEffectivenessMCFlexible::updateParams()
{
	ModuleParams::updateParams();

	_geometry.num_sensors = NUM_SENSORS_MAX;

	for (int i=0; i<_geometry.num_sensors; ++i){
		// matrix::Vector3f &position = _geometry.sensors[i].position;
		// param_get(_param_handles[i].position_x, &position(0));
		// param_get(_param_handles[i].position_y, &position(1));
		// param_get(_param_handles[i].position_z, &position(2));

		matrix::Vector3f &max_val = _calib[i].max_val;
		param_get(_param_handles[i].cal_max_x, &max_val(0));
		param_get(_param_handles[i].cal_max_y, &max_val(1));
		param_get(_param_handles[i].cal_max_z, &max_val(2));
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
		// _hall_effect[i] = measurements[i] - _calib[i].center;
		for(int j=0; j<3;j++)
		{
			_hall_effect[i](j) = (measurements[i](j) - _calib[i].center(j))/(_calib[i].center(j) - _calib[i].max_val(j) );
			// clip for safety
			if(_hall_effect[i](j) >= 1.0f )
				_hall_effect[i](j) = 1.0f;
			if(_hall_effect[i](j) <= -1.0f )
				_hall_effect[i](j) = -1.0f;

		}
		// PX4_INFO("%d %f %f %f ", i, (double)_hall_effect[i](0), (double)_hall_effect[i](1), (double)_hall_effect[i](2)   );
	}

	sensor_mag_mux_calib_s report;
	report.timestamp = hrt_absolute_time();
	for (int i = 0; i < count; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			report.mags[i].xyz[j] = _hall_effect[i](j);
			report.mags[i].center[j] = _calib[i].center(j);
			report.mags[i].max[j] = _calib[i].max_val(j);
		}
	}
	_sensor_mag_mux_calib_pub.publish(report);

	return true;
}
