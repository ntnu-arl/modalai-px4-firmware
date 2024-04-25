#include "ActuatorEffectivenessMCFlexible.hpp"

ActuatorEffectivenessMCFlexible::ActuatorEffectivenessMCFlexible(ModuleParams* parent)
	: ModuleParams(parent),
	_mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, false, true)
{
	for (int i = 0; i < NUM_SENSORS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_AZ_0", i);
		_param_handles[i].azimuth_0 = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_AZ_X", i);
		_param_handles[i].azimuth_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_AZ_Y", i);
		_param_handles[i].azimuth_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_AZ_Z", i);
		_param_handles[i].azimuth_z = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_EL_0", i);
		_param_handles[i].elevation_0 = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_EL_X", i);
		_param_handles[i].elevation_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_EL_Y", i);
		_param_handles[i].elevation_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_EL_Z", i);
		_param_handles[i].elevation_z = param_find(buffer);
	}

	updateParams();
}

void ActuatorEffectivenessMCFlexible::updateParams()
{
	ModuleParams::updateParams();

	_geometry.num_sensors = NUM_SENSORS_MAX;

	for (int i = 0; i < NUM_SENSORS_MAX; ++i)
	{
		RegressionParameters &azimuth = _angle_params[i].azimuth;
		param_get(_param_handles[i].azimuth_0, &azimuth.b0);
		param_get(_param_handles[i].azimuth_x, &azimuth.bx);
		param_get(_param_handles[i].azimuth_y, &azimuth.by);
		param_get(_param_handles[i].azimuth_z, &azimuth.bz);

		RegressionParameters &elevation = _angle_params[i].elevation;
		param_get(_param_handles[i].elevation_0, &elevation.b0);
		param_get(_param_handles[i].elevation_x, &elevation.bx);
		param_get(_param_handles[i].elevation_y, &elevation.by);
		param_get(_param_handles[i].elevation_z, &elevation.bz);
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

  for (int i = 0; i < count; ++i)
  {
    for (int j = 0; j < 3; j++)
    {
			// store data
      _hall_effect[i](j) = measurements[i](j);
    }
		// apply model
		applyRegression(_hall_effect[i], _angle_params[i].azimuth, _angle_measurement[i].azimuth);
		applyRegression(_hall_effect[i], _angle_params[i].elevation, _angle_measurement[i].elevation);
  }

	publishAngles();

  return true;
}

void ActuatorEffectivenessMCFlexible::applyRegression(const matrix::Vector3f& mag, const RegressionParameters& params, float& angle)
{
	// assuming angle = constant + bx*x + by*y + bz*z
  angle = params.b0 + params.bx * mag(0) + params.by * mag(1) + params.bz * mag(2);
}

void ActuatorEffectivenessMCFlexible::publishAngles()
{
	sensor_angles_s report;
	report.timestamp = hrt_absolute_time();
	for (int i = 0; i < NUM_SENSORS_MAX; ++i)
	{
		report.azimuth[i] = _angle_measurement[i].azimuth;
		report.elevation[i] = _angle_measurement[i].elevation;
	}
	_sensor_angles_pub.publish(report);
}