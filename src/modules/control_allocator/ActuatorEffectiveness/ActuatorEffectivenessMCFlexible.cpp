#include "ActuatorEffectivenessMCFlexible.hpp"

ActuatorEffectivenessMCFlexible::ActuatorEffectivenessMCFlexible(ModuleParams* parent)
	: ModuleParams(parent),
	_mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, false, true)
{
	updateParams();
}

void ActuatorEffectivenessMCFlexible::updateParams()
{
	ModuleParams::updateParams();

	_geometry.num_sensors = NUM_SENSORS_MAX;
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
		// _hall_effect[i] = measurements[i] - _calib[i].center;
		for(int j=0; j<3;j++)
		{
			// _hall_effect[i](j) = (measurements[i](j) - _calib[i].center(j))/(_calib[i].center(j) - _calib[i].max_val(j) );
			_hall_effect[i](j) = measurements[i](j);

		}
		// PX4_INFO("%d %f %f %f ", i, (double)_hall_effect[i](0), (double)_hall_effect[i](1), (double)_hall_effect[i](2)   );
	}

	return true;
}
