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
	_mc_rotors.updateRotorsFromSensors(_angles);
	const bool rotors_added_successfully = _mc_rotors.addActuators(configuration);

	return rotors_added_successfully;
}

bool ActuatorEffectivenessMCFlexible::updateAngles(const matrix::Vector2f* measurements, const int& count)
{
	if (count > NUM_SENSORS_MAX)
	{
		return false;
	}

	for (int i = 0; i < count; ++i)
	{
		_angles[i] = measurements[i];
	}

	return true;
}