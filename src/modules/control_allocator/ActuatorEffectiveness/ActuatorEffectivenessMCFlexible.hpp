#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"

class ActuatorEffectivenessMCFlexible : public ModuleParams, public ActuatorEffectiveness
{
public:
	static constexpr int NUM_SENSORS_MAX = 4;

	struct SensorGeometry {
		matrix::Vector3f position;
	};

	struct Sensor {
		SensorGeometry sensors[NUM_SENSORS_MAX];
		int num_sensors{0};
  };

	struct Calibration {
		matrix::Vector3f center;
		matrix::Vector3f max_val;
	};

  ActuatorEffectivenessMCFlexible(ModuleParams* parent);
  virtual ~ActuatorEffectivenessMCFlexible() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	const char *name() const override { return "MC Flexible"; }

	bool updateHallEffect(const matrix::Vector3f* measurements, const int& count) override;

protected:
	ActuatorEffectivenessRotors _mc_rotors;

private:
// REVIEW: does tis need params?
	void updateParams() override;

	struct ParamHandles {
		param_t position_x;
		param_t position_y;
		param_t position_z;
		param_t cal_max_x;
		param_t cal_max_y;
		param_t cal_max_z;
	};
	ParamHandles _param_handles[NUM_SENSORS_MAX];

	Sensor _geometry{};
  matrix::Vector3f _hall_effect[NUM_SENSORS_MAX]{};

	Calibration _calib[NUM_SENSORS_MAX]{};
	const int _calibration_count {100}; // TODO: make param
};
