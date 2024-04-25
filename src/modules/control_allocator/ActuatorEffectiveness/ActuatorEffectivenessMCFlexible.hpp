#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"

#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_angles.h>

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

	struct RegressionParameters {
		float b0;
		float bx;
		float by;
		float bz;
	};
	struct AngleRegression {
		RegressionParameters azimuth;
		RegressionParameters elevation;
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

	bool updateHallEffect(const uint64_t& timestamp, const matrix::Vector3f* measurements, const int& count) override;

	void publishAngles(const uint64_t& timestamp);

protected:
	ActuatorEffectivenessRotors _mc_rotors;

private:
// REVIEW: does tis need params?
	void updateParams() override;

	void applyRegression(const matrix::Vector3f& mag, const RegressionParameters& params, float& angle);

	uORB::Publication<sensor_angles_s>	_sensor_angles_pub{ORB_ID(sensor_angles)};
	
	struct ParamHandles {
		param_t azimuth_0;
		param_t azimuth_x;
		param_t azimuth_y;
		param_t azimuth_z;
		param_t elevation_0;
		param_t elevation_x;
		param_t elevation_y;
		param_t elevation_z;
	};
	ParamHandles _param_handles[NUM_SENSORS_MAX];

	Sensor _geometry{};
	matrix::Vector3f _hall_effect[NUM_SENSORS_MAX]{};
	AngleRegression _angle_params[NUM_SENSORS_MAX]{};
	matrix::Vector2f _angle_measurement[NUM_SENSORS_MAX]{};
};
