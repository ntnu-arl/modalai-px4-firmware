#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

using namespace matrix;

class SMPositionControl
{
public:
	SMPositionControl() = default;
	~SMPositionControl() = default;

	// PD
	void setKp(const Matrix3f &K_p) { _K_p = K_p; }
	void setKd(const Matrix3f &K_d) { _K_d = K_d; }
	// SMC
	void setSwitchingGain(const Matrix3f &switching_gain) { _switching_gain = switching_gain; }
	void setLambda(const Matrix3f &lambda) { _lambda = lambda; }
	void setTanhFactor(const float& tanh_factor) { _tanh_factor = tanh_factor; }
	// STSMC

	// INDI
	void setKpIndi(const Matrix3f &K_p) { _K_p_indi = K_p; }
	void setKdIndi(const Matrix3f &K_d) { _K_d_indi = K_d; }
  void setKdIndi(const Matrix3f &K_ff) { _K_ff_indi = K_ff; }
	void setThrustCoeff(const float& thrust_coeff) { _thrust_coeff = thrust_coeff; }
  void setCutoffIndi(const int& frequency) {
    _cutoff_frequency_indi = frequency;
    _lp_filter_accel.set_cutoff_frequency(_sample_frequency, _cutoff_frequency_indi);
    _lp_filter_force.set_cutoff_frequency(_sample_frequency, _cutoff_frequency_indi);
  }
  void setCutoffPos(const int& frequency) {
  void setCutoffindi(const int& frequency) {_lp_filter_accel.set_cutoff_frequency(_sample_frequency, _cutoff_frequency_indi)}
	void setRPMVals (const int& rpm1, const int& rpm2, const int& rpm3, const int& rpm4) {
		_rpm1 = rpm1;
		_rpm2 = rpm2;
		_rpm3 = rpm3;
		_rpm4 = rpm4;
	}

	// general
  void setFilterPos(const int& apply_filter) {_filter_position = apply_filter}
  void setCutoffPos(const int& frequency) {
    _cutoff_frequency_position = frequency;
    _lp_filter_position.set_cutoff_frequency(_sample_frequency, _cutoff_frequency_position);
  }

	void setMass(const float &mass) { _mass = mass; }

	void setYawSetpoint(const float& yaw) { _yaw_setpoint = yaw; }

	void setPositionSetpoint(const Vector3f &position_setpoint) { _position_setpoint = position_setpoint; }

	void setLinearVelocitySetpoint(const Vector3f &linear_velocity_setpoint) { _linear_velocity_setpoint = linear_velocity_setpoint; }

	void setLinearAccelerationSetpoint(const Vector3f &linear_acceleration_setpoint) { _linear_acceleration_setpoint = linear_acceleration_setpoint; }

	void setAttitude(const Quatf &quaternion) { _attitude = Dcmf(quaternion);	}

	void setPosition(const Vector3f &position) { _position = position;	}

	void setLinearVelocity(const Vector3f &linear_velocity) { _linear_velocity = linear_velocity;	}

	void setLinearAcceleration(const Vector3f &linear_acceleration) { _linear_acceleration = linear_acceleration;	}

	Quatf getAttitude() { return Quatf(_attitude); }

	Vector3f getPosition() { return _position; }

  /**
   * Run one control loop cycle calculation
   */
  void updatePD(float& thrust_setpoint, Quatf& quaternion_setpoint) const;
  void updateSM(float& thrust_setpoint, Quatf& quaternion_setpoint) const;
  void updateINDI(float& thrust_setpoint, Quatf& quaternion_setpoint) const;

private:
  Vector3f signum(const Vector3f& input) const {
    // return Vector3f(std::tanh(v[0]), std::tanh(v[1]), std::tanh(v[1]));
		Vector3f result;
		for (auto i=0; i<3; i++) {
			result(i) = std::tanh(_tanh_factor * input(i));
		}

		return result;
  }

	Vector3f calculateAccelerationPD() const;
	Vector3f calculateAccelerationSM() const;
	Vector3f calculateAccelerationINDI() const;

  float calculateThrust(const Vector3f& acceleration) const;

  Dcmf calculateAttitude(const Vector3f& acceleration) const;

  // controller parameters
  float _mass;

  //math::LowPassFilter2p _lp_filter_force[3] {{_sample_frequency, _cutoff_frequency_indi}, {_sample_frequency, _cutoff_frequency_indi}, {_sample_frequency, _cutoff_frequency_indi}};
  // PD
  Matrix3f _K_p;
  Matrix3f _K_d;
  // SMC
  Matrix3f _switching_gain;
  Matrix3f _lambda;
  float _tanh_factor;
  // STSMC

  // INDI
  Matrix3f _K_p_indi;
  Matrix3f _K_d_indi;
  Matrix3f _K_ff_indi;
  float _rpm1;
  float _rpm2;
  float _rpm3;
  float _rpm4;
  float _thrust_coeff;
  const float _sample_frequency = 800.f;
  const float _cutoff_frequency_position = 50.f;
  const float _cutoff_frequency_indi = 50.f;
  math::LowPassFilter2p<float> _lp_filter_accel[3] {{_sample_frequency, _cutoff_frequency_indi}, {_sample_frequency, _cutoff_frequency_indi}, {_sample_frequency, _cutoff_frequency_indi}};	// linear acceleration
  math::LowPassFilter2p<float> _lp_filter_force[3] {{_sample_frequency, _cutoff_frequency_indi}, {_sample_frequency, _cutoff_frequency_indi}, {_sample_frequency, _cutoff_frequency_indi}};


  // general
  int _filter_position;
  math::LowPassFilter2p<float> _lp_filter_position[3] {{_sample_frequency, _cutoff_frequency_position}, {_sample_frequency, _cutoff_frequency_position}, {_sample_frequency, _cutoff_frequency_position}};	// force command


  // setpoints
  float _yaw_setpoint;
  Vector3f _position_setpoint{};
  Vector3f _linear_velocity_setpoint{};
  Vector3f _linear_acceleration_setpoint{};

  // measurements
  Dcmf _attitude{};
  Vector3f _position{};
  Vector3f _linear_velocity{};
  Vector3f _linear_acceleration{};
};
