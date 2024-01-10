#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

class SMAttitudeControl
{
public:
	SMAttitudeControl() = default;
	~SMAttitudeControl() = default;

	void setSwitchingGain(const matrix::Matrix3f &switching_gain) { _switching_gain = switching_gain; }

	void setLambda(const matrix::Matrix3f &lambda) { _lambda = lambda; }

	void setInertia(const matrix::Matrix3f &inertia) { _inertia = inertia; }

	void setTanhFactor(const float& tanh_factor) { _tanh_factor = tanh_factor; }

	/**
	 * Set hard limit for output rate setpoints
	 * @param rate_limit [rad/s] 3D vector containing limits for roll, pitch, yaw
	 */
	void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param qd desired vehicle attitude setpoint
	 */
	void setAttitudeSetpoint(const matrix::Quatf &quaternion_setpoint)
	{
		_attitude_setpoint = matrix::Dcmf(quaternion_setpoint);
	}

	void setAngularVelocitySetpoint(const matrix::Vector3f &angular_velocity_setpoint) { _angular_velocity_setpoint = angular_velocity_setpoint; }

	void setAngularAccelerationSetpoint(const matrix::Vector3f &angular_acceleration_setpoint) { _angular_acceleration_setpoint = angular_acceleration_setpoint; }

	void setAttitude(const matrix::Quatf &quaternion) { _attitude = matrix::Dcmf(quaternion); }

	void setAngularVelocity(const matrix::Vector3f &angular_velocity) {_angular_velocity = angular_velocity; }

	void setAngularAcceleration(const matrix::Vector3f &angular_acceleration) { _angular_acceleration = angular_acceleration; }

	/**
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * @param q_delta delta rotation to apply
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
	{
		_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
		_attitude_setpoint_q.normalize();
	}

	/**
	 * Run one control loop cycle calculation
	 * @param q estimation of the current vehicle attitude unit quaternion
	 * @return [rad/s] body frame 3D angular rate setpoint vector to be executed by the rate controller
	 */
	matrix::Vector3f update() const;

private:
  matrix::Vector3f signum(const matrix::Vector3f& input) const {
    // return matrix::Vector3f(std::tanh(v[0]), std::tanh(v[1]), std::tanh(v[1]));
		matrix::Vector3f result;
		for (auto i=0; i<3; i++) {
			result(i) = std::tanh(_tanh_factor * input(i));
		}

		return result;
  }

  // controller parameters
  matrix::Vector3f _rate_limit;
  matrix::Matrix3f _switching_gain;
	matrix::Matrix3f _lambda;
	matrix::Matrix3f _inertia;
	float _tanh_factor;

	// setpoints
	matrix::Dcmf _attitude_setpoint;
	matrix::Vector3f _angular_velocity_setpoint;
	matrix::Vector3f _angular_acceleration_setpoint;

	// measurments
	matrix::Dcmf _attitude;
	matrix::Vector3f _angular_velocity;
	matrix::Vector3f _angular_acceleration;
};
c
