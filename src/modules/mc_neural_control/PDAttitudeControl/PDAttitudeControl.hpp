#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

using namespace matrix;

class PDAttitudeControl
{
public:
	PDAttitudeControl() = default;
	~PDAttitudeControl() = default;

	// PD
	void setKp(const Matrix3f &K_p) { _K_p = K_p; }
	void setKd(const Matrix3f &K_d) { _K_d = K_d; }

	// STSMC
	void setInertia(const matrix::Matrix3f &inertia) { _inertia = inertia; }

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
		//printf("quaternion_setpoint: %f %f %f %f\n", (double)quaternion_setpoint(1),(double)quaternion_setpoint(2),(double)quaternion_setpoint(3),(double)quaternion_setpoint(0));
	}

	void setAngularVelocitySetpoint(const matrix::Vector3f &angular_velocity_setpoint) {
		_angular_velocity_setpoint = angular_velocity_setpoint;
		//printf("angular_velocity_setpoint: %f %f %f\n", (double)angular_velocity_setpoint(0),(double)angular_velocity_setpoint(1),(double)angular_velocity_setpoint(2));
	}

	void setAngularAccelerationSetpoint(const matrix::Vector3f &angular_acceleration_setpoint) {
		_angular_acceleration_setpoint = angular_acceleration_setpoint;
		//printf("_angular_acceleration_setpoint: %f %f %f\n", (double)_angular_acceleration_setpoint(0),(double)_angular_acceleration_setpoint(1),(double)_angular_acceleration_setpoint(2));
	}

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
		_attitude_setpoint = matrix::Dcmf(q_delta) * _attitude_setpoint;
		_attitude_setpoint.renormalize();
	}

	/**
	 * Run one control loop cycle calculation
	 * @param q estimation of the current vehicle attitude unit quaternion
	 * @return [rad/s] body frame 3D angular rate setpoint vector to be executed by the rate controller
	 */
	Vector3f updatePD() const;

private:

  // controller parameters
  // PD
  Matrix3f _K_p;
  Matrix3f _K_d;

  // STSMC
  matrix::Vector3f _rate_limit;
  matrix::Matrix3f _inertia;

  // setpoints
  matrix::Dcmf _attitude_setpoint;
  matrix::Vector3f _angular_velocity_setpoint;
  matrix::Vector3f _angular_acceleration_setpoint;

  // measurments
  matrix::Dcmf _attitude;
  matrix::Vector3f _angular_velocity;
  matrix::Vector3f _angular_acceleration;
};
