#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

using namespace matrix;

class PDPositionControl
{
public:
	PDPositionControl() = default;
	~PDPositionControl() = default;

	// PD
	void setKp(const Matrix3f &K_p) { _K_p = K_p; }
	void setKd(const Matrix3f &K_d) { _K_d = K_d; }

	// STSMC
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
  void updatePD(float& thrust_setpoint, Vector3f& acceleration_setpoint) const;

  void convertToAttitude(const Vector3f& acceleration, Quatf& attitude) const { attitude = Quatf(calculateAttitude(acceleration)); }

private:

  Vector3f calculateAccelerationPD() const;

  float calculateThrust(const Vector3f& acceleration) const;

  Dcmf calculateAttitude(const Vector3f& acceleration) const;

  // controller parameters
  float _mass;
  // PD
  Matrix3f _K_p;
  Matrix3f _K_d;

  // setpoints
  float _yaw_setpoint;
  Vector3f _position_setpoint{};
  Vector3f _linear_velocity_setpoint{};
  Vector3f _linear_acceleration_setpoint{};

  // measurments
  Dcmf _attitude{};
  Vector3f _position{};
  Vector3f _linear_velocity{};
  Vector3f _linear_acceleration{};
};
