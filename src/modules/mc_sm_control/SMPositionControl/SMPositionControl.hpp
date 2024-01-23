#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

using namespace matrix;

class SMPositionControl
{
public:
	SMPositionControl() = default;
	~SMPositionControl() = default;

	void setSwitchingGain(const Matrix3f &switching_gain) { _switching_gain = switching_gain; }

	void setLambda(const Matrix3f &lambda) { _lambda = lambda; }

	void setMass(const float &mass) { _mass = mass; }

	void setTanhFactor(const float& tanh_factor) { _tanh_factor = tanh_factor; }

	void setYawSetpoint(const float& yaw) { _yaw_setpoint = yaw; }

	void setPositionSetpoint(const Vector3f &position_setpoint) { _position_setpoint = position_setpoint; }

	void setLinearVelocitySetpoint(const Vector3f &linear_velocity_setpoint) { _linear_velocity_setpoint = linear_velocity_setpoint; }

	void setLinearAccelerationSetpoint(const Vector3f &linear_acceleration_setpoint) { _linear_acceleration_setpoint = linear_acceleration_setpoint; }

	void setAttitude(const Quatf &quaternion) {
		_attitude = Dcmf(quaternion);
		printf("quaternion ");
		quaternion.print();
	}

	void setPosition(const Vector3f &position) {
		_position = position;
		printf("position ");
		position.print();
	}

	void setLinearVelocity(const Vector3f &linear_velocity) {
		_linear_velocity = linear_velocity;
		printf("linear_velocity ");
		linear_velocity.print();
	}

	void setLinearAcceleration(const Vector3f &linear_acceleration) {
		_linear_acceleration = linear_acceleration;
		printf("linear_acceleraton ");
		linear_acceleration.print();
	}

	void getAttitude() { return Quatf(_attitude); }

	void getPosition() { return _position; }

	/**
	 * Run one control loop cycle calculation
	 */
	void update(float &thrust_setpoint, Quatf &quaternion_setpoint) const;

private:
  Vector3f signum(const Vector3f& input) const {
    // return Vector3f(std::tanh(v[0]), std::tanh(v[1]), std::tanh(v[1]));
		Vector3f result;
		for (auto i=0; i<3; i++) {
			result(i) = std::tanh(_tanh_factor * input(i));
		}

		return result;
  }

	Vector3f calculateAcceleration() const;

	float calculateThrust(const Vector3f &acceleration) const;

	Dcmf calculateAttitude(const Vector3f &acceleration) const;

  // controller parameters
  Matrix3f _switching_gain;
	Matrix3f _lambda;
	float _mass;
	float _tanh_factor;

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
