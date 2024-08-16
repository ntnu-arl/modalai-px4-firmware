#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <lib/eigen/Eigen/Dense>
//#include <lib/eigen/Eigen/Core>

using namespace matrix;

class NeuralControl
{
public:
	NeuralControl();
	~NeuralControl() = default;

	void setPositionSetpoint(const Vector3f &position_setpoint) { _position_setpoint = position_setpoint; }

	void setAttitude(const Quatf &quaternion) { _attitude = Dcmf(quaternion);	}

	void setAngularVelocity(const matrix::Vector3f &angular_velocity) {_angular_velocity = angular_velocity; }

	void setPosition(const Vector3f &position) { _position = position;	}

	void setLinearVelocity(const Vector3f &linear_velocity) { _linear_velocity = linear_velocity;	}

	Quatf getAttitude() { return Quatf(_attitude); }

	Vector3f getPosition() { return _position; }

  /**
   * Run one control loop cycle calculation
   */
  Vector4f updateNeural() const;

private:

  Eigen::VectorXf _bias_control_net_layer_1;
  Eigen::MatrixXf _weight_control_net_layer_1;
  Eigen::VectorXf _bias_control_net_layer_2;
  Eigen::MatrixXf _weight_control_net_layer_2;
  Eigen::VectorXf _bias_control_net_layer_3;
  Eigen::MatrixXf _weight_control_net_layer_3;
  Eigen::VectorXf _bias_allocation_net_layer_1;
  Eigen::MatrixXf _weight_allocation_net_layer_1;
  Eigen::VectorXf _bias_allocation_net_layer_2;
  Eigen::MatrixXf _weight_allocation_net_layer_2;
  Eigen::VectorXf _max_thrust;
  Eigen::VectorXf _min_thrust;
  Eigen::VectorXf _max_torque;
  Eigen::VectorXf _min_torque;
  float _min_u_training;
  float _max_u_training;
  // this are the min and max forces that the motor 
  // can generate and have to be estimated from the real system
  float _min_u_deploy = 2.0f;
  float _max_u_deploy = 0.0f;

  // setpoints
  Vector3f _position_setpoint{};

  // measurments
  Dcmf _attitude{};
  Vector3f _position{};
  Vector3f _linear_velocity{};
  Vector3f _angular_velocity;
};
