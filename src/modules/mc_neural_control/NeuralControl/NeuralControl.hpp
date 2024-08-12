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
  void updateNeural(float& thrust_setpoint, Quatf& quaternion_setpoint) const;

private:

  Eigen::VectorXf bias_control_net_layer_1;
  Eigen::MatrixXf weight_control_net_layer_1;
  Eigen::VectorXf bias_control_net_layer_2;
  Eigen::MatrixXf weight_control_net_layer_2;
  Eigen::VectorXf bias_control_net_layer_3;
  Eigen::MatrixXf weight_control_net_layer_3;
  Eigen::VectorXf bias_allocation_net_layer_1;
  Eigen::MatrixXf weight_allocation_net_layer_1;
  Eigen::VectorXf bias_allocation_net_layer_2;
  Eigen::MatrixXf weight_allocation_net_layer_2;
  Eigen::VectorXf max_thrust;
  Eigen::VectorXf min_thrust;
  Eigen::VectorXf max_torque;
  Eigen::VectorXf min_torque;
  Eigen::VectorXf limits_u;

  // setpoints
  Vector3f _position_setpoint{};

  // measurments
  Dcmf _attitude{};
  Vector3f _position{};
  Vector3f _linear_velocity{};
  Vector3f _angular_velocity;
};
