#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>
#include <lib/eigen/Eigen/Dense>
#include <uORB/topics/neural_control.h>
//#include <lib/eigen/Eigen/Core>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

using namespace matrix;

class NeuralControlUnconstrained
{
public:
	NeuralControlUnconstrained( int n_motors);
	~NeuralControlUnconstrained() = default;

	void setPositionSetpoint(const Vector3f &position_setpoint) { _position_setpoint = position_setpoint; }

  void setVelocitySetpoint(const Vector3f &velocity_setpoint) { _velocity_setpoint = velocity_setpoint; }

	void setAttitude(const Quatf &quaternion) { _attitude = quaternion;	}

	void setAngularVelocity(const matrix::Vector3f &angular_velocity) {_angular_velocity = angular_velocity; }

	void setPosition(const Vector3f &position) { _position = position;	}

	void setLinearVelocity(const Vector3f &linear_velocity) { _linear_velocity = linear_velocity;	}

  void setMaxRPM(int max_rpm) { _max_rpm = max_rpm; }

  void setMinRPM(int min_rpm) { _min_rpm = min_rpm; }

  void setThrustCoefficient(const float thrust_coefficient) { _thrust_coefficient = thrust_coefficient; }

  void setMaxError(const float max_error) { _max_error = max_error; }

	Quatf getAttitude() { return Quatf(_attitude); }

	Vector3f getPosition() { return _position; }

  void fillDebugMessage(neural_control_s &message);

  Eigen::Vector3f transform_frame_ned_enu(Eigen::Vector3f vec_ned);

  Eigen::Quaternionf transform_orientation_ned_enu(Eigen::Quaternionf quat_ned);

  int _n_motors; 

  /**
   * Run one control loop cycle calculation
   */
  matrix::Vector<float,6> updateNeural();

private:

  Eigen::VectorXf _bias_control_net_layer_0;
  Eigen::MatrixXf _weight_control_net_layer_0;
  Eigen::VectorXf _bias_control_net_layer_1;
  Eigen::MatrixXf _weight_control_net_layer_1;
  Eigen::VectorXf _bias_control_net_layer_3;
  Eigen::MatrixXf _weight_control_net_layer_3;
  Eigen::VectorXf _bias_control_net_layer_5;
  Eigen::MatrixXf _weight_control_net_layer_5;
  Eigen::VectorXf _bias_control_net_layer_7;
  Eigen::MatrixXf _weight_control_net_layer_7;
  Eigen::VectorXf _limits_u;

  float _min_u_training;
  float _max_u_training;
  int _max_rpm;
  int _min_rpm;
  float _max_error;
  float _thrust_coefficient;

  // transform observations in correct frame
  matrix::Dcmf _frame_transf;
  matrix::Dcmf _frame_transf_2;

  // this are the min and max forces that the motor 
  // can generate and have to be estimated from the real system

  // setpoints
  Vector3f _position_setpoint{};
  Vector3f _velocity_setpoint{};

  // measurments
  Quatf _attitude{};
  Vector3f _position{};
  Vector3f _linear_velocity{};
  Vector3f _angular_velocity;

  // Debug States
  Eigen::VectorXf _force_clamped;
  Eigen::VectorXf _input;

};
