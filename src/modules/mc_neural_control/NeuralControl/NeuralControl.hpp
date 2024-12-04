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

class NeuralControl
{
public:
	NeuralControl();
	~NeuralControl() = default;

	void setPositionSetpoint(const Vector3f &position_setpoint) { _position_setpoint = position_setpoint; }

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

  /**
   * Run one control loop cycle calculation
   */
  Vector4f updateNeural();

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
  Vector3f _smoothed_pos;
  Vector3f _smoothed_vel;
  Eulerf _smoothed_att;
  Vector3f _smoothed_ang_vel;

  float _min_u_training;
  float _max_u_training;
  int _max_rpm;
  int _min_rpm;
  float _max_error;
  float _thrust_coefficient;

  // this are the min and max forces that the motor 
  // can generate and have to be estimated from the real system

  // setpoints
  Vector3f _position_setpoint{};

  // measurments
  Quatf _attitude{};
  Vector3f _position{};
  Vector3f _linear_velocity{};
  Vector3f _angular_velocity;

  // Debug States
  Eigen::VectorXf _scaled_input_allocation_net;
  Eigen::VectorXf _force_clamped;
  Eigen::VectorXf _force_offset_comp;
  Eigen::VectorXf _input;

};
