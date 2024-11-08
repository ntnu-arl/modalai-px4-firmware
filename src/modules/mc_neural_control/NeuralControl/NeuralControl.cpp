#include <NeuralControl.hpp>
#include <mathlib/math/Functions.hpp>
#include <matrix/matrix/math.hpp>
#include <drivers/drv_hrt.h>
#include <unsupported/Eigen/EulerAngles>

#include <iostream>
#include <memory>
#include <fstream>
#include <unordered_map>
#include <string>
#include <vector>
#include <chrono>
#include <math.h>

#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923

// typedef Eigen::EulerSystem<Eigen::EULER_Z, Eigen::EULER_Y, Eigen::EULER_X> MySystem;
// typedef Eigen::EulerAngles<float, MySystem> VehicleAngles;

// /**
//  * @brief Convert euler angles to quaternion.
//  */
// Eigen::Quaternionf quaternion_from_rpy(const Eigen::Vector3f &rpy)
// {
// 	// YPR - ZYX
// 	return Eigen::Quaternionf(
// 			Eigen::AngleAxisf(rpy.z(), Eigen::Vector3f::UnitZ()) *
// 			Eigen::AngleAxisf(rpy.y(), Eigen::Vector3f::UnitY()) *
// 			Eigen::AngleAxisf(rpy.x(), Eigen::Vector3f::UnitX())
// 			);
// }

// /**
//  * @brief Convert euler angles to quaternion.
//  *
//  * @return quaternion, same as @a tf::quaternionFromRPY() but in Eigen format.
//  */
// inline Eigen::Quaternionf quaternion_from_rpy(const float roll, const float pitch, const float yaw) {
// 	return quaternion_from_rpy(Eigen::Vector3f(roll, pitch, yaw));
// }

// static const auto NED_ENU_Q = quaternion_from_rpy(M_PI, 0.0, M_PI_2);
// static const auto AIRCRAFT_BASELINK_Q = quaternion_from_rpy(M_PI, 0.0, 0.0);
// static const auto NED_ENU_R = NED_ENU_Q.normalized().toRotationMatrix();
// static const auto AIRCRAFT_BASELINK_R = AIRCRAFT_BASELINK_Q.normalized().toRotationMatrix();
// static const Eigen::PermutationMatrix<3> NED_ENU_REFLECTION_XY(Eigen::Vector3i(1,0,2));
// static const Eigen::DiagonalMatrix<float,3> NED_ENU_REFLECTION_Z(1,1,-1);

Eigen::MatrixXf openData(std::string fileToOpen)
{

  std::vector<float> matrixEntries;

  // in this object we store the data from the matrix
  try
  {
    std::ifstream matrixDataFile(fileToOpen);
  }
  catch (const std::exception &e)
  {
    PX4_INFO("Error: while loading model files");
    std::cerr << "Error: while loading model files" << std::endl;
  }
  std::ifstream matrixDataFile(fileToOpen);

  // this variable is used to store the row of the matrix that contains commas
  std::string matrixRowString;

  // this variable is used to store the matrix entry;
  std::string matrixEntry;

  // this variable is used to track the number of rows
  int matrixRowNumber = 0;

  try
  {
    while (getline(matrixDataFile, matrixRowString)) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
    {
      std::stringstream matrixRowStringStream(matrixRowString); // convert matrixRowString that is a string to a stream variable.

      while (getline(matrixRowStringStream, matrixEntry, ',')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
      {
        matrixEntries.push_back(stod(matrixEntry)); // here we convert the string to double and fill in the row vector storing all the matrix entries
      }
      matrixRowNumber++; // update the column numbers
    }
  }
  catch (const std::exception &e)
  {
    PX4_INFO("Error: while processing model files");
    std::cerr << "Error: while processing model files" << std::endl;
  }

  // here we convet the vector variable into the matrix and return the resulting object,
  // note that matrixEntries.data() is the pointer to the first memory location at which the entries of the vector matrixEntries are stored;
  return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
}

// Eigen::Vector3f NeuralControl::transform_frame_ned_enu(Eigen::Vector3f vec_ned)
// {
//   return NED_ENU_REFLECTION_XY.transpose() * (NED_ENU_REFLECTION_Z * vec_ned);
// }

// Eigen::Quaternionf NeuralControl::transform_orientation_ned_enu(Eigen::Quaternionf q_ned)
// {
//   return NED_ENU_Q.inverse() * q_ned * AIRCRAFT_BASELINK_Q.inverse();
// }

NeuralControl::NeuralControl()
{
  try
  {
    std::string path = "/home/model_files/";
    _bias_control_net_layer_1 = openData(path + "bias_control_net_layer_1.csv");
    _weight_control_net_layer_1 = openData(path + "weight_control_net_layer_1.csv");
    _bias_control_net_layer_2 = openData(path + "bias_control_net_layer_2.csv");
    _weight_control_net_layer_2 = openData(path + "weight_control_net_layer_2.csv");
    _bias_control_net_layer_3 = openData(path + "bias_control_net_layer_3.csv");
    _weight_control_net_layer_3 = openData(path + "weight_control_net_layer_3.csv");
    _bias_allocation_net_layer_1 = openData(path + "bias_allocation_net_layer_1.csv");
    _weight_allocation_net_layer_1 = openData(path + "weight_allocation_net_layer_1.csv");
    _bias_allocation_net_layer_2 = openData(path + "bias_allocation_net_layer_2.csv");
    _weight_allocation_net_layer_2 = openData(path + "weight_allocation_net_layer_2.csv");
    _max_thrust = openData(path + "max_thrust.csv");
    _min_thrust = openData(path + "min_thrust.csv");
    _max_torque = openData(path + "max_torque.csv");
    _min_torque = openData(path + "min_torque.csv");
    Eigen::VectorXf limits_u = openData(path + "lim_u.csv");

    _min_u_training = limits_u(0);
    _max_u_training = limits_u(1);
    PX4_INFO("model files loaded!");

    // Debug State
    scaled_input_allocation_net = Eigen::VectorXf::Zero(6);
    force_clamped = Eigen::VectorXf::Zero(4);
    input = Eigen::VectorXf::Zero(15);
    
  }
  catch (const std::exception &e)
  {
    PX4_INFO("Error: while loading model files");
    std::cerr << "Error: while loading model files" << std::endl;
  }
}

void NeuralControl::fillDebugMessage(neural_control_s &message)
{
  int i = 0;
  message.timestamp = hrt_absolute_time();
  for (i = 0; i < 4; i++)
  {
    message.motor_thrust[i] = force_clamped(i);
  }
  for (i = 0; i < 6; i++)
  {
    message.wrench[i] = scaled_input_allocation_net(i);
  }
  for (i = 0; i < 15; i++)
  {
    message.observation[i] = input(i);
  }
}

matrix::Vector4f NeuralControl::updateNeural()
{

  std::chrono::time_point<std::chrono::system_clock> start1, end1;
  start1 = std::chrono::system_clock::now();

  // transform observations in correct frame
  matrix::Dcmf frame_transf;
  frame_transf(0, 0) = 1.0f;
  frame_transf(0, 1) = 0.0f;
  frame_transf(0, 2) = 0.0f;
  frame_transf(1, 0) = 0.0f;
  frame_transf(1, 1) = -1.0f;
  frame_transf(1, 2) = 0.0f;
  frame_transf(2, 0) = 0.0f;
  frame_transf(2, 1) = 0.0f;
  frame_transf(2, 2) = -1.0f;

  matrix::Dcmf frame_transf_2;
  frame_transf_2(0, 0) = 0.0f;
  frame_transf_2(0, 1) = 1.0f;
  frame_transf_2(0, 2) = 0.0f;
  frame_transf_2(1, 0) = -1.0f;
  frame_transf_2(1, 1) = 0.0f;
  frame_transf_2(1, 2) = 0.0f;
  frame_transf_2(2, 0) = 0.0f;
  frame_transf_2(2, 1) = 0.0f;
  frame_transf_2(2, 2) = 1.0f;

  // Reset States
  scaled_input_allocation_net = Eigen::VectorXf::Zero(6);
  force_clamped = Eigen::VectorXf::Zero(4);
  input = Eigen::VectorXf::Zero(15);

  Vector3f position_local;
  position_local = frame_transf * frame_transf_2 * _position;
  Vector3f position_setpoint_local;
  position_setpoint_local = frame_transf * frame_transf_2 * _position_setpoint;
  Vector3f linear_velocity_local;
  linear_velocity_local = frame_transf * frame_transf_2 * _linear_velocity;
  matrix::Dcmf _attitude_local_mat = frame_transf * (frame_transf_2 * matrix::Dcmf(_attitude)) * frame_transf.transpose();
  Vector3f angular_vel_local = frame_transf * _angular_velocity;

  // get state positions
  Eigen::Vector3f pos_state;
  pos_state << position_local(0), position_local(1), position_local(2);
  Eigen::Vector3f pos_setpoint;
  pos_setpoint << position_setpoint_local(0), position_setpoint_local(1), position_setpoint_local(2);
  Eigen::Vector3f pos_input = pos_setpoint - pos_state;

  // clamp error to guarantee input lies in training envelope
  Eigen::Vector3f pos_input_clamped = pos_input; //.cwiseMax(-1.).cwiseMin(1.);

  // convert linear velocities
  Eigen::Vector3f vel_state;
  vel_state << linear_velocity_local(0), linear_velocity_local(1), linear_velocity_local(2);

  // convert attitude
  Eigen::VectorXf attitude_state(6);
  attitude_state << _attitude_local_mat(0, 0), _attitude_local_mat(0, 1), _attitude_local_mat(0, 2),
      _attitude_local_mat(1, 0), _attitude_local_mat(1, 1), _attitude_local_mat(1, 2);

  // convert angular velocities
  Eigen::Vector3f angular_velocity_state;
  angular_velocity_state << angular_vel_local(0), angular_vel_local(1), angular_vel_local(2);

  // input vector for network
  input << pos_input_clamped, attitude_state, vel_state, angular_velocity_state;
  
  // forward path
  Eigen::VectorXf co1 = _weight_control_net_layer_1 * input + _bias_control_net_layer_1;
  Eigen::VectorXf ca1 = co1.array().tanh();
  Eigen::VectorXf co2 = _weight_control_net_layer_2 * ca1 + _bias_control_net_layer_2;
  Eigen::VectorXf ca2 = co2.array().tanh();
  Eigen::VectorXf co3 = _weight_control_net_layer_3 * ca2 + _bias_control_net_layer_3;
  Eigen::VectorXf ca3 = co3.array().tanh();
  Eigen::VectorXf input_allocation_net = ca3;

  Eigen::VectorXf scaled_thrust = input_allocation_net(Eigen::seq(0, 2)).cwiseProduct(_max_thrust - _min_thrust) / 2 + (_max_thrust + _min_thrust) / 2;
  Eigen::VectorXf scaled_torque = input_allocation_net(Eigen::seq(3, 5)).cwiseProduct(_max_torque - _min_torque) / 2 + (_max_torque + _min_torque) / 2;

  scaled_input_allocation_net << scaled_thrust, scaled_torque;

  Eigen::VectorXf ao1 = _weight_allocation_net_layer_1 * scaled_input_allocation_net + _bias_allocation_net_layer_1;
  // Eigen::VectorXf aa1 = ao1.cwiseMax(0);
  Eigen::VectorXf ao2 = _weight_allocation_net_layer_2 * ao1 + _bias_allocation_net_layer_2;

  force_clamped = ao2.cwiseMax(_min_u_training).cwiseMin(_max_u_training);

  static const float _thrust_coefficient = 0.000016781;
  // conversion to rpm
  Eigen::VectorXf rps = force_clamped / _thrust_coefficient; // _thrust_coefficient;0.000013781
  rps = rps.cwiseSqrt();
  Eigen::VectorXf rpm = rps * 60;

  // conversion to motor commands (inverse of the scaling done in mixer module)
  Vector4f motor_commands;

  motor_commands(0) = (rpm(0) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
  motor_commands(1) = (rpm(2) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
  motor_commands(2) = (rpm(3) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
  motor_commands(3) = (rpm(1) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);

  Vector4f mixer_values;

  const float a = 0.8f;
  const float b = (1.f - 0.8f);

  // don't recompute for all values (ax^2+bx+c=0)
  const float tmp1 = b / (2.f * a);
  const float tmp2 = b * b / (4.f * a * a);

  mixer_values(0) = a * (((motor_commands(0) + 1.0f) / 2.0f + tmp1) * ((motor_commands(0) + 1.0f) / 2.0f + tmp1) - tmp2);
  mixer_values(1) = a * (((motor_commands(1) + 1.0f) / 2.0f + tmp1) * ((motor_commands(1) + 1.0f) / 2.0f + tmp1) - tmp2);
  mixer_values(2) = a * (((motor_commands(2) + 1.0f) / 2.0f + tmp1) * ((motor_commands(2) + 1.0f) / 2.0f + tmp1) - tmp2);
  mixer_values(3) = a * (((motor_commands(3) + 1.0f) / 2.0f + tmp1) * ((motor_commands(3) + 1.0f) / 2.0f + tmp1) - tmp2);

  // PX4_WARN("value neural control: %f %f %f %f", (double)motor_commands(0), (double)motor_commands(1), (double)motor_commands(2), (double)motor_commands(3));

  end1 = std::chrono::system_clock::now();

  PX4_WARN("time total: %f",  double(std::chrono::duration_cast<std::chrono::microseconds>(end1 - start1).count()));

  return mixer_values;
}

matrix::Vector4f NeuralControl::allocate_output(matrix::VectorXf )
{

}
