#include <NeuralControlUnconstrained.hpp>
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

Eigen::MatrixXf openDataUnconstrained(std::string fileToOpen)
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

NeuralControlUnconstrained::NeuralControlUnconstrained(int n_motors)
{
  _n_motors = n_motors;

  try
  {
    std::string path = "/home/model_files_unconstrained/";
    PX4_INFO("loading model files");

    _bias_control_net_layer_1 = openDataUnconstrained(path + "bias_control_net_layer_1.csv");
    _weight_control_net_layer_1 = openDataUnconstrained(path + "weight_control_net_layer_1.csv");
    _bias_control_net_layer_2 = openDataUnconstrained(path + "bias_control_net_layer_2.csv");
    _weight_control_net_layer_2 = openDataUnconstrained(path + "weight_control_net_layer_2.csv");
    _bias_control_net_layer_3 = openDataUnconstrained(path + "bias_control_net_layer_3.csv");
    _weight_control_net_layer_3 = openDataUnconstrained(path + "weight_control_net_layer_3.csv");

    _bias_allocation_net_layer_1 = openDataUnconstrained(path + "bias_allocation_net_layer_1.csv");
    _weight_allocation_net_layer_1 = openDataUnconstrained(path + "weight_allocation_net_layer_1.csv");
    _bias_allocation_net_layer_2 = openDataUnconstrained(path + "bias_allocation_net_layer_2.csv");
    _weight_allocation_net_layer_2 = openDataUnconstrained(path + "weight_allocation_net_layer_2.csv");
    
    _max_wrench = openDataUnconstrained(path + "max_wrench.csv");
    _min_wrench = openDataUnconstrained(path + "min_wrench.csv");
    _pa_rot_mat = openDataUnconstrained(path + "pa_rot_matrix.csv");
    _pa_center = openDataUnconstrained(path + "pa_center.csv");
    _limits_u = openDataUnconstrained(path + "lim_u.csv");

    _min_u_training = _limits_u(0);
    _max_u_training = _limits_u(1);
    PX4_INFO("model files loaded!");

    // Debug State
    _scaled_input_allocation_net = Eigen::VectorXf::Zero(6);
    _force_clamped = Eigen::VectorXf::Zero(4);
    _force_offset_comp = Eigen::VectorXf::Zero(4);
    _input = Eigen::VectorXf::Zero(15);

    _frame_transf(0, 0) = 1.0f;
    _frame_transf(0, 1) = 0.0f;
    _frame_transf(0, 2) = 0.0f;
    _frame_transf(1, 0) = 0.0f;
    _frame_transf(1, 1) = -1.0f;
    _frame_transf(1, 2) = 0.0f;
    _frame_transf(2, 0) = 0.0f;
    _frame_transf(2, 1) = 0.0f;
    _frame_transf(2, 2) = -1.0f;

    _frame_transf_2(0, 0) = 0.0f;
    _frame_transf_2(0, 1) = 1.0f;
    _frame_transf_2(0, 2) = 0.0f;
    _frame_transf_2(1, 0) = -1.0f;
    _frame_transf_2(1, 1) = 0.0f;
    _frame_transf_2(1, 2) = 0.0f;
    _frame_transf_2(2, 0) = 0.0f;
    _frame_transf_2(2, 1) = 0.0f;
    _frame_transf_2(2, 2) = 1.0f;
    
  }
  catch (const std::exception &e)
  {
    PX4_INFO("Error: while loading model files");
    std::cerr << "Error: while loading model files" << std::endl;
  }
}

void NeuralControlUnconstrained::fillDebugMessage(neural_control_s &message)
{
  int i = 0;
  message.timestamp = hrt_absolute_time();
  for (i = 0; i < _n_motors; i++)
  {
    message.motor_thrust[i] = _force_clamped(i);
  }
  for (i = 0; i < 6; i++)
  {
    message.wrench[i] = _scaled_input_allocation_net(i);
  }
  for (i = 0; i < 15; i++)
  {
    message.observation[i] = _input(i);
  }
  for (i = 0; i < 6; i++)
  {
    message.debug[i] = _debug(i);
  }
}

matrix::Vector<float,6> NeuralControlUnconstrained::updateNeural()
{

  // transform observations in correct frame
  Vector3f position_local;
  position_local = _frame_transf * _frame_transf_2 * _position;

  Vector3f position_setpoint_local;
  position_setpoint_local = _frame_transf * _frame_transf_2 * _position_setpoint;

  Vector3f linear_velocity_local;
  linear_velocity_local = _frame_transf * _frame_transf_2 * _linear_velocity;

  matrix::Dcmf _attitude_local_mat = _frame_transf * (_frame_transf_2 * matrix::Dcmf(_attitude)) * _frame_transf.transpose();
  matrix::Eulerf euler_angles_local(_attitude_local_mat);

  // PX4_INFO("________________________");
  //PX4_INFO("attitude: %f %f %f", (double)euler_angles_local.phi(), (double)euler_angles_local.theta(), (double)euler_angles_local.psi());

  Vector3f angular_vel_local = _frame_transf * _angular_velocity;

  // get state positions
  Eigen::Vector3f pos_state;
  pos_state << position_local(0), position_local(1), position_local(2);
  Eigen::Vector3f pos_setpoint;
  pos_setpoint << position_setpoint_local(0), position_setpoint_local(1), position_setpoint_local(2);

  Eigen::Vector3f pos_input = pos_setpoint - pos_state;

  //PX4_INFO("position: %f %f %f", (double)pos_input(0), (double)pos_input(1), (double)pos_input(2));

  // clamp error to guarantee input lies in training envelope
  Eigen::Vector3f pos_input_clamped = pos_input; //.cwiseMax(-1.).cwiseMin(1.);

  // convert linear velocities
  Eigen::Vector3f vel_state;
  vel_state << linear_velocity_local(0), linear_velocity_local(1), linear_velocity_local(2);

  Eigen::VectorXf attitude_state(6);
  attitude_state << _attitude_local_mat(0, 0), _attitude_local_mat(0, 1), _attitude_local_mat(0, 2),
      _attitude_local_mat(1, 0), _attitude_local_mat(1, 1), _attitude_local_mat(1, 2);

  // convert angular velocities
  Eigen::Vector3f angular_velocity_state;
  angular_velocity_state << angular_vel_local(0), angular_vel_local(1), angular_vel_local(2);

  // input vector for network
  _input = Eigen::VectorXf::Zero(15);
  _input << pos_input_clamped, attitude_state, vel_state, angular_velocity_state;

  // forward path
  Eigen::VectorXf co1 = _weight_control_net_layer_1 * _input + _bias_control_net_layer_1;
  Eigen::VectorXf ca1 = co1.array().tanh();
  Eigen::VectorXf co2 = _weight_control_net_layer_2 * ca1 + _bias_control_net_layer_2;
  Eigen::VectorXf ca2 = co2.array().tanh();
  Eigen::VectorXf co3 = _weight_control_net_layer_3 * ca2 + _bias_control_net_layer_3;
  Eigen::VectorXf ca3 = co3.array().tanh();
  Eigen::VectorXf input_allocation_net = ca3;

  Eigen::VectorXf scaled_input_allocation_net = input_allocation_net.cwiseProduct(_max_wrench - _min_wrench)/2 + (_max_wrench + _min_wrench)/2;
  Eigen::VectorXf transformed_input_allocation_net = _pa_rot_mat * scaled_input_allocation_net + _pa_center;

  Eigen::VectorXf ao1 = _weight_allocation_net_layer_1 * transformed_input_allocation_net + _bias_allocation_net_layer_1;
  Eigen::VectorXf ao2 = _weight_allocation_net_layer_2 * ao1 + _bias_allocation_net_layer_2;
  Eigen::VectorXf output_allocation_net = ao2;

  _force_clamped = Eigen::VectorXf::Zero(_n_motors);
  _force_clamped = output_allocation_net.cwiseMax(_min_u_training).cwiseMin(_max_u_training);

  // conversion to rpm
  static const float _thrust_coefficient = 0.00001286412;

  Eigen::VectorXf rps = Eigen::VectorXf::Zero(_n_motors);
  rps = _force_clamped / _thrust_coefficient;
  rps = rps.cwiseSqrt();
  Eigen::VectorXf rpm = rps * 60;

  // conversion to motor commands (inverse of the scaling done in mixer module)
  matrix::Vector<float,6> motor_commands;

  if (_n_motors == 4){
    motor_commands(0) = (rpm(0) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
    motor_commands(1) = (rpm(2) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
    motor_commands(2) = (rpm(3) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
    motor_commands(3) = (rpm(1) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
    motor_commands(4) = -NAN;
    motor_commands(5) = -NAN;
  }
  else if (_n_motors == 6)
  {
    motor_commands(0) = (rpm(0) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
    motor_commands(1) = (rpm(2) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
    motor_commands(2) = (rpm(3) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
    motor_commands(3) = (rpm(5) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
    motor_commands(4) = (rpm(1) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
    motor_commands(5) = (rpm(4) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
  }
  else
  {
    PX4_INFO("Error: number of motors not supported");
  }

  matrix::Vector<float,6> mixer_values;

  const float a = 0.8f;
  const float b = (1.f - 0.8f);

  // don't recompute for all values (ax^2+bx+c=0)
  const float tmp1 = b / (2.f * a);
  const float tmp2 = b * b / (4.f * a * a);

  for (int i = 0; i < 6; i++)
  {
    mixer_values(i) = a * (((motor_commands(i) + 1.0f) / 2.0f + tmp1) * ((motor_commands(i) + 1.0f) / 2.0f + tmp1) - tmp2);
  }

  _debug(0) = mixer_values(0);
  _debug(1) = mixer_values(1);
  _debug(2) = mixer_values(2);
  _debug(3) = mixer_values(3);
  _debug(4) = mixer_values(4);
  _debug(5) = mixer_values(5);

  return mixer_values;
}


