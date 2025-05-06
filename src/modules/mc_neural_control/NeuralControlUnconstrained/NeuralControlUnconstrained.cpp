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

    first_time_set = false;
    starting_position_offset = Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
    goal_index = 0;

    std::string path = "/home/model_files_unconstrained/";
    PX4_INFO("loading model files");

    _bias_control_net_layer_0 = openDataUnconstrained(path + "bias_control_net_layer_0.csv");
    _weight_control_net_layer_0 = openDataUnconstrained(path + "weight_control_net_layer_0.csv");
    _bias_control_net_layer_1 = openDataUnconstrained(path + "bias_control_net_layer_1.csv");
    _weight_control_net_layer_1 = openDataUnconstrained(path + "weight_control_net_layer_1.csv");
    _bias_control_net_layer_3 = openDataUnconstrained(path + "bias_control_net_layer_3.csv");
    _weight_control_net_layer_3 = openDataUnconstrained(path + "weight_control_net_layer_3.csv");
    _bias_control_net_layer_5 = openDataUnconstrained(path + "bias_control_net_layer_5.csv");
    _weight_control_net_layer_5 = openDataUnconstrained(path + "weight_control_net_layer_5.csv");
    _bias_control_net_layer_7 = openDataUnconstrained(path + "bias_control_net_layer_7.csv");
    _weight_control_net_layer_7 = openDataUnconstrained(path + "weight_control_net_layer_7.csv");

    _limits_u = openDataUnconstrained(path + "lim_u.csv");

    _min_u_training = _limits_u(0);
    _max_u_training = _limits_u(1);
    PX4_INFO("model files loaded!");

    // Debug State
    _force_clamped = Eigen::VectorXf::Zero(6);
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
  PX4_INFO("writing debug message");
  message.timestamp = hrt_absolute_time();
  for (i = 0; i < _n_motors; i++)
  {
    message.motor_thrust[i] = _force_clamped(i);
  }
  for (i = 0; i < 15; i++)
  {
    message.observation[i] = _input(i);
  }
}

matrix::Vector<float,6> NeuralControlUnconstrained::updateNeural()
{


  Eigen::Vector3f desired_starting_position(-0.5f, 0.0f, 0.0f);
  std::vector<Vector3f> goals_list = {
    Vector3f(0.0f, 0.0f, 0.0f),
    Vector3f(0.25f, 0.0f, 0.0f),
    Vector3f(0.5f, 0.0f, 0.0f),
    Vector3f(0.75f, 0.0f, 0.0f),
    Vector3f(1.0f, 0.6145519614219666f, 0.03982148319482803f),
    Vector3f(1.25f, 0.6145519614219666f, 0.03982148319482803f),
    Vector3f(1.5f, 0.6145519614219666f, 0.03982148319482803f),
    Vector3f(1.75f, 0.6145519614219666f, 0.03982148319482803f),
    Vector3f(2.0f, 0.6145519614219666f, 0.03982148319482803f),
    Vector3f(2.25f, 0.6145519614219666f, 0.03982148319482803f),
    Vector3f(2.5f, 0.6145519614219666f, 0.03982148319482803f),
    Vector3f(2.75f, 0.6145519614219666f, 0.03982148319482803f),
    Vector3f(3.0f, 0.6145519614219666f, 0.03982148319482803f),
    Vector3f(3.25f, -0.013058841228485107f, 0.06924483180046082f),
    Vector3f(3.5f, -0.013058841228485107f, 0.06924483180046082f),
    Vector3f(3.75f, -0.013058841228485107f, 0.06924483180046082f),
    Vector3f(4.0f, -0.013058841228485107f, 0.06924483180046082f),
    Vector3f(4.25f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(4.5f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(4.75f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(5.0f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(5.25f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(5.5f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(5.75f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(6.0f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(6.25f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(6.5f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(6.75f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(7.0f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(7.25f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(7.5f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(7.75f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(8.0f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(8.25f, -0.5529854893684387f, 0.0442693829536438f),
    Vector3f(8.5f, -0.5529854893684387f, 0.0442693829536438f),
  };


  // transform observations in correct frame
  Vector3f position_local;
  position_local = _frame_transf * _frame_transf_2 * _position;

  Vector3f position_setpoint_local;
  position_setpoint_local = _frame_transf * _frame_transf_2 * _position_setpoint;

  Vector3f linear_velocity_local;
  linear_velocity_local = _frame_transf * _frame_transf_2 * _linear_velocity;

  Vector3f linear_velocity_setpoint_local;
  linear_velocity_setpoint_local = _frame_transf * _frame_transf_2 * _velocity_setpoint;

  matrix::Dcmf _attitude_local_mat = _frame_transf * (_frame_transf_2 * matrix::Dcmf(_attitude)) * _frame_transf.transpose();
  matrix::Eulerf euler_angles_local(_attitude_local_mat);

  Vector3f angular_vel_local = _frame_transf * _angular_velocity;

  Eigen::Vector3f pos_state;
  pos_state << position_local(0), position_local(1), position_local(2);



  if (!first_time_set){
    starting_position_offset = pos_state - desired_starting_position;
    first_time_set = true;
  }

  if (starting_position_offset.array().isNaN().any()) {
    PX4_ERR("starting_position_offset contains NaN, crashing.");
    std::cerr << "Error: starting_position_offset is NaN. Exiting." << std::endl;
    exit(EXIT_FAILURE);
  }

  pos_state_w_starting_offset = pos_state - starting_position_offset;
  float min_y_dist_for_new_gate = 0.30f;
  if (goals_list[goal_index](0) < pos_state_w_starting_offset(0) && std::abs(goals_list[goal_index](1) - pos_state_w_starting_offset(1)) < min_y_dist_for_new_gate)
  {
    if (goal_index < goals_list.size()-1)
    {
      goal_index++;
    }
  }

  pos_setpoint << goals_list[goal_index](0), goals_list[goal_index](1), goals_list[goal_index](2);
  PX4_INFO("Updated pos_setpoint: %f %f %f", double(pos_setpoint(0)), double(pos_setpoint(1)), double(pos_setpoint(2)));

  pos_input = pos_setpoint - pos_state_w_starting_offset;
  pos_input_clamped = pos_input; // no clamping for now

  // convert linear velocities
  Eigen::Vector3f vel_state;
  vel_state << linear_velocity_local(0), linear_velocity_local(1), linear_velocity_local(2);

  Eigen::Vector3f vel_setpoint;
  vel_setpoint << linear_velocity_setpoint_local(0), linear_velocity_setpoint_local(1), linear_velocity_setpoint_local(2);

  Eigen::Vector3f vel_input = vel_state; // - vel_setpoint;

  Eigen::VectorXf attitude_state(6);
  attitude_state << _attitude_local_mat(0, 0), _attitude_local_mat(0, 1), _attitude_local_mat(0, 2),
      _attitude_local_mat(1, 0), _attitude_local_mat(1, 1), _attitude_local_mat(1, 2);

  // convert angular velocities
  Eigen::Vector3f angular_velocity_state;
  angular_velocity_state << angular_vel_local(0), angular_vel_local(1), angular_vel_local(2);

  // input vector for network
  _input = Eigen::VectorXf::Zero(15);
  _input << pos_input_clamped, attitude_state, vel_input, angular_velocity_state;

  // forward path
  Eigen::VectorXf co0 = _weight_control_net_layer_0 * _input + _bias_control_net_layer_0;
  Eigen::VectorXf co1 = _weight_control_net_layer_1 * co0 + _bias_control_net_layer_1;
  Eigen::VectorXf ca1 = co1.array().tanh();
  Eigen::VectorXf co2 = _weight_control_net_layer_3 * ca1 + _bias_control_net_layer_3;
  Eigen::VectorXf ca2 = co2.array().tanh();
  Eigen::VectorXf co3 = _weight_control_net_layer_5 * ca2 + _bias_control_net_layer_5;
  Eigen::VectorXf ca3 = co3.array().tanh();
  Eigen::VectorXf co4 = _weight_control_net_layer_7 * ca3 + _bias_control_net_layer_7;
  Eigen::VectorXf net_output = co4;

  Eigen::VectorXf scaled_motor_commands = net_output*(_max_u_training - _min_u_training)/2 + Eigen::VectorXf::Ones(6)*(_max_u_training + _min_u_training)/2;

  _force_clamped = Eigen::VectorXf::Zero(_n_motors);
  _force_clamped = scaled_motor_commands.cwiseMax(_min_u_training).cwiseMin(_max_u_training);

  // conversion to rpm
  _thrust_coefficient = 0.00001286412;

  Eigen::VectorXf rps = Eigen::VectorXf::Zero(_n_motors);
  rps = _force_clamped / _thrust_coefficient;
  rps = rps.cwiseSqrt();
  Eigen::VectorXf rpm = rps * 60;

  // conversion to motor commands (inverse of the scaling done in mixer module)
  matrix::Vector<float,6> motor_commands;

  motor_commands(0) = (rpm(3) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
  motor_commands(1) = (rpm(5) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
  motor_commands(2) = (rpm(2) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
  motor_commands(3) = (rpm(0) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
  motor_commands(4) = (rpm(4) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);
  motor_commands(5) = (rpm(1) * 2 - _max_rpm - _min_rpm) / (_max_rpm - _min_rpm);

  // PX4_INFO("commands %f %f %f %f %f %f", double(motor_commands(0)), double(motor_commands(1)), double(motor_commands(2)), double(motor_commands(3)), double(motor_commands(4)), double(motor_commands(5)));

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

  // std::string poses_log_file_path = "/home/paran/Dropbox/NTNU/11_constraints_encoding/code/poses_gz.txt";
  // {
  //   std::ofstream ofs(poses_log_file_path, std::ios::app);
  //   Eigen::Matrix3f attitudeMat;
  //   for (int i = 0; i < 3; i++) {
  //       for (int j = 0; j < 3; j++) {
  //           attitudeMat(i, j) = _attitude_local_mat(i, j);
  //       }
  //   }
  //   Eigen::Quaternionf q(attitudeMat);
  //   ofs << "{'timestamp': " << hrt_absolute_time() << ", 'position': [" 
  //     << pos_state_w_starting_offset(0) << ", " << pos_state_w_starting_offset(1) << ", " << pos_state_w_starting_offset(2) 
  //     << "], 'orientation': [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() 
  //     << "], 'velocity': [" << linear_velocity_local(0) << ", " << linear_velocity_local(1) << ", " << linear_velocity_local(2)
  //     << "], 'angular_velocity': [" << angular_vel_local(0) << ", " << angular_vel_local(1) << ", " << angular_vel_local(2) 
  //     << "], 'actions': [" << motor_commands(0) << ", " << motor_commands(1) << ", " << motor_commands(2) 
  //     << ", " << motor_commands(3) << ", " << motor_commands(4) << ", " << motor_commands(5) 
  //     << "], 'target': [" << pos_setpoint(0) << ", " << pos_setpoint(1) << ", " << pos_setpoint(2) 
  //     << "], 'error': [" << (pos_setpoint(0)-pos_state(0)) << ", " << (pos_setpoint(1)-pos_state(1)) << ", " 
  //     << (pos_setpoint(2)-pos_state(2)) << "]}" << std::endl;
  //   ofs.flush();
  // }

  return mixer_values;
}


