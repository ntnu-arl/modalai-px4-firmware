#include <NeuralControl.hpp>
#include <mathlib/math/Functions.hpp>
#include <matrix/matrix/math.hpp>

#include <iostream>
#include <memory>
#include <fstream>
#include <unordered_map>
#include <string>
#include <vector>


Eigen::MatrixXf openData(std::string fileToOpen)
{

	std::vector<float> matrixEntries;

	// in this object we store the data from the matrix
  try {
    std::ifstream matrixDataFile(fileToOpen);
  }
  catch (const std::exception& e) {
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

  try {
    while (getline(matrixDataFile, matrixRowString)) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
    {
      std::stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.

      while (getline(matrixRowStringStream, matrixEntry, ',')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
      {
        matrixEntries.push_back(stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries
      }
      matrixRowNumber++; //update the column numbers
    }
  }
  catch (const std::exception& e) {
    PX4_INFO("Error: while processing model files");
    std::cerr << "Error: while processing model files" << std::endl;
  }

	// here we convet the vector variable into the matrix and return the resulting object, 
	// note that matrixEntries.data() is the pointer to the first memory location at which the entries of the vector matrixEntries are stored;
	return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
}

NeuralControl::NeuralControl()
{
  try {
    std::string path = "/home/model_files/";
    PX4_INFO("loading model files");
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

  }
  catch (const std::exception& e) {
    PX4_INFO("Error: while loading model files");
    std::cerr << "Error: while loading model files" << std::endl;
  }
}

matrix::Vector4f NeuralControl::updateNeural() const
{

  // transform observations in correct frame
  matrix::Dcmf frame_transf; 
  frame_transf(0,0) = 1.0f;
  frame_transf(0,1) = 0.0f;
  frame_transf(0,2) = 0.0f;
  frame_transf(1,0) = 0.0f;
  frame_transf(1,1) = -1.0f;
  frame_transf(1,2) = 0.0f;
  frame_transf(2,0) = 0.0f;
  frame_transf(2,1) = 0.0f;
  frame_transf(2,2) = -1.0f;

  Vector3f position_local;
  position_local = frame_transf* _position;

  Vector3f position_setpoint_local;
  position_setpoint_local = frame_transf * _position_setpoint;

  Vector3f linear_velocity_local;
  linear_velocity_local = frame_transf * _linear_velocity;

  matrix::Dcmf _attitude_local_mat = frame_transf * matrix::Dcmf(_attitude) * frame_transf.transpose();
  Vector3f angular_vel_local = frame_transf * _angular_velocity;

  matrix::Eulerf euler_local_attitude(_attitude_local_mat);
  matrix::Eulerf euler_attitude(_attitude);

  //get state positions
  Eigen::Vector3f pos_state;
  pos_state << position_local(0),position_local(1),position_local(2);
  Eigen::Vector3f pos_setpoint;
  pos_setpoint << position_setpoint_local(0),position_setpoint_local(1),position_setpoint_local(2);
  pos_setpoint(0) = 0.0f;
  pos_setpoint(1) = 0.0f;
  pos_setpoint(2) = 1.0f;

  //Eigen::Vector3f ground_offset_local;
  //ground_offset_local << 0.0f, 0.0f, 1.0f;

  Eigen::Vector3f pos_input = pos_setpoint - pos_state;// + ground_offset_local; 

  // clamp error to guarantee input lies in training envelope
  Eigen::Vector3f pos_input_clamped = pos_input.cwiseMax(-0.3).cwiseMin(0.3);  

  //convert linear velocities
  Eigen::Vector3f vel_state;
  vel_state << linear_velocity_local(0),linear_velocity_local(1),linear_velocity_local(2);

  Eigen::VectorXf attitude_state(6);
  attitude_state << _attitude_local_mat(0,0), _attitude_local_mat(0,1), _attitude_local_mat(0,2),
                    _attitude_local_mat(1,0), _attitude_local_mat(1,1), _attitude_local_mat(1,2);

  //convert angular velocities
  Eigen::Vector3f angular_velocity_state;
  angular_velocity_state << angular_vel_local(0),angular_vel_local(1),angular_vel_local(2);

  // input vector for network
  Eigen::VectorXf input(pos_state.size() + attitude_state.size() + vel_state.size() + angular_velocity_state.size());
  input << pos_input_clamped, attitude_state, vel_state, angular_velocity_state;

  // forward path
  Eigen::VectorXf co1 = _weight_control_net_layer_1 * input + _bias_control_net_layer_1;
  Eigen::VectorXf ca1 = co1.array().tanh();
  Eigen::VectorXf co2 = _weight_control_net_layer_2 * ca1 + _bias_control_net_layer_2;
  Eigen::VectorXf ca2 = co2.array().tanh();
  Eigen::VectorXf co3 = _weight_control_net_layer_3 * ca2 + _bias_control_net_layer_3;
  Eigen::VectorXf ca3 = co3.array().tanh();
  Eigen::VectorXf input_allocation_net = ca3;

  Eigen::VectorXf scaled_thrust = input_allocation_net(Eigen::seq(0,2)).cwiseProduct(_max_thrust - _min_thrust)/2 + (_max_thrust + _min_thrust)/2;
  Eigen::VectorXf scaled_torque = input_allocation_net(Eigen::seq(3,5)).cwiseProduct(_max_torque - _min_torque)/2 + (_max_torque + _min_torque)/2;

  Eigen::VectorXf scaled_input_allocation_net = Eigen::VectorXf::Zero(6);
  scaled_input_allocation_net << scaled_thrust, scaled_torque;

  Eigen::VectorXf ao1 = _weight_allocation_net_layer_1 * scaled_input_allocation_net + _bias_allocation_net_layer_1;
  Eigen::VectorXf aa1 = ao1.cwiseMax(0);
  Eigen::VectorXf ao2 = _weight_allocation_net_layer_2 * aa1 + _bias_allocation_net_layer_2;

  Eigen::VectorXf force_clamped = ao2.cwiseMax(_min_u_training).cwiseMin(_max_u_training);

  //conversion to rpm
  Eigen::VectorXf rps = force_clamped / _thrust_coefficient;
  rps = rps.cwiseSqrt();
  Eigen::VectorXf rpm = rps*60;

  //conversion to motor commands (inverse of the scaling done in mixer module)
  Vector4f motor_commands;

  //PX4_WARN("rpm: %f %f %f %f", (double)rpm(0), (double)rpm(1), (double)rpm(2), (double)rpm(3));

  motor_commands(0) = (rpm(0)*2 -_max_rpm - _min_rpm)/(_max_rpm - _min_rpm); 
  motor_commands(1) = (rpm(2)*2 -_max_rpm - _min_rpm)/(_max_rpm - _min_rpm);
  motor_commands(2) = (rpm(3)*2 -_max_rpm - _min_rpm)/(_max_rpm - _min_rpm);
  motor_commands(3) = (rpm(1)*2 -_max_rpm - _min_rpm)/(_max_rpm - _min_rpm);

  //motor_commands(0) = (rps(0) - 100.0f)/720.0f;//commands(1);
  //motor_commands(1) = (rps(2) - 100.0f)/720.0f;//commands(1);
  //motor_commands(2) = (rps(3) - 100.0f)/720.0f;//commands(2);
  //motor_commands(3) = (rps(1) - 100.0f)/720.0f;//commands(3);

  return motor_commands;

}
