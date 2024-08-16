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
	std::ifstream matrixDataFile(fileToOpen);

	// this variable is used to store the row of the matrix that contains commas 
	std::string matrixRowString;

	// this variable is used to store the matrix entry;
	std::string matrixEntry;

	// this variable is used to track the number of rows
	int matrixRowNumber = 0;


	while (getline(matrixDataFile, matrixRowString)) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
	{
		std::stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.

		while (getline(matrixRowStringStream, matrixEntry, ',')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
		{
			matrixEntries.push_back(stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries
		}
		matrixRowNumber++; //update the column numbers
	}

	// here we convet the vector variable into the matrix and return the resulting object, 
	// note that matrixEntries.data() is the pointer to the first memory location at which the entries of the vector matrixEntries are stored;
	return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);

}

NeuralControl::NeuralControl()
{
  std::string path = "/usr/local/workspace/px4-firmware/src/modules/mc_neural_control/model_files/";
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
  _min_u_training = float(limits_u(0));
  _max_u_training = float(limits_u(1));
}

matrix::Vector4f NeuralControl::updateNeural() const
{
  //get state positions
  Eigen::Vector3f pos_state;
  pos_state << _position(0),_position(1),_position(2);
  Eigen::Vector3f pos_setpoint;
  pos_setpoint << _position_setpoint(0),_position_setpoint(1),_position_setpoint(2);
  Eigen::Vector3f pos_input = pos_state - pos_setpoint; 

  //convert linear velocities
  Eigen::Vector3f vel_state;
  vel_state << _linear_velocity(0),_linear_velocity(1),_linear_velocity(2);

  //convert attitude
  matrix::Quatf q_attitude(_attitude);
  Eigen::Vector4f attitude_state;
  attitude_state << q_attitude(3),q_attitude(1),q_attitude(2),q_attitude(0);
  std::cout << "Double check that quaternion are in the correct order!!!" << std::endl;

  //convert angular velocities
  Eigen::Vector3f angular_velocity_state;
  angular_velocity_state << _angular_velocity(0),_angular_velocity(1),_angular_velocity(2);

  // zeros used for goal position during training
  Eigen::Vector3f zeros = Eigen::Vector3f::Zero(3);

  // input vector for network
  Eigen::VectorXf input(pos_state.size() + attitude_state.size() + vel_state.size() + angular_velocity_state.size() + zeros.size());
  input << pos_input, attitude_state, vel_state, angular_velocity_state, zeros;

  // forward path
  Eigen::VectorXf co1 = _weight_control_net_layer_1 * input + _bias_control_net_layer_1;
  Eigen::VectorXf ca1 = co1.array().tanh();
  Eigen::VectorXf co2 = _weight_control_net_layer_2 * ca1 + _bias_control_net_layer_2;z
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

  Eigen::VectorXf u_clamped = ao2.cwiseMax(_min_u_training).cwiseMin(_max_u_training);
  std::cout << "Double check that clamping works!" << std::endl;

  Eigen::VectorXf scaled_motor_thrust = (u_clamped.array() - _min_u_deploy) / (_max_u_deploy - _min_u_deploy);
  std::cout << "Double check that scaling works" << std::endl;

  matrix::Vector4f motor_commands;
  motor_commands(0) = scaled_motor_thrust(0);
  motor_commands(1) = scaled_motor_thrust(1);
  motor_commands(2) = scaled_motor_thrust(2);
  motor_commands(3) = scaled_motor_thrust(3);

  return motor_commands;

}
