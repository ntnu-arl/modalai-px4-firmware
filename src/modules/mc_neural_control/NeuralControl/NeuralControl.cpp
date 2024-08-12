#include <NeuralControl.hpp>
#include <mathlib/math/Functions.hpp>

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
  this->bias_control_net_layer_1 = openData(path + "bias_control_net_layer_1.csv");
  this->weight_control_net_layer_1 = openData(path + "weight_control_net_layer_1.csv");
  this->bias_control_net_layer_2 = openData(path + "bias_control_net_layer_2.csv");
  this->weight_control_net_layer_2 = openData(path + "weight_control_net_layer_2.csv");
  this->bias_control_net_layer_3 = openData(path + "bias_control_net_layer_3.csv");
  this->weight_control_net_layer_3 = openData(path + "weight_control_net_layer_3.csv");
  this->bias_allocation_net_layer_1 = openData(path + "bias_allocation_net_layer_1.csv");
  this->weight_allocation_net_layer_1 = openData(path + "weight_allocation_net_layer_1.csv");
  this->bias_allocation_net_layer_2 = openData(path + "bias_allocation_net_layer_2.csv");
  this->weight_allocation_net_layer_2 = openData(path + "weight_allocation_net_layer_2.csv");
  this->max_thrust = openData(path + "max_thrust.csv");
  this->min_thrust = openData(path + "min_thrust.csv");
  this->max_torque = openData(path + "max_torque.csv");
  this->min_torque = openData(path + "min_torque.csv");
  this->limits_u = openData(path + "lim_u.csv");
}

void NeuralControl::updateNeural(float& thrust_setpoint, Quatf& quaternion_setpoint) const
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(3,3);
  Eigen::MatrixXd B = Eigen::MatrixXd::Random(3,3);
  Eigen::MatrixXd C = A*B;
 
  // forward path
  Eigen::VectorXf input = Eigen::VectorXf::Random(16);//Eigen::VectorXf::Zero(16);
  Eigen::VectorXf co1 = this->weight_control_net_layer_1 * input + this->bias_control_net_layer_1;
  Eigen::VectorXf ca1 = co1.array().tanh();
  Eigen::VectorXf co2 = this->weight_control_net_layer_2 * ca1 + this->bias_control_net_layer_2;
  Eigen::VectorXf ca2 = co2.array().tanh();
  Eigen::VectorXf co3 = this->weight_control_net_layer_3 * ca2 + this->bias_control_net_layer_3;
  Eigen::VectorXf ca3 = co3.array().tanh();
  Eigen::VectorXf input_allocation_net = ca3;

  Eigen::VectorXf scaled_thrust = input_allocation_net(Eigen::seq(0,2)).cwiseProduct(this->max_thrust - this->min_thrust)/2 + (this->max_thrust + this->min_thrust)/2;
  Eigen::VectorXf scaled_torque = input_allocation_net(Eigen::seq(3,5)).cwiseProduct(this->max_torque - this->min_torque)/2 + (this->max_torque + this->min_torque)/2;

  Eigen::VectorXf scaled_input_allocation_net = Eigen::VectorXf::Zero(6);
  scaled_input_allocation_net << scaled_thrust, scaled_torque;

  Eigen::VectorXf ao1 = this->weight_allocation_net_layer_1 * scaled_input_allocation_net + this->bias_allocation_net_layer_1;
  Eigen::VectorXf aa1 = ao1.cwiseMax(0);
  Eigen::VectorXf ao2 = this->weight_allocation_net_layer_2 * aa1 + this->bias_allocation_net_layer_2;

}
