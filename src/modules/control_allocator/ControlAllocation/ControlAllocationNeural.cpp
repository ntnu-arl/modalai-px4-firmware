#include "ControlAllocationNeural.hpp"

#include <iostream>
#include <memory>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <math.h>

Eigen::MatrixXf loadData(std::string fileToOpen)
{

  std::vector<float> matrixEntries;

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
  std::string matrixRowString;
  std::string matrixEntry;

  int matrixRowNumber = 0;

  try
  {
    while (getline(matrixDataFile, matrixRowString))
    {
      std::stringstream matrixRowStringStream(matrixRowString);

      while (getline(matrixRowStringStream, matrixEntry, ','))
      {
        matrixEntries.push_back(stod(matrixEntry));
      }
      matrixRowNumber++;
    }
  }
  catch (const std::exception &e)
  {
    PX4_INFO("Error: while processing model files");
    std::cerr << "Error: while processing model files" << std::endl;
  }
  return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
}

ControlAllocationNeural::ControlAllocationNeural(){

	std::string path = "/home/model_files/";
	_bias_allocation_net_layer_1 = loadData(path + "bias_allocation_net_layer_1.csv");
    _weight_allocation_net_layer_1 = loadData(path + "weight_allocation_net_layer_1.csv");
    _bias_allocation_net_layer_2 = loadData(path + "bias_allocation_net_layer_2.csv");
    _weight_allocation_net_layer_2 = loadData(path + "weight_allocation_net_layer_2.csv");
	PX4_INFO("model files loaded!");
}

void
ControlAllocationNeural::allocate()
{
	_prev_actuator_sp = _actuator_sp;
	
	matrix::Vector<float, NUM_AXES> normalized_input = _control_sp - _control_trim;

	Eigen::VectorXf normalized_input_allocation_net(6);
	normalized_input_allocation_net << normalized_input(0), normalized_input(1), normalized_input(2), normalized_input(3), normalized_input(4), normalized_input(5);

	Eigen::VectorXf input_allocation_net = scale_wrench(normalized_input_allocation_net);

	Eigen::VectorXf output_allocation_net = forwardPath(input_allocation_net);

	matrix::Vector<float, NUM_ACTUATORS> output;
	output(0) = output_allocation_net(0);
	output(1) = output_allocation_net(1);
	output(2) = output_allocation_net(2);
	output(3) = output_allocation_net(3);
	output(4) = output_allocation_net(4);
	output(5) = output_allocation_net(5);

	// Allocate
	_actuator_sp = _actuator_trim + output;
}

Eigen::VectorXf
ControlAllocationNeural::forwardPath(Eigen::VectorXf input)
{

	Eigen::VectorXf ao1 = _weight_allocation_net_layer_1 * input + _bias_allocation_net_layer_1;
  	// Eigen::VectorXf aa1 = ao1.cwiseMax(0); // change when supervised control allocation is used
  	Eigen::VectorXf ao2 = _weight_allocation_net_layer_2 * ao1 + _bias_allocation_net_layer_2;

	return ao2;
}

Eigen::VectorXf ControlAllocationNeural::scale_wrench(Eigen::VectorXf wrench)
{
	Eigen::VectorXf scaled_wrench(6);
	scaled_wrench = wrench;
	return scaled_wrench;
}
