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

inline Eigen::VectorXf elu(const Eigen::VectorXf& x)
{
    return x.array().max(0.0f)
         + (x.array().min(0.0f).exp() - 1.0f);
}

inline Eigen::VectorXf layernorm(
    const Eigen::VectorXf& x,
    const Eigen::VectorXf& gamma,
    const Eigen::VectorXf& beta,
    float eps = 1e-5f
) {
    const float mean = x.mean();
    const float var = (x.array() - mean).square().mean();

    Eigen::VectorXf x_hat = (x.array() - mean) / std::sqrt(var + eps);
    return gamma.array() * x_hat.array() + beta.array();
}

NeuralControlUnconstrained::NeuralControlUnconstrained(int n_motors)
{
  _n_motors = n_motors;

  try
  {
    std::string path = "/home/model_files_unconstrained/";
    PX4_INFO("loading model files");
    _obs_mean = openDataUnconstrained(path + "running_mean_std_mean.csv");
    _obs_var = openDataUnconstrained(path + "running_mean_std_var.csv");
    _obs_eps = 1e-5f;

    _bias_layer_1 = openDataUnconstrained(path + "bias_layer_1.csv");
    _weight_layer_1 = openDataUnconstrained(path + "weight_layer_1.csv");
    _bias_layer_2 = openDataUnconstrained(path + "bias_layer_2.csv");
    _weight_layer_2 = openDataUnconstrained(path + "weight_layer_2.csv");
    _bias_layer_3 = openDataUnconstrained(path + "bias_layer_3.csv");
    _weight_layer_3 = openDataUnconstrained(path + "weight_layer_3.csv");
    _bias_layer_4 = openDataUnconstrained(path + "bias_layer_4.csv");
    _weight_layer_4 = openDataUnconstrained(path + "weight_layer_4.csv");

    _bias_allocation_layer_1 = openDataUnconstrained(path + "bias_allocation_layer_1.csv");
    _weight_allocation_layer_1 = openDataUnconstrained(path + "weight_allocation_layer_1.csv");

    _bias_output_layer = openDataUnconstrained(path + "bias_output_layer.csv");
    _weight_output_layer = openDataUnconstrained(path + "weight_output_layer.csv");

    _norm_weight = openDataUnconstrained(path + "norm_weight.csv");
    _norm_bias = openDataUnconstrained(path + "norm_bias.csv");

    PX4_INFO("model files loaded!");

    // Debug State
    _scaled_input_allocation_net = Eigen::VectorXf::Zero(6);
    _motor_cmds = Eigen::VectorXf::Zero(6);
    _force_offset_comp = Eigen::VectorXf::Zero(4);
    _input = Eigen::VectorXf::Zero(61); // 15

    _static_obs.resize(48);
    _static_obs << 1.7820054293f,      1.9009206295,
          0.5512463450,     -0.0218369160,     -0.0267387982,
          0.1366616040,      0.8021930456,      0.7921100259,
          0.1790838093,      0.0146056293,      0.0078212060,
          -0.0686258152,     -3.1968226433,     -1.4655148983,
          -0.1597105563,      0.0127784461,      0.0024540315,
          0.0222310014,      3.4531297684,      0.3609554470,
          0.6219188571,     -0.0135005787,      0.0072640372,
          0.0100276815,     -0.5797182918,      4.3422651291,
          0.1995685995,     -0.0213653855,     -0.0125859659,
          0.1271253228,     -2.3787565231,     -5.7737803459,
          -0.3825006187,      0.0288977493,      0.0215753485,
          -0.2248043418,      0.0149016120,      0.0211659707,
          0.0337398686,      0.0480504110,      0.0363027342,
          0.0297534037,      0.0000128641,      0.0000128641,
          0.0000128641,      0.0000128641,      0.0000128641,
          0.0000128641;

    _motor_min_thrusts = Eigen::VectorXf::Constant(_n_motors, 0.05f);
    _motor_max_thrusts = Eigen::VectorXf::Constant(_n_motors, 1.7f);

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
    message.motor_thrust[i] = _motor_cmds(i); //_force_clamped(i);
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

  PRINT_INFO()

  // transform observations in correct frame
  Vector3f position_local;
  position_local = _frame_transf * _frame_transf_2 * _position;

  Vector3f position_setpoint_local;
  position_setpoint_local = _frame_transf * _frame_transf_2 * _position_setpoint;

  Vector3f linear_velocity_setpoint_local;
  linear_velocity_setpoint_local = _frame_transf * _frame_transf_2 * _linear_velocity_setpoint;

  Vector3f linear_velocity_local;
  linear_velocity_local = _frame_transf * _frame_transf_2 * _linear_velocity;

  matrix::Dcmf _attitude_local_mat = _frame_transf * (_frame_transf_2 * matrix::Dcmf(_attitude)) * _frame_transf.transpose();
  matrix::Eulerf euler_angles_local(_attitude_local_mat);
  matrix::Quatf q_local(_attitude_local_mat); // [w,x,y,z]

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
  // Eigen::Vector3f vel_state;
  // vel_state << linear_velocity_local(0), linear_velocity_local(1), linear_velocity_local(2);
  // Eigen::Vector3f vel_setpoint;
  // vel_setpoint << linear_velocity_setpoint_local(0), linear_velocity_setpoint_local(1), linear_velocity_setpoint_local(2);
  // Eigen::Vector3f vel_input = vel_setpoint - vel_state;
  // Vector3f vel_input_b = _attitude_local_mat.transpose() * vel_input;
  // --- velocity error in BODY frame (use PX4 matrix types) ---
  matrix::Vector3f vel_state_m{
    linear_velocity_local(0), linear_velocity_local(1), linear_velocity_local(2)
  };
  matrix::Vector3f vel_sp_m{
    linear_velocity_setpoint_local(0), linear_velocity_setpoint_local(1), linear_velocity_setpoint_local(2)
  };

  matrix::Vector3f vel_err_m = vel_sp_m - vel_state_m;
  matrix::Vector3f vel_err_b_m = _attitude_local_mat.transpose() * vel_err_m;
  // copy into Eigen for the NN input
  Eigen::Vector3f vel_input_b;
  vel_input_b << vel_err_b_m(0), vel_err_b_m(1), vel_err_b_m(2);

  Eigen::VectorXf quat_input(4);
  quat_input << q_local(1), q_local(2), q_local(3), q_local(0);

  // convert angular velocities
  // Eigen::Vector3f angular_velocity_state;
  // angular_velocity_state << angular_vel_local(0), angular_vel_local(1), angular_vel_local(2);
  // Vector3f angular_velocity_state_b = _attitude_local_mat.transpose() * angular_velocity_state;
  // --- angular velocity in BODY frame (use PX4 matrix types) ---
  matrix::Vector3f angvel_m{
    angular_vel_local(0), angular_vel_local(1), angular_vel_local(2)
  };
  matrix::Vector3f angvel_b_m = _attitude_local_mat.transpose() * angvel_m;

  // copy into Eigen
  Eigen::Vector3f angular_velocity_state_b;
  angular_velocity_state_b << angvel_b_m(0), angvel_b_m(1), angvel_b_m(2);

  // input vector for network
  _unnorm_input = Eigen::VectorXf::Zero(61);
  // 0..2  position error (world)
  _unnorm_input.segment<3>(0) = pos_input_clamped;

  // 3..6  quaternion [x,y,z,w]
  _unnorm_input.segment<4>(3) = quat_input;

  // 7..9  velocity error (body)
  _unnorm_input.segment<3>(7) = vel_input_b;

  // 10..12 angular velocity error (body)
  _unnorm_input.segment<3>(10) = angular_velocity_state_b;
  _unnorm_input.segment<48>(13) = _static_obs;



  // normalize observations
  //Eigen::VectorXf 
  _input = _unnorm_input;
  for (int i = 0; i < _input.size(); i++) {
      const float denom = sqrtf(_obs_var(i) + _obs_eps);
      float v = (_input(i) - _obs_mean(i)) / denom;

      // clamp to [-5, 5]
      if (v > 5.f) v = 5.f;
      else if (v < -5.f) v = -5.f;

      _input(i) = v;
  }
  Eigen::VectorXf main = _input.head(13);          // (13)
  Eigen::VectorXf side = _input.segment(13, 48);   // (48)

  // forward path
  Eigen::VectorXf co1 = _weight_layer_1 * main + _bias_layer_1;
  Eigen::VectorXf ca1 = elu(co1);
  Eigen::VectorXf co2 = _weight_layer_2 * ca1 + _bias_layer_2;
  Eigen::VectorXf ca2 = elu(co2);
  Eigen::VectorXf co1_ = _weight_allocation_layer_1 * side + _bias_allocation_layer_1;
  Eigen::VectorXf ca1_ = elu(co1_);

  Eigen::VectorXf cat(128);
  cat << ca2, ca1_;
  // LAYERNORM HERE
  cat = layernorm(cat, _norm_weight, _norm_bias);

  Eigen::VectorXf co3 = _weight_layer_3 * cat + _bias_layer_3;
  Eigen::VectorXf ca3 = elu(co3);
  Eigen::VectorXf co4 = _weight_layer_4 * ca3 + _bias_layer_4;
  Eigen::VectorXf ca4 = elu(co4);
  Eigen::VectorXf output = _weight_output_layer * ca4 + _bias_output_layer;


  Eigen::VectorXf actions = output.cwiseMax(-1.f).cwiseMin(1.f);
  actions = (actions.array() + 1.f) * 0.5f;
  _force_clamped = Eigen::VectorXf::Zero(_n_motors);
  _force_clamped =  _motor_min_thrusts + (_motor_max_thrusts - _motor_min_thrusts).cwiseProduct(actions);


  // conversion to rpm
  static const float _thrust_coefficient = 0.00001286412;

  Eigen::VectorXf rpm = Eigen::VectorXf::Zero(_n_motors);
  rpm = _force_clamped / _thrust_coefficient;
  rpm = rpm.cwiseSqrt();
  //Eigen::VectorXf rpm = rps * 60;

  // conversion to motor commands (inverse of the scaling done in mixer module)
  matrix::Vector<float,6> motor_commands;
  rpm = rpm.reverse(); // reverse for correct motor order
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
    // what is going on here??
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

  _debug(0) = pos_setpoint(0);
  _debug(1) = pos_setpoint(1);
  _debug(2) = pos_setpoint(2);
  _debug(3) = linear_velocity_setpoint_local(0);
  _debug(4) = linear_velocity_setpoint_local(1);
  _debug(5) = linear_velocity_setpoint_local(2);

  return mixer_values;
}


