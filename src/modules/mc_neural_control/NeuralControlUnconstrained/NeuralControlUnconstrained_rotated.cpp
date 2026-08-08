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
  PX4_INFO("mc_neural_control_main entered");

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

inline Eigen::VectorXf sigmoid(const Eigen::VectorXf& x){
  return 1.0f / (1.0f + (-x.array()).exp());
}


// Gate order in PyTorch nn.GRU is: r, z, n
static inline Eigen::VectorXf gru_cell_step_pytorch(
  const Eigen::VectorXf& x_t,      // [I]
  const Eigen::VectorXf& h_prev,   // [H]
  const Eigen::MatrixXf& W_ih,     // [3H, I]
  const Eigen::MatrixXf& W_hh,     // [3H, H]
  const Eigen::VectorXf& b_ih,     // [3H]
  const Eigen::VectorXf& b_hh      // [3H]
) {
  const int H = static_cast<int>(h_prev.size());
  // if (W_ih.rows() != 3 * H || W_hh.rows() != 3 * H || b_ih.size() != 3 * H || b_hh.size() != 3 * H) {
  //   throw std::runtime_error("GRU param shapes inconsistent with hidden size H");
  // }
  // if (W_hh.cols() != H) {
  //   throw std::runtime_error("W_hh must be [3H, H]");
  // }
  // if (W_ih.cols() != x_t.size()) {
  //   throw std::runtime_error("W_ih must be [3H, I] where I == x_t.size()");
  // }

  // Split into r,z,n blocks (PyTorch order: r, z, n)
  const auto W_ih_r = W_ih.block(0,    0, H, W_ih.cols());
  const auto W_ih_z = W_ih.block(H,    0, H, W_ih.cols());
  const auto W_ih_n = W_ih.block(2*H,  0, H, W_ih.cols());

  const auto W_hh_r = W_hh.block(0,    0, H, W_hh.cols());
  const auto W_hh_z = W_hh.block(H,    0, H, W_hh.cols());
  const auto W_hh_n = W_hh.block(2*H,  0, H, W_hh.cols());

  const auto b_ih_r = b_ih.segment(0,   H);
  const auto b_ih_z = b_ih.segment(H,   H);
  const auto b_ih_n = b_ih.segment(2*H, H);

  const auto b_hh_r = b_hh.segment(0,   H);
  const auto b_hh_z = b_hh.segment(H,   H);
  const auto b_hh_n = b_hh.segment(2*H, H);

  // r_t = sigmoid(W_ir x + b_ir + W_hr h + b_hr)
  Eigen::VectorXf r = sigmoid(W_ih_r * x_t + b_ih_r + W_hh_r * h_prev + b_hh_r);

  // z_t = sigmoid(W_iz x + b_iz + W_hz h + b_hz)
  Eigen::VectorXf z = sigmoid(W_ih_z * x_t + b_ih_z + W_hh_z * h_prev + b_hh_z);

  // n_t = tanh(W_in x + b_in + r ⊙ (W_hn h + b_hn))
  Eigen::VectorXf n = (W_ih_n * x_t + b_ih_n).array()
                  + r.array() * (W_hh_n * h_prev + b_hh_n).array();
  n = n.array().tanh();


  // h_t = (1 - z) ⊙ n + z ⊙ h_prev   (PyTorch convention)
  Eigen::VectorXf h = (Eigen::VectorXf::Ones(H) - z).array() * n.array()
                    + z.array() * h_prev.array();
  return h;
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
    // _bias_layer_3 = openDataUnconstrained(path + "bias_layer_3.csv");
    // _weight_layer_3 = openDataUnconstrained(path + "weight_layer_3.csv");
    // _bias_layer_4 = openDataUnconstrained(path + "bias_layer_4.csv");
    // _weight_layer_4 = openDataUnconstrained(path + "weight_layer_4.csv");
    _gru_b_ih = openDataUnconstrained(path + "gru_b_ih.csv");
    _gru_w_ih = openDataUnconstrained(path + "gru_w_ih.csv");
    _gru_b_hh = openDataUnconstrained(path + "gru_b_hh.csv");
    _gru_w_hh = openDataUnconstrained(path + "gru_w_hh.csv");


    _bias_allocation_layer_1 = openDataUnconstrained(path + "bias_allocation_layer_1.csv");
    _weight_allocation_layer_1 = openDataUnconstrained(path + "weight_allocation_layer_1.csv");

    _bias_output_layer = openDataUnconstrained(path + "bias_output_layer.csv");
    _weight_output_layer = openDataUnconstrained(path + "weight_output_layer.csv");

    // _norm_weight = openDataUnconstrained(path + "norm_weight.csv");
    // _norm_bias = openDataUnconstrained(path + "norm_bias.csv");

    PX4_INFO("model files loaded!");

    // Debug State
    _scaled_input_allocation_net = Eigen::VectorXf::Zero(6);
    _motor_cmds = Eigen::VectorXf::Zero(6);
    _force_offset_comp = Eigen::VectorXf::Zero(4);
    _input = Eigen::VectorXf::Zero(49); // 15
    hidden_state = Eigen::VectorXf::Zero(50); // 15

    _static_obs.resize(36);
    _static_obs << -0.0000000116, 0.0000000002, 0.1666666567,
                    .0014827710, -0.0059679542, 0.0711167902,
                    -0.0000000119, 0.0000000019, 0.1666667610,
                    0.0054448424, -0.0016208383, -0.0747171715,
                    -0.0000000118, -0.0000000026, 0.1666666418,
                    0.0039620697, 0.0043471125, 0.0711188838,
                    -0.0000000115, -0.0000000002, 0.1666667163,
                    -0.0014827723, 0.0059679528, -0.0711167976,
                    -0.0000000112, -0.0000000019, 0.1666667014,
                    -0.0054448415, 0.0016208341, 0.0747171566,
                    -0.0000000113, 0.0000000026, 0.1666667014,
                    -0.0039620697, -0.0043471167, -0.0711188689;

    _motor_min_thrusts = Eigen::VectorXf::Constant(_n_motors, 0.05f);
    _motor_max_thrusts = Eigen::VectorXf::Constant(_n_motors, 1.7f);

    float deg = 15.0f; 
    float theta = math::radians(deg); 

    Rz_body = matrix::Dcmf(matrix::Eulerf(0.f, 0.f, theta));


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

  //PX4_INFO("got here 1");

  // transform observations in correct frame
  Vector3f position_local;
  position_local = _frame_transf * _frame_transf_2 * _position;

  Vector3f position_setpoint_local;
  position_setpoint_local = _frame_transf * _frame_transf_2 * _position_setpoint;

  Vector3f linear_velocity_setpoint_local;
  linear_velocity_setpoint_local = _frame_transf * _frame_transf_2 * _linear_velocity_setpoint;

  Vector3f linear_velocity_local;
  linear_velocity_local = _frame_transf * _frame_transf_2 * _linear_velocity;

  matrix::Dcmf attitude_mat(_attitude);
  matrix::Dcmf rotated_attitude = attitude_mat * Rz_body;
  
  matrix::Dcmf _attitude_local_mat = _frame_transf * (_frame_transf_2 * rotated_attitude) * _frame_transf.transpose();
  //matrix::Dcmf _attitude_local_mat = _frame_transf * (_frame_transf_2 * matrix::Dcmf(_attitude)) * _frame_transf.transpose();
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

  // --- velocity error in BODY frame (use PX4 matrix types) ---
  matrix::Vector3f vel_state_m{
    linear_velocity_local(0), linear_velocity_local(1), linear_velocity_local(2)
  };
  matrix::Vector3f vel_sp_m{
    linear_velocity_setpoint_local(0), linear_velocity_setpoint_local(1), linear_velocity_setpoint_local(2)
  };

  //vel_sp_m *= 0;
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

  // re-express in rotated body frame (consistent with attitude_mat * Rz_body)
  matrix::Vector3f angvel_bprime_m = Rz_body.transpose() * angvel_m;

  // copy into Eigen
  Eigen::Vector3f angular_velocity_state_b;
  angular_velocity_state_b << angvel_bprime_m(0), angvel_bprime_m(1), angvel_bprime_m(2);

  // input vector for network
  _unnorm_input = Eigen::VectorXf::Zero(49);
  // 0..2  position error (world)
  _unnorm_input.segment<3>(0) = pos_input_clamped;

  // 3..6  quaternion [x,y,z,w]
  _unnorm_input.segment<4>(3) = quat_input;

  // 7..9  velocity error (body)
  _unnorm_input.segment<3>(7) = vel_input_b;

  // 10..12 angular velocity error (body)
  _unnorm_input.segment<3>(10) = angular_velocity_state_b;
  _unnorm_input.segment<36>(13) = _static_obs;

  // normalize observations
  //Eigen::VectorXf 
  _input = _unnorm_input;
  
  //_input.segment<13>(0) << -0.046624f, -0.028190f, 0.042185f, 0.027167f, 0.011528f, 0.055297f, 0.998034f, 
  //             -0.069047f, -0.029921f, 0.042351f, -0.084985f, 0.031911f, -0.02201f;

  // PX4_INFO("input size %f %f %f %f %f %f %f ",double(_input(0)), double(_input(1)), double(_input(2)), double(_input(3)), double(_input(4)), double(_input(5)), double(_input(6)));
  // PX4_INFO("input size %f %f %f %f %f %f %f ",double(_input(7)), double(_input(8)), double(_input(9)), double(_input(10)), double(_input(11)), double(_input(12)), double(_input(13)));

  for (int i = 0; i < _input.size(); i++) {
      const float denom = sqrtf(_obs_var(i) + _obs_eps);
      float v = (_input(i) - _obs_mean(i)) / denom;

      // clamp to [-5, 5]
      // if (v > 5.f) v = 5.f;
      // else if (v < -5.f) v = -5.f;

      _input(i) = v;
  }
  Eigen::VectorXf main = _input.head(13);          // (13)
  Eigen::VectorXf side = _input.segment(13, 36);   // (36)
  // forward path
  Eigen::VectorXf co1 = _weight_layer_1 * main + _bias_layer_1;
  Eigen::VectorXf ca1 = elu(co1);
  //PX4_INFO("first layer done");
  Eigen::VectorXf co2 = _weight_layer_2 * ca1 + _bias_layer_2;
  Eigen::VectorXf ca2 = elu(co2);
  Eigen::VectorXf co1_ = _weight_allocation_layer_1 * side + _bias_allocation_layer_1;
  Eigen::VectorXf ca1_ = elu(co1_);

  Eigen::VectorXf cat(100); //cat(128);
  cat << ca2, ca1_;

  Eigen::VectorXf gru_output = gru_cell_step_pytorch(cat, hidden_state, _gru_w_ih, _gru_w_hh, _gru_b_ih, _gru_b_hh);
  Eigen::VectorXf output = _weight_output_layer * gru_output + _bias_output_layer;
  // IMPORTANT: update hidden for next tick
  hidden_state = gru_output;


  // Eigen::VectorXf co3 = _weight_layer_3 * cat + _bias_layer_3;
  // Eigen::VectorXf ca3 = elu(co3);
  // Eigen::VectorXf co4 = _weight_layer_4 * ca3 + _bias_layer_4;
  // Eigen::VectorXf ca4 = elu(co4);
  // Eigen::VectorXf output = _weight_output_layer * ca4 + _bias_output_layer;

  //Eigen::VectorXf output = Eigen::VectorXf::Zero(_n_motors);
  Eigen::VectorXf actions = output.cwiseMax(-1.f).cwiseMin(1.f);
  actions = (actions.array() + 1.f) * 0.5f;
  Eigen::VectorXf _forces_clamped_reverse(_n_motors);
  _forces_clamped_reverse =  _motor_min_thrusts + (_motor_max_thrusts - _motor_min_thrusts).cwiseProduct(actions);
  _force_clamped = Eigen::VectorXf::Zero(_n_motors);
  _force_clamped << _forces_clamped_reverse(5), _forces_clamped_reverse(4), _forces_clamped_reverse(3), _forces_clamped_reverse(2), _forces_clamped_reverse(1), _forces_clamped_reverse(0);

  // PX4_INFO("thrusts: %f %f %f %f %f %f", double(_force_clamped(0)), double(_force_clamped(1)), double(_force_clamped(2)), double(_force_clamped(3)), double(_force_clamped(4)), double(_force_clamped(5)));
  // conversion to rpm
  static const float _thrust_coefficient = 0.00002308; //0.00001286412;

  Eigen::VectorXf rps = Eigen::VectorXf::Zero(_n_motors);
  rps = _force_clamped / _thrust_coefficient;
  rps = rps.cwiseSqrt();
  Eigen::VectorXf rpm = rps * 60;

  // conversion to motor commands (inverse of the scaling done in mixer module)
  matrix::Vector<float,6> motor_commands;
  //rpm = rpm.reverse(); // reverse for correct motor order
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

  //PX4_INFO("got here 2");

  //PX4_INFO("mixer values %f %f %f %f %f %f",double(mixer_values(0)),double(mixer_values(1)),double(mixer_values(2)),double(mixer_values(3)),double(mixer_values(4)),double(mixer_values(5)));

  return mixer_values;
}


