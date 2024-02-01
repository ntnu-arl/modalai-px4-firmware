#include <SMPositionControl.hpp>
#include <mathlib/math/Functions.hpp>

Vector3f SMPositionControl::calculateAccelerationPD() const
{
  const Vector3f error_position = _position - _position_setpoint;
  const Vector3f error_velocity = _linear_velocity - _linear_velocity_setpoint;

  const Vector3f gravity(0, 0, 9.80665);

  const Vector3f acceleration =
      -_K_p * error_position - _K_d * error_velocity + _mass * (_linear_acceleration_setpoint - gravity);

  return acceleration;
}

Vector3f SMPositionControl::calculateAccelerationSM() const
{
  const Vector3f error_position = _position - _position_setpoint;
  const Vector3f error_velocity = _linear_velocity - _linear_velocity_setpoint;

  // TODO: make _lambda a vector
  const Vector3f sigma = error_velocity + _lambda * error_position;

  const Vector3f gravity(0, 0, 9.80665);

  const Vector3f acceleration =
      _mass * (_linear_acceleration_setpoint - gravity - _lambda * error_velocity - _switching_gain * signum(sigma));

  return acceleration;
}

Vector3f SMPositionControl::calculateAccelerationINDI()
{
  const Vector3f error_position = _position - _position_setpoint;
  const Vector3f error_velocity = _linear_velocity - _linear_velocity_setpoint;
  const Vector3f error_acceleration = _linear_acceleration - _linear_acceleration_setpoint;

  const Vector3f acceleration_cmd = -_K_p_indi * error_position - _K_d_indi * error_velocity - _K_ff_indi * error_acceleration + _linear_acceleration_setpoint;

  // compute nominal force
  const Vector3f gravity(0, 0, 9.80665);
  const Vector3f e3(0, 0, 1);
  // const float thrust_current = float(_thrust_coeff * double(powf(_rpm1, 2.f) + powf(_rpm2, 2.f) + powf(_rpm3, 2.f) + powf(_rpm4, 2.f)));
  // const Vector3f f_current = -thrust_current * _attitude * e3;
  // const Vector3f f_current = -_prev_thrust * _attitude * e3;

  // // apply low pass filter
  // const Vector3f f_current_lp(_lp_filter_force[0].apply(f_current(0)),
  //                             _lp_filter_force[1].apply(f_current(1)),
  //                             _lp_filter_force[2].apply(f_current(2)));
  // const Vector3f f_current = -_thrust_current * _attitude * e3;
  // const Vector3f f_current_lp = -_thrust_current_lp * _attitude * e3;

  // compute incremental update
  Vector3f f_command = _mass * (acceleration_cmd - _linear_acceleration) + _f_current_lp;
  // f_command = _mass * (acceleration_cmd - _linear_acceleration) + f_current;
  // f_command = _mass * (acceleration_cmd - gravity - (_linear_acceleration));
  // f_command = _mass * (acceleration_cmd - _linear_acceleration) + f_current_lp;

  printf("measurements/setpoints\n");
  _position.print();
  _position_setpoint.print();
  _linear_velocity.print();
  _linear_velocity_setpoint.print();
  _linear_acceleration.print();
  _linear_acceleration_setpoint.print();
  // _K_p_indi.print();
  // _K_d_indi.print();
  // // _K_ff_indi.print();
  printf("errors\n");
  error_position.print();
  error_velocity.print();
  error_acceleration.print();
  printf("control laws\n");
  printf("Kp*error_position");
  (-_K_p_indi * error_position).print();
  printf("kd*error_velocity");
  (-_K_d_indi * error_velocity).print();
  printf("kff");
  (- _K_ff_indi * error_acceleration).print();

  acceleration_cmd.print();
  _linear_acceleration.print();
  _f_current.print();
  _f_current_lp.print();

  // printf("rpm: %f %f %f %f", double(_rpm1), double(_rpm2), double(_rpm3), double(_rpm4));
  printf("specific thrust: %f\tf_current f_current_lp f_command: \n", double(_thrust_current_lp));
  // f_current.print();
  // f_current_lp.print();
  f_command.print();
  // _prev_thrust = f_command.norm();

  return f_command;
}

float SMPositionControl::calculateThrust(const Vector3f& acceleration) const
{
  const Vector3f e3(0, 0, 1);
  // TODO: make cleaner
  return -(acceleration.T() * _attitude * e3)(0, 0);
}

Dcmf SMPositionControl::calculateAttitude(const Vector3f& acceleration) const
{
  const Vector3f b3 = -acceleration / (acceleration.norm() + 1e-6f);
  const Vector3f b1_init(std::cos(_yaw_setpoint), std::sin(_yaw_setpoint), 0.0f);

  const Vector3f b2_init = b3.cross(b1_init);

  const Vector3f b2 = b2_init / (b2_init.norm() + 1e-6f);
  const Vector3f b1 = b2.cross(b3);

  Matrix3f result;
  for (auto i = 0; i < 3; ++i)
  {
    result(i, 0) = b1(i);
    result(i, 1) = b2(i);
    result(i, 2) = b3(i);
  }

  return Dcmf(result);
}


void SMPositionControl::updatePD(float& thrust_setpoint, Quatf& quaternion_setpoint)
{
  const Vector3f acceleration = calculateAccelerationPD();

  // ==============================================================================================================
  // apply low pass filter to the control signal.
  // This introduces some unwanted time delay but it attenuates high frequency noise fed to the attitude controller
  Vector3f acceleration_lp;
  if (_filter_position) {
    acceleration_lp = Vector3f(_lp_filter_position[0].apply(acceleration(0)),
                              _lp_filter_position[1].apply(acceleration(1)),
                              _lp_filter_position[2].apply(acceleration(2)));
  }
  else {
    acceleration_lp = acceleration;
  }
  // ==============================================================================================================

  thrust_setpoint = calculateThrust(acceleration_lp);
  const Dcmf attitude_setpoint = calculateAttitude(acceleration_lp);

  quaternion_setpoint = Quatf(attitude_setpoint);
}

void SMPositionControl::updateSM(float& thrust_setpoint, Quatf& quaternion_setpoint)
{
  const Vector3f acceleration = calculateAccelerationSM();

  thrust_setpoint = calculateThrust(acceleration);
  const Dcmf attitude_setpoint = calculateAttitude(acceleration);

  quaternion_setpoint = Quatf(attitude_setpoint);
}

void SMPositionControl::updateINDI(float& thrust_setpoint, Quatf& quaternion_setpoint)
{
  const Vector3f acceleration = calculateAccelerationINDI();

  thrust_setpoint = calculateThrust(acceleration);
  const Dcmf attitude_setpoint = calculateAttitude(acceleration);

  quaternion_setpoint = Quatf(attitude_setpoint);
}
