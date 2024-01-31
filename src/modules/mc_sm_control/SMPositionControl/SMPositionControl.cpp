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

  const Vector3f gravity(0, 0, 9.80665);

  const Vector3f acceleration_cmd = (-_K_p_indi * error_position - _K_d_indi * error_velocity - _K_ff_indi * error_acceleration + _linear_acceleration_setpoint - gravity);
  //const Vector3f force_cmd = _mass * acceleration_cmd;

  // compute filtered acceleration
  Vector3f a_current_lp(_lp_filter_accel[0].apply(_linear_acceleration(0)),
                        _lp_filter_accel[1].apply(_linear_acceleration(1)),
                        _lp_filter_accel[2].apply(_linear_acceleration(2)));
  // compute nominal force
  const Vector3f e3(0, 0, 1);
  const float thrust_current = _thrust_coeff * (powf(_rpm1, 2.f) + powf(_rpm2, 2.f) + powf(_rpm3, 2.f) + powf(_rpm4, 2.f));
  const Vector3f f_current = -thrust_current * _attitude * e3;

  // apply low pass filter
  const Vector3f f_current_lp(_lp_filter_force[0].apply(f_current(0)),
                              _lp_filter_force[1].apply(f_current(1)),
                              _lp_filter_force[2].apply(f_current(2)));

  // compute incremental update
  const Vector3f f_command = _mass*(acceleration_cmd - a_current_lp) + f_current_lp;
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
