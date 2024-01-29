#include <SMAttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

Vector3f SMAttitudeControl::updatePD() const
{
  const Matrix3f attitude_T = _attitude.transpose();
  const Matrix3f attitude_sp_T = _attitude_setpoint.transpose();

  const Vector3f error_attitude =
      0.5f * (matrix::Dcmf(attitude_sp_T * _attitude - attitude_T * _attitude_setpoint).vee());
  const Vector3f error_angular_velocity =
      _angular_velocity - attitude_T * _attitude_setpoint * _angular_velocity_setpoint;

  printf("update pd\n");
  error_attitude.print();
  error_angular_velocity.print();
  _K_p.print();
  _K_d.print();

  const Matrix3f omega = _angular_velocity.hat();
  const Vector3f moment = -_K_p * error_attitude - _K_d * error_angular_velocity +
                          omega * _inertia * _angular_velocity -
                          _inertia * (omega * attitude_T * _attitude_setpoint * _angular_velocity_setpoint);

  return moment;
}

matrix::Vector3f SMAttitudeControl::updateSM() const
{
  const matrix::Matrix3f attitude_T = _attitude.transpose();
  const matrix::Matrix3f attitude_sp_T = _attitude_setpoint.transpose();

  const matrix::Vector3f error_attitude =
      0.5f * (matrix::Dcmf(attitude_sp_T * _attitude - attitude_T * _attitude_setpoint).vee());
  const matrix::Vector3f error_angular_velocity =
      _angular_velocity - attitude_T * _attitude_setpoint * _angular_velocity_setpoint;

  // TODO: make _lambda a vector
  const matrix::Vector3f sigma = error_angular_velocity + _lambda * error_attitude;

  const matrix::Matrix3f omega = _angular_velocity.hat();
  const matrix::Vector3f sign_sigma = signum(sigma);
  const matrix::Vector3f moment = omega * _inertia * _angular_velocity -
                                  _inertia * (omega * attitude_T * _attitude_setpoint * _angular_velocity_setpoint +
                                              _lambda * error_angular_velocity + _switching_gain * sign_sigma);

  return moment;
}
