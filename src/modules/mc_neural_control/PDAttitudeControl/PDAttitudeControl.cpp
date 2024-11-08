#include <PDAttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

Vector3f PDAttitudeControl::updatePD() const
{
  const Matrix3f attitude_T = _attitude.transpose();
  const Matrix3f attitude_sp_T = _attitude_setpoint.transpose();

  const Vector3f error_attitude =
      0.5f * (matrix::Dcmf(attitude_sp_T * _attitude - attitude_T * _attitude_setpoint).vee());
  const Vector3f error_angular_velocity =
      _angular_velocity - attitude_T * _attitude_setpoint * _angular_velocity_setpoint;

  const Eulerf attitude(_attitude);
  const Eulerf attitude_setpoint(_attitude_setpoint); 

  printf("update pd\n");
  error_attitude.print();
  error_angular_velocity.print();
  _K_p.print();
  _K_d.print();

  const Matrix3f omega = _angular_velocity.hat();
  const Vector3f moment = -_K_p * error_attitude - _K_d * error_angular_velocity +
                          omega * _inertia * _angular_velocity -
                          _inertia * (omega * attitude_T * _attitude_setpoint * _angular_velocity_setpoint);

  // PX4_WARN("moment: %f %f %f", double(moment(0)), double(moment(1)), double(moment(2)));

  return moment;
}