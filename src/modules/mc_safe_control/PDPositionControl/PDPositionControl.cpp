#include <PDPositionControl.hpp>

#include <mathlib/math/Functions.hpp>

Vector3f PDPositionControl::calculateAccelerationPD() const
{
  const Vector3f error_position = _position - _position_setpoint;
  const Vector3f error_velocity = _linear_velocity - _linear_velocity_setpoint;

  const Vector3f gravity(0, 0, 9.80665);

  const Vector3f acceleration =
      -_K_p * error_position - _K_d * error_velocity + _mass * (_linear_acceleration_setpoint - gravity);

  return acceleration;
}

float PDPositionControl::calculateThrust(const Vector3f& acceleration) const
{
  const Vector3f e3(0, 0, 1);
  // TODO: make cleaner
  return -(acceleration.T() * _attitude * e3)(0, 0);
}

Quatf PDPositionControl::calculateAttitude(const Vector3f& acceleration) const
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

  return Quatf(Dcmf(result));
}

void PDPositionControl::updatePD(Vector3f& acceleration_setpoint) const
{
  acceleration_setpoint = calculateAccelerationPD();
}
