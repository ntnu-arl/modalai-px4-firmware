#include <SMAttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

matrix::Vector3f SMAttitudeControl::update() const
{
	const matrix::Matrix3f attitude_T = _attitude.transpose();
	const matrix::Matrix3f attitude_sp_T = _attitude_setpoint.transpose();

	const matrix::Vector3f error_attitude = 0.5f* (matrix::Dcmf(attitude_sp_T * _attitude - attitude_T * _attitude_setpoint).vee());
	const matrix::Vector3f error_angular_velocity = _angular_velocity - attitude_T * _attitude_setpoint * _angular_velocity_setpoint;

	// TODO: make _lambda a vector
	const matrix::Vector3f sigma = error_angular_velocity + _lambda * error_attitude;

	const matrix::Matrix3f omega = _angular_velocity.hat();
	const matrix::Vector3f sign_sigma = signum(sigma);
  const matrix::Vector3f moment = omega * _inertia * _angular_velocity -
                                  _inertia * (omega * attitude_T * _attitude_setpoint * _angular_velocity_setpoint +
                                              _lambda * error_angular_velocity + _switching_gain * sign_sigma);

	return moment;
}
