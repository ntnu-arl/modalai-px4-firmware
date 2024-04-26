/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ActuatorEffectivenessRotors.hpp
 *
 * Actuator effectiveness computed from rotors position and orientation
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessRotors.hpp"

#include "ActuatorEffectivenessTilts.hpp"

using namespace matrix;

ActuatorEffectivenessRotors::ActuatorEffectivenessRotors(ModuleParams *parent, AxisConfiguration axis_config,
		bool tilt_support, bool flexible_support)
	: ModuleParams(parent), _axis_config(axis_config), _tilt_support(tilt_support), _flexible_support(flexible_support)
{
	for (int i = 0; i < NUM_ROTORS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PX", i);
		_param_handles[i].position_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PY", i);
		_param_handles[i].position_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PZ", i);
		_param_handles[i].position_z = param_find(buffer);

		if (_axis_config == AxisConfiguration::Configurable) {
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AX", i);
			_param_handles[i].axis_x = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AY", i);
			_param_handles[i].axis_y = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AZ", i);
			_param_handles[i].axis_z = param_find(buffer);
		}

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_CT", i);
		_param_handles[i].thrust_coef = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_KM", i);
		_param_handles[i].moment_ratio = param_find(buffer);

		if (_tilt_support) {
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_TILT", i);
			_param_handles[i].tilt_index = param_find(buffer);
		}

		if (_flexible_support) {
			// sensor position wrt COG
			snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_PX", i);
			_param_handles[i].sensor_x = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_PY", i);
			_param_handles[i].sensor_y = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_PZ", i);
			_param_handles[i].sensor_z = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_SENSOR%u_YAW", i);
			_param_handles[i].sensor_yaw = param_find(buffer);
		}
	}

	_count_handle = param_find("CA_ROTOR_COUNT");

	updateParams();
}

void ActuatorEffectivenessRotors::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_count_handle, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_geometry.num_rotors = math::min(NUM_ROTORS_MAX, (int)count);

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		Vector3f &position = _geometry.rotors[i].position;
		param_get(_param_handles[i].position_x, &position(0));
		param_get(_param_handles[i].position_y, &position(1));
		param_get(_param_handles[i].position_z, &position(2));

		Vector3f &axis = _geometry.rotors[i].axis;

		switch (_axis_config) {
		case AxisConfiguration::Configurable:
			param_get(_param_handles[i].axis_x, &axis(0));
			param_get(_param_handles[i].axis_y, &axis(1));
			param_get(_param_handles[i].axis_z, &axis(2));
			break;

		case AxisConfiguration::FixedForward:
			axis = Vector3f(1.f, 0.f, 0.f);
			break;

		case AxisConfiguration::FixedUpwards:
			axis = Vector3f(0.f, 0.f, -1.f);
			break;
		}

		param_get(_param_handles[i].thrust_coef, &_geometry.rotors[i].thrust_coef);
		param_get(_param_handles[i].moment_ratio, &_geometry.rotors[i].moment_ratio);

		if (_tilt_support) {
			int32_t tilt_param{0};
			param_get(_param_handles[i].tilt_index, &tilt_param);
			_geometry.rotors[i].tilt_index = tilt_param - 1;

		} else {
			_geometry.rotors[i].tilt_index = -1;
		}

		if (_flexible_support) {
			Vector3f &O_t_OS = _geometry.sensors[i].O_t_OS;
			param_get(_param_handles[i].sensor_x, &O_t_OS(0));
			param_get(_param_handles[i].sensor_y, &O_t_OS(1));
			param_get(_param_handles[i].sensor_z, &O_t_OS(2));

			// PX4_INFO("O_t_OS: %f %f %f ",(double)O_t_OS(0), (double)O_t_OS(1), (double)O_t_OS(2) );

			_geometry.sensors[i].O_t_OR = _geometry.rotors[i].position;
			_geometry.sensors[i].axis = _geometry.rotors[i].axis;

			// create rotation for nominal case
			// asuming pitch and roll are 0
			float yaw = 0;
			param_get(math::radians(_param_handles[i].sensor_yaw), &yaw);
			// const float tilt_direction = math::radians((float)tilt.tilt_direction);

			param_get(_param_handles[i].sensor_yaw, &yaw);

			_geometry.sensors[i].O_rot_S = matrix::Quaternionf(matrix::Eulerf(0.0f, 0.0f, math::radians(yaw)));
			// _geometry.sensors[i].O_rot_S =
			//     matrix::Quaternionf(matrix::Vector3f(1.0f, 0.0f, 0.0f), matrix::Vector3f(sensor(0), sensor(1), 0.0f));

			// position of rotor with respect to sensor
			Vector3f &S_t_SR = _geometry.sensors[i].S_t_SR;
			const Vector3f &O_t_OR = _geometry.rotors[i].position;
			const Vector3f O_t_SR = O_t_OR - O_t_OS;
			S_t_SR = _geometry.sensors[i].O_rot_S.rotateVectorInverse(O_t_SR);
		}
	}
}

bool
ActuatorEffectivenessRotors::addActuators(Configuration &configuration)
{
	if (configuration.num_actuators[(int)ActuatorType::SERVOS] > 0) {
		PX4_ERR("Wrong actuator ordering: servos need to be after motors");
		return false;
	}

	int num_actuators = computeEffectivenessMatrix(_geometry,
			    configuration.effectiveness_matrices[configuration.selected_matrix],
			    configuration.num_actuators_matrix[configuration.selected_matrix]);
	configuration.actuatorsAdded(ActuatorType::MOTORS, num_actuators);
	return true;
}

int
ActuatorEffectivenessRotors::computeEffectivenessMatrix(const Geometry &geometry,
		EffectivenessMatrix &effectiveness, int actuator_start_index)
{
	int num_actuators = 0;

	for (int i = 0; i < geometry.num_rotors; i++) {

		if (i + actuator_start_index >= NUM_ACTUATORS) {
			break;
		}

		++num_actuators;

		// Get rotor axis
		Vector3f axis = geometry.rotors[i].axis;

		// Normalize axis
		float axis_norm = axis.norm();

		if (axis_norm > FLT_EPSILON) {
			axis /= axis_norm;

		} else {
			// Bad axis definition, ignore this rotor
			continue;
		}

		// Get rotor position
		const Vector3f &position = geometry.rotors[i].position;

		// Get coefficients
		float ct = geometry.rotors[i].thrust_coef;
		float km = geometry.rotors[i].moment_ratio;

		if (geometry.propeller_torque_disabled) {
			km = 0.f;
		}

		if (geometry.propeller_torque_disabled_non_upwards) {
			bool upwards = fabsf(axis(0)) < 0.1f && fabsf(axis(1)) < 0.1f && axis(2) < -0.5f;

			if (!upwards) {
				km = 0.f;
			}
		}

		if (fabsf(ct) < FLT_EPSILON) {
			continue;
		}

		// Compute thrust generated by this rotor
		matrix::Vector3f thrust = ct * axis;

		// Compute moment generated by this rotor
		matrix::Vector3f moment = ct * position.cross(axis) - ct * km * axis;

		// Fill corresponding items in effectiveness matrix
		for (size_t j = 0; j < 3; j++) {
			effectiveness(j, i + actuator_start_index) = moment(j);
			effectiveness(j + 3, i + actuator_start_index) = thrust(j);
		}

		if (geometry.yaw_by_differential_thrust_disabled) {
			// set yaw effectiveness to 0 if yaw is controlled by other means (e.g. tilts)
			effectiveness(2, i + actuator_start_index) = 0.f;
		}

		if (geometry.three_dimensional_thrust_disabled) {
			// Special case tiltrotor: instead of passing a 3D thrust vector (that would mostly have a x-component in FW, and z in MC),
			// pass the vector magnitude as z-component, plus the collective tilt. Passing 3D thrust plus tilt is not feasible as they
			// can't be allocated independently, and with the current controller it's not possible to have collective tilt calculated
			// by the allocator directly.

			effectiveness(0 + 3, i + actuator_start_index) = 0.f;
			effectiveness(1 + 3, i + actuator_start_index) = 0.f;
			effectiveness(2 + 3, i + actuator_start_index) = -ct;
		}
	}

	return num_actuators;
}

uint32_t ActuatorEffectivenessRotors::updateAxisFromTilts(const ActuatorEffectivenessTilts &tilts,
		float collective_tilt_control)
{
	if (!PX4_ISFINITE(collective_tilt_control)) {
		collective_tilt_control = -1.f;
	}

	uint32_t nontilted_motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		int tilt_index = _geometry.rotors[i].tilt_index;

		if (tilt_index == -1 || tilt_index >= tilts.count()) {
			nontilted_motors |= 1u << i;
			continue;
		}

		const ActuatorEffectivenessTilts::Params &tilt = tilts.config(tilt_index);
		const float tilt_angle = math::lerp(tilt.min_angle, tilt.max_angle, (collective_tilt_control + 1.f) / 2.f);
		const float tilt_direction = math::radians((float)tilt.tilt_direction);
		_geometry.rotors[i].axis = tiltedAxis(tilt_angle, tilt_direction);
	}

	return nontilted_motors;
}

// REVIEW: unnecessary?
matrix::Vector3f sph2cart(const float& azimuth, const float& elevation)
{
  return matrix::Vector3f(std::cos(azimuth) * std::cos(elevation), std::sin(azimuth) * std ::cos(elevation),
                        - std::sin(elevation));
}

void ActuatorEffectivenessRotors::updateRotorsFromSensors(const matrix::Vector2f* angles)
{
	for (int i=0; i < _geometry.num_rotors; ++i) {
		// REVIEW: does spherical2cartesian make sense?
		// rotation of rotor deflection
		const matrix::Vector3f mag_xyz = sph2cart(angles[i](0), angles[i](1));
		const matrix::Quaternionf Sd_rot_S(matrix::Vector3f(1.0f, 0.0f, 0.0f), mag_xyz);   // rotation from S (sensor) to Sd (sensor deflected)

		const matrix::Quaternionf& O_rot_S = _geometry.sensors[i].O_rot_S;
		// PX4_INFO("O_rot_S: %f %f %f %f ",(double)O_rot_S(0), (double)O_rot_S(1), (double)O_rot_S(2), (double)O_rot_S(3)  );

		// update position
		// TODO: update notation, shouldn't be S_t_SRd, should probably be Sd_t_SR
		const matrix::Vector3f S_t_SRd = Sd_rot_S.rotateVector(_geometry.sensors[i].S_t_SR); // deflected rotor wrt to sensor
		// PX4_INFO("S_t_SRd: %f %f %f ",(double)S_t_SRd(0), (double)S_t_SRd(1), (double)S_t_SRd(2)  );

		const matrix::Vector3f O_t_SRd = O_rot_S.rotateVector(S_t_SRd);
			_geometry.rotors[i].position = O_t_SRd + _geometry.sensors[i].O_t_OS;
		// PX4_INFO("_geometry.rotors[0].position: %f %f %f ", (double)_geometry.rotors[0].position(0), (double)_geometry.rotors[0].position(1), (double)_geometry.rotors[0].position(2)  );

		// PX4_INFO("%f %f",(double)S_t_SRd(0,0) , (double)S_t_SRd(0,0));
		// PX4_INFO("%f %f %f ",(double)S_t_SRd(0), (double)S_t_SRd(1), (double)S_t_SRd(2)  );

		// update axis
		_geometry.rotors[i].axis = O_rot_S.rotateVector(Sd_rot_S.rotateVector(_geometry.sensors[i].axis)); // assuming prop-vertical thrust
	}
}

Vector3f ActuatorEffectivenessRotors::tiltedAxis(float tilt_angle, float tilt_direction)
{
	Vector3f axis{0.f, 0.f, -1.f};
	return Dcmf{Eulerf{0.f, -tilt_angle, tilt_direction}} * axis;
}

uint32_t ActuatorEffectivenessRotors::getUpwardsMotors() const
{
	uint32_t upwards_motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		const Vector3f &axis = _geometry.rotors[i].axis;

		if (fabsf(axis(0)) < 0.1f && fabsf(axis(1)) < 0.1f && axis(2) < -0.5f) {
			upwards_motors |= 1u << i;
		}
	}

	return upwards_motors;
}

bool
ActuatorEffectivenessRotors::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	return addActuators(configuration);
}
