/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file mc_sm_att_control.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_sm_pos_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

MulticopterSMPositionControl::MulticopterSMPositionControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	// _vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	parameters_updated();
	// Rate of change 5% per second -> 1.6 seconds to ramp to default 8% MPC_MANTHR_MIN
	//_manual_throttle_minimum.setSlewRate(0.05f);
	// Rate of change 50% per second -> 2 seconds to ramp to 100%
	//_manual_throttle_maximum.setSlewRate(0.5f);
}

MulticopterSMPositionControl::~MulticopterSMPositionControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterSMPositionControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterSMPositionControl::parameters_updated()
{
	// Store some of the parameters in a more convenient way & precompute often-used values
	const float lambda3x3[] = {
		_param_lam_x.get(), 0, 0,
		0, _param_lam_y.get(), 0,
		0, 0, _param_lam_z.get()
	};
	_position_control.setLambda(matrix::Matrix3f(lambda3x3));

	const float gain3x3[] = {
		_param_gain_x.get(), 0, 0,
		0, _param_gain_y.get(), 0,
		0, 0, _param_gain_z.get()
	};
  _position_control.setSwitchingGain(matrix::Matrix3f(gain3x3));

	_position_control.setTanhFactor(_param_tanh_factor.get());
	_position_control.setMass(_param_mass.get());
}

void
MulticopterSMPositionControl::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	// run controller on position updates
	vehicle_local_position_s vehicle_local_position;

	if (_vehicle_local_position_sub.update(&vehicle_local_position)) {

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		//const float dt = math::constrain(((vehicle_attitude.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = vehicle_local_position.timestamp_sample;

		_position_control.setPosition(Vector3f(vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z));

		// Check for new position setpoint
		if (_vehicle_local_position_setpoint_sub.updated()) {
			vehicle_local_position_setpoint_s vehicle_local_position_setpoint;

			if (_vehicle_local_position_setpoint_sub.copy(&vehicle_local_position_setpoint)
			    && (vehicle_local_position_setpoint.timestamp > _last_vehicle_local_position_setpoint)) {

				_position_control.setPositionSetpoint(Vector3f(vehicle_local_position_setpoint.x, vehicle_local_position_setpoint.y, vehicle_local_position_setpoint.z));
				_last_vehicle_local_position_setpoint = vehicle_local_position_setpoint.timestamp;
			}
		}

		//_position_control.setAttitudeSetpoint(Quatf(1.0,0.0,0.0,0.0));


		// update angular velocity
		// update angular acceleration
		if (_vehicle_angular_velocity_sub.updated()) {
			vehicle_angular_velocity_s vehicle_angular_velocity;
			if(_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
				_position_control.setAngularVelocity(Vector3f(vehicle_angular_velocity.xyz[0], vehicle_angular_velocity.xyz[1], vehicle_angular_velocity.xyz[2]));
				_position_control.setAngularAcceleration(Vector3f(vehicle_angular_velocity.xyz_derivative[0], vehicle_angular_velocity.xyz_derivative[1], vehicle_angular_velocity.xyz_derivative[2]));
			}

		}

		// update angular velocity setpoint
		if (_vehicle_rates_setpoint_sub.updated()) {
			vehicle_rates_setpoint_s vehicle_rates_setpoint;
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				_position_control.setAngularVelocitySetpoint(Vector3f(vehicle_rates_setpoint.roll, vehicle_rates_setpoint.pitch, vehicle_rates_setpoint.yaw));
			}
		}

		// set angular accel setpoint to 0
		if (_vehicle_rates_setpoint_sub.updated()) {
			vehicle_rates_setpoint_s vehicle_rates_setpoint;
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				_position_control.setAngularAccelerationSetpoint(Vector3f(0.0, 0.0, 0.0));
			}
		}
		//

		// Check for a heading reset
		if (_quat_reset_counter != vehicle_attitude.quat_reset_counter) {
			const Quatf delta_q_reset(vehicle_attitude.delta_q_reset);

			// for stabilized attitude generation only extract the heading change from the delta quaternion
			_man_yaw_sp = wrap_pi(_man_yaw_sp + Eulerf(delta_q_reset).psi());

			if (vehicle_attitude.timestamp > _last_attitude_setpoint) {
				// adapt existing attitude setpoint unless it was generated after the current attitude estimate
				_position_control.adaptAttitudeSetpoint(delta_q_reset);
			}

			_quat_reset_counter = vehicle_attitude.quat_reset_counter;
		}

		/* check for updates in other topics */
		//_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		if (_vehicle_control_mode_sub.updated()) {
			if (_vehicle_control_mode_sub.copy(&_vehicle_control_mode)) {
				/* PX4_INFO("%lu", _vehicle_control_mode.timestamp);
				PX4_INFO("offboard %d", _vehicle_control_mode.flag_control_offboard_enabled);
				PX4_INFO("manual %d", _vehicle_control_mode.flag_control_manual_enabled);
				PX4_INFO("position %d", _vehicle_control_mode.flag_control_position_enabled);
				PX4_INFO("velocity %d", _vehicle_control_mode.flag_control_velocity_enabled);
				PX4_INFO("altitude %d", _vehicle_control_mode.flag_control_altitude_enabled);
				PX4_INFO("climb rate %d", _vehicle_control_mode.flag_control_climb_rate_enabled);
				PX4_INFO("accerl %d", _vehicle_control_mode.flag_control_acceleration_enabled);
				PX4_INFO("attitude %d", _vehicle_control_mode.flag_control_attitude_enabled);
				PX4_INFO("rates %d", _vehicle_control_mode.flag_control_rates_enabled); */
			}
		}

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {

				const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
				// _spooled_up = armed && hrt_elapsed_time(&vehicle_status.armed_time) > _param_com_spoolup_time.get() * 1_s;
				_spooled_up = armed;
			}
		}

		// =================================
		// publish offboard control commands
		// =================================
		offboard_control_mode_s ocm{};
		ocm.position = false;
		ocm.velocity = false;
		ocm.acceleration = false;
		ocm.attitude = false;
		ocm.body_rate = false;
		ocm.thrust_and_torque = true;
		ocm.direct_actuator = false;
		ocm.timestamp = hrt_absolute_time();
		_offboard_control_mode_pub.publish(ocm);



		//compute control torques and thrust
		if (true) {

			// if (_vehicle_control_mode.flag_control_offboard_enabled) {

			// 	generate_attitude_setpoint(q, dt, _reset_yaw_sp);
			// 	attitude_setpoint_generated = true;

			// } else {
			// 	_man_roll_input_filter.reset(0.f);
			// 	_man_pitch_input_filter.reset(0.f);
			// }

			if (_vehicle_control_mode.flag_control_offboard_enabled) {
				Vector3f torque_sp = _position_control.update();
				//float thrust_sp = _position_control


				//const hrt_abstime now = hrt_absolute_time();



				// publish thrust and torque setpoints
				vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
				vehicle_torque_setpoint_s vehicle_torque_setpoint{};

				_thrust_setpoint_body.copyTo(vehicle_thrust_setpoint.xyz);
				vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_sp(0)) ? torque_sp(0) : 0.f;
				vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_sp(1)) ? torque_sp(1) : 0.f;
				vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_sp(2)) ? torque_sp(2) : 0.f;


				for(int i=0; i<3; i++){
					torque_sp(i) = constrain(torque_sp(i),-1.f,1.f);
				}

				vehicle_thrust_setpoint.timestamp_sample = vehicle_attitude.timestamp_sample;
				vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
				_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

				vehicle_torque_setpoint.timestamp_sample = vehicle_attitude.timestamp_sample;
				vehicle_torque_setpoint.timestamp = hrt_absolute_time();
				_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);
			}

		}



	}

	perf_end(_loop_perf);
}

int MulticopterSMPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterSMPositionControl *instance = new MulticopterSMPositionControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterSMPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterSMPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_sm_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_sm_att_control_main(int argc, char *argv[])
{
	return MulticopterSMPositionControl::main(argc, argv);
}
