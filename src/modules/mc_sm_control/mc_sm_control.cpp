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

#include "mc_sm_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

MulticopterSMControl::MulticopterSMControl(bool vtol) :
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

MulticopterSMControl::~MulticopterSMControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterSMControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterSMControl::parameters_updated()
{
	// Store some of the parameters in a more convenient way & precompute often-used values
	const float posLambda3x3[] = {
		_param_pos_lam_x.get(), 0, 0,
		0, _param_pos_lam_y.get(), 0,
		0, 0, _param_pos_lam_z.get()
	};
	_position_control.setLambda(matrix::Matrix3f(posLambda3x3));

	const float posGain3x3[] = {
		_param_pos_gain_x.get(), 0, 0,
		0, _param_pos_gain_y.get(), 0,
		0, 0, _param_pos_gain_z.get()
	};
  	_position_control.setSwitchingGain(matrix::Matrix3f(posGain3x3));

	const float attLambda3x3[] = {
		_param_att_lam_x.get(), 0, 0,
		0, _param_att_lam_y.get(), 0,
		0, 0, _param_att_lam_z.get()
	};
	_attitude_control.setLambda(matrix::Matrix3f(attLambda3x3));

	const float attGain3x3[] = {
		_param_att_gain_x.get(), 0, 0,
		0, _param_att_gain_y.get(), 0,
		0, 0, _param_att_gain_z.get()
	};
  	_attitude_control.setSwitchingGain(matrix::Matrix3f(attGain3x3));
	_position_control.setTanhFactor(_param_pos_tanh_factor.get());
	_attitude_control.setTanhFactor(_param_att_tanh_factor.get());
	_position_control.setMass(_param_mass.get());

	const float inertia3x3[] = {
		_param_inertia_xx.get(), 0, 0,
		0, _param_inertia_yy.get(), 0,
		0, 0, _param_inertia_zz.get()
	};
	_attitude_control.setInertia(matrix::Matrix3f(inertia3x3));
}

float
MulticopterSMControl::throttle_curve(float throttle_stick_input){
	return (throttle_stick_input + 1.0f) / 2.0f;
}

void
MulticopterSMControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
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

	// run controller on angular rate updates
	vehicle_angular_velocity_s vehicle_angular_velocity;
	if (_vehicle_angular_velocity_sub.update(&vehicle_angular_velocity)) {

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		//const float dt = math::constrain(((vehicle_attitude.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = vehicle_angular_velocity.timestamp_sample;
		_attitude_control.setAngularVelocity(Vector3f(vehicle_angular_velocity.xyz));
		_attitude_control.setAngularAcceleration(Vector3f(vehicle_angular_velocity.xyz_derivative));

		// update vehicle attitude
		if (_vehicle_attitude_sub.updated()) {
			vehicle_attitude_s vehicle_attitude;
			if(_vehicle_attitude_sub.copy(&vehicle_attitude)) {
				_attitude = Quatf(vehicle_attitude.q);
				_position_control.setAttitude(_attitude);
				_attitude_control.setAttitude(_attitude);
			}
		}


		// update position
		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position;
			if(_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				_position_control.setPosition(Vector3f(vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z));
				_position_control.setLinearVelocity(Vector3f(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz));
				_position_control.setLinearAcceleration(Vector3f(vehicle_local_position.ax, vehicle_local_position.ay, vehicle_local_position.az));
			}
		}


		// Check for new position setpoint
		if (_vehicle_local_position_setpoint_sub.updated()) {
			vehicle_local_position_setpoint_s vehicle_local_position_setpoint;

			if (_vehicle_local_position_setpoint_sub.copy(&vehicle_local_position_setpoint)
			    && (vehicle_local_position_setpoint.timestamp > _last_vehicle_local_position_setpoint)) {

				_position_control.setPositionSetpoint(Vector3f(vehicle_local_position_setpoint.x, vehicle_local_position_setpoint.y, vehicle_local_position_setpoint.z));
				_position_control.setLinearVelocitySetpoint(Vector3f(vehicle_local_position_setpoint.vx, vehicle_local_position_setpoint.vy, vehicle_local_position_setpoint.vz));
				_position_control.setLinearAccelerationSetpoint(Vector3f(vehicle_local_position_setpoint.acceleration[0], vehicle_local_position_setpoint.acceleration[1], vehicle_local_position_setpoint.acceleration[2]));
				_position_control.setYawSetpoint(vehicle_local_position_setpoint.yaw);
				_last_vehicle_local_position_setpoint = vehicle_local_position_setpoint.timestamp;
			}
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


		if (_manual_control_setpoint_sub.updated()) {
			manual_control_setpoint_s manual_control_setpoint;
			if (_manual_control_setpoint_sub.copy(&manual_control_setpoint)) {
				_manual_thrust = manual_control_setpoint.throttle;
				_manual_roll = manual_control_setpoint.roll;
				_manual_pitch = -manual_control_setpoint.pitch;
				_manual_yaw = manual_control_setpoint.yaw;

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
		ocm.actuator = true;
		// ocm.thrust_and_torque = true;
		// ocm.direct_actuator = false;
		ocm.timestamp = hrt_absolute_time();
		_offboard_control_mode_pub.publish(ocm);



		//compute control torques and thrust
		if (true) {

			if (_vehicle_control_mode.flag_control_offboard_enabled) {

				float thrust_setpoint;
				Quatf attitude_setpoint;

				attitude_setpoint.print();

				// ====================================
				// manual attitude setpoint feedthrough
				// ====================================
				if (true){
					// get an attitude setpoint from the current manual inputs
					float roll_ref = 1.f * _manual_roll * M_PI_4_F;
					float pitch_ref = 1.f * _manual_pitch * M_PI_4_F;
					float yawspeed_ref = 1.f * _manual_yaw * M_PI_2_F;
					float yaw_ref = Eulerf(_attitude).psi();

					Quatf q_sp(Eulerf(roll_ref, pitch_ref, yaw_ref));

					vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
					const Eulerf euler_sp(q_sp);
					vehicle_attitude_setpoint.roll_body = euler_sp(0);
					vehicle_attitude_setpoint.pitch_body = euler_sp(1);
					vehicle_attitude_setpoint.yaw_body = euler_sp(2);
					q_sp.copyTo(vehicle_attitude_setpoint.q_d);
					vehicle_attitude_setpoint.yaw_sp_move_rate = yawspeed_ref;

					vehicle_attitude_setpoint.timestamp = hrt_absolute_time();
					_vehicle_attitude_setpoint_pub.publish(vehicle_attitude_setpoint);

					// run attitude controller
					_attitude_control.setAngularVelocitySetpoint(Vector3f(0.0f,0.0f,yawspeed_ref));
					_attitude_control.setAngularAccelerationSetpoint(Vector3f(0.0f,0.0f,0.0f));
					_attitude_control.setAttitudeSetpoint(q_sp);
					thrust_setpoint = -throttle_curve(_manual_thrust);

				}


				// TODO: setpoint from mocap
				else {
					_position_control.setPositionSetpoint(Vector3f(0.0f,0.0f,-2.5f));
					_position_control.setYawSetpoint(0.0f);
					_position_control.setLinearVelocitySetpoint(Vector3f(0.0f,0.0f,0.0f));
					_position_control.setLinearAccelerationSetpoint(Vector3f(0.0f,0.0f,0.0f));
					_position_control.update(thrust_setpoint, attitude_setpoint);
					// run attitude controller
					_attitude_control.setAngularVelocitySetpoint(Vector3f(0.0f,0.0f,0.0f));
					_attitude_control.setAngularAccelerationSetpoint(Vector3f(0.0f,0.0f,0.0f));
					_attitude_control.setAttitudeSetpoint(attitude_setpoint);

					// PX4_INFO("thrust setpoint: %f", (double)thrust_setpoint);
					thrust_setpoint = -constrain(thrust_setpoint, 0.0f, _param_thrust_max.get()) / _param_thrust_max.get();

				}

				// run attitude controller
				attitude_setpoint.print();
				Vector3f torque_setpoint = _attitude_control.update();
				// PX4_INFO("torque setpoint: %f %f %f", (double)torque_setpoint(0), (double)torque_setpoint(1), (double)torque_setpoint(2));

				// publish thrust and attitude setpoints
				vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
				vehicle_torque_setpoint_s vehicle_torque_setpoint{};

				for(int i=0; i<3; i++){
					torque_setpoint(i) = PX4_ISFINITE(torque_setpoint(i)) ? torque_setpoint(i) : 0.f;
					vehicle_torque_setpoint.xyz[i] = constrain(torque_setpoint(i), -1.f, 1.f);
				}

				// PX4_INFO("thrust setpoint (normalized): %f", (double)thrust_setpoint);
				vehicle_thrust_setpoint.xyz[0] = 0.0f;
				vehicle_thrust_setpoint.xyz[1] = 0.0f;
				vehicle_thrust_setpoint.xyz[2] = thrust_setpoint;

				vehicle_thrust_setpoint.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
				vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
				_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

				vehicle_torque_setpoint.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
				vehicle_torque_setpoint.timestamp = hrt_absolute_time();
				_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);
			}

		}



	}

	perf_end(_loop_perf);
}

int MulticopterSMControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterSMControl *instance = new MulticopterSMControl(vtol);

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

int MulticopterSMControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterSMControl::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("mc_sm_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_sm_control_main(int argc, char *argv[])
{
	return MulticopterSMControl::main(argc, argv);
}
