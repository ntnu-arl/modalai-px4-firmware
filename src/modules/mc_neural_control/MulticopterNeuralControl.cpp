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

#include "MulticopterNeuralControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

MulticopterNeuralControl::MulticopterNeuralControl(bool vtol) :
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

MulticopterNeuralControl::~MulticopterNeuralControl()
{
  perf_free(_loop_perf);
}

bool MulticopterNeuralControl::init()
{
  if (!_vehicle_angular_velocity_sub.registerCallback())
  {
    PX4_ERR("callback registration failed");
    return false;
  }

  return true;
}

void MulticopterNeuralControl::parameters_updated()
{
  // Store some of the parameters in a more convenient way & precompute often-used values
  // PD
  const float posKp3x3[] = { _param_pd_kp_xyz.get(), 0, 0, 0, _param_pd_kp_xyz.get(), 0, 0, 0, _param_pd_kp_xyz.get() };
  _pd_position_control.setKp(Matrix3f(posKp3x3));

  const float posKd3x3[] = { _param_pd_kd_xyz.get(), 0, 0, 0, _param_pd_kd_xyz.get(), 0, 0, 0, _param_pd_kd_xyz.get() };
  _pd_position_control.setKd(Matrix3f(posKd3x3));

  const float attKp3x3[] = { _param_pd_kp_rp.get(), 0, 0, 0, _param_pd_kp_rp.get(), 0, 0, 0, _param_pd_kp_y.get() };
  _pd_attitude_control.setKp(Matrix3f(attKp3x3));

  const float attKd3x3[] = { _param_pd_kd_rp.get(), 0, 0, 0, _param_pd_kd_rp.get(), 0, 0, 0, _param_pd_kd_y.get() };
  _pd_attitude_control.setKd(Matrix3f(attKd3x3));

  _pd_position_control.setMass(_param_mass.get());
  const float inertia3x3[] = { _param_inertia_xx.get(), 0, 0, 0, _param_inertia_yy.get(), 0, 0, 0,
                               _param_inertia_zz.get() };
  _pd_attitude_control.setInertia(Matrix3f(inertia3x3));
}

float MulticopterNeuralControl::throttle_curve(float throttle_stick_input)
{
  return (throttle_stick_input + 1.0f) / 2.0f;
}

void MulticopterNeuralControl::generateFailsafeTrajectory(trajectory_setpoint_s& traj_sp, const Vector3f& position,
                                                      const Quatf& attitude)
{
  // set position/yaw to current position/yaw
  position.copyTo(traj_sp.position);
  traj_sp.yaw = Eulerf(attitude).psi();

  // set derivatives to zero
  const Vector3f zero(0.0f, 0.0f, 0.0f);
  zero.copyTo(traj_sp.velocity);
  zero.copyTo(traj_sp.acceleration);
  traj_sp.yawspeed = 0.0f;
}

void MulticopterNeuralControl::Run()
{
  if (should_exit())
  {
    _vehicle_angular_velocity_sub.unregisterCallback();
    exit_and_cleanup();
    return;
  }

  perf_begin(_loop_perf);

  // Check if parameters have changed
  if (_parameter_update_sub.updated())
  {
    // clear update
    parameter_update_s param_update;
    _parameter_update_sub.copy(&param_update);

    updateParams();
    parameters_updated();
  }

  // run controller on angular rate updates
  vehicle_angular_velocity_s vehicle_angular_velocity;
  if (_vehicle_angular_velocity_sub.update(&vehicle_angular_velocity))
  {
    // Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
    // const float dt = math::constrain(((vehicle_attitude.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
    _last_run = vehicle_angular_velocity.timestamp_sample;
    _neural_control.setAngularVelocity(Vector3f(vehicle_angular_velocity.xyz));

    _pd_attitude_control.setAngularVelocity(Vector3f(vehicle_angular_velocity.xyz));
    _pd_attitude_control.setAngularAcceleration(Vector3f(vehicle_angular_velocity.xyz));

    // update vehicle attitude
    if (_vehicle_attitude_sub.updated())
    {
      vehicle_attitude_s vehicle_attitude;
      if (_vehicle_attitude_sub.copy(&vehicle_attitude))
      {
        _attitude = Quatf(vehicle_attitude.q);
        _neural_control.setAttitude(_attitude);

        _pd_position_control.setAttitude(_attitude);
        _pd_attitude_control.setAttitude(_attitude);
      }
    }

    // update position
    vehicle_local_position_s vehicle_local_position;
    if (_vehicle_local_position_sub.update(&vehicle_local_position))
    {
        // PX4_INFO("vehicle_local_position: %f | %f | %f", (double)vehicle_local_position.x,
        // (double)vehicle_local_position.y, (double)vehicle_local_position.z);

      _neural_control.setPosition(
          Vector3f(vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z));
      _neural_control.setLinearVelocity(
          Vector3f(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz));

      _pd_position_control.setPosition(
          Vector3f(vehicle_local_position.x, vehicle_local_position.y,vehicle_local_position.z));
      _pd_position_control.setLinearVelocity(
          Vector3f(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz));
      _pd_position_control.setLinearAcceleration(
          Vector3f(vehicle_local_position.ax, vehicle_local_position.ay, vehicle_local_position.az));
    }

    /* check for updates in other topics */
    //_manual_control_setpoint_sub.update(&_manual_control_setpoint);
    if (_vehicle_control_mode_sub.updated())
    {
      const bool previous_offboard_enabled = _vehicle_control_mode.flag_control_offboard_enabled;

      if (_vehicle_control_mode_sub.update(&_vehicle_control_mode))
      {
        if (!previous_offboard_enabled && _vehicle_control_mode.flag_control_offboard_enabled)
        {
          _time_offboard_enabled = _vehicle_control_mode.timestamp;
        }
        else if (previous_offboard_enabled && !_vehicle_control_mode.flag_control_offboard_enabled)
        {
          PX4_INFO("implement empty setpoint");
          generateFailsafeTrajectory(_trajectory_setpoint, _pd_position_control.getPosition(),
                                     _pd_position_control.getAttitude()); // should be the same for _pd_position_control and neural control
        }
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

    if (_manual_control_setpoint_sub.updated())
    {
      manual_control_setpoint_s manual_control_setpoint;
      if (_manual_control_setpoint_sub.copy(&manual_control_setpoint))
      {
        _manual_thrust = manual_control_setpoint.throttle;
        _manual_roll = manual_control_setpoint.roll;
        _manual_pitch = -manual_control_setpoint.pitch;
        _manual_yaw = manual_control_setpoint.yaw;
      }
    }

    if (_trajectory_setpoint_sub.updated()){

      // trajectory_setpoint_s trajectory_setpoint;
      _trajectory_setpoint_sub.copy(&_trajectory_setpoint);
      // if(_trajectory_setpoint_sub.copy(&_trajectory_setpoint))
      // {
        // PX4_INFO("setpoint received: %f %f %f", double(_trajectory_setpoint.position[0]),
        //       double(_trajectory_setpoint.position[1]), double(_trajectory_setpoint.position[2]));
      // }
    }
    // _trajectory_setpoint_sub.update(&_trajectory_setpoint);
    if (_vehicle_control_mode.flag_control_offboard_enabled)
    {
      // set failsafe setpoint if there hasn't been a new
      // trajectory setpoint since offboard control started
      if ((_trajectory_setpoint.timestamp < _time_offboard_enabled) &&
          (vehicle_angular_velocity.timestamp_sample > _time_offboard_enabled))
      {
        PX4_WARN("invalid setpoint, impl failsafe: %f %f %f", double(_trajectory_setpoint.position[0]),
                 double(_trajectory_setpoint.position[1]), double(_trajectory_setpoint.position[2]));
        generateFailsafeTrajectory(_trajectory_setpoint, _pd_position_control.getPosition(),
                                   _pd_position_control.getAttitude());
        _trajectory_setpoint.timestamp = vehicle_angular_velocity.timestamp_sample;
      }
      else {
        // PX4_INFO("setpoint: %f %f %f", double(_trajectory_setpoint.position[0]),
        //         double(_trajectory_setpoint.position[1]), double(_trajectory_setpoint.position[2]));
        // PX4_INFO("vel: %f %f %f", double(_trajectory_setpoint.velocity[0]),
        //         double(_trajectory_setpoint.velocity[1]), double(_trajectory_setpoint.velocity[2]));
        // PX4_INFO("acc: %f %f %f", double(_trajectory_setpoint.acceleration[0]),
        //         double(_trajectory_setpoint.acceleration[1]), double(_trajectory_setpoint.acceleration[2]));
        //         PX4_INFO("yaw yawspeed: %f %f", double(_trajectory_setpoint.yaw), double(_trajectory_setpoint.yawspeed));
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

    // compute control torques and thrust
    if (_vehicle_control_mode.flag_control_offboard_enabled &&
        (_param_manual_ctrl.get() || (_trajectory_setpoint.timestamp >= _time_offboard_enabled)))
    {
      float thrust_setpoint = 0.0f;
      Quatf attitude_setpoint{};

      // attitude_setpoint.print();

      // ====================================
      // manual attitude setpoint feedthrough
      // ====================================
      if (_param_manual_ctrl.get())
      {
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

        // not sure if this has to be here
        // ?????????????????????????? //
        //_neural_control.setPositionSetpoint(Vector3f(_trajectory_setpoint.position));
        // ?????????????????????????? //

        // run attitude controller
        _pd_attitude_control.setAngularVelocitySetpoint(Vector3f(0.0f, 0.0f, yawspeed_ref));
        _pd_attitude_control.setAngularAccelerationSetpoint(Vector3f(0.0f, 0.0f, 0.0f));
        _pd_attitude_control.setAttitudeSetpoint(q_sp);
        thrust_setpoint = throttle_curve(_manual_thrust);
      }
      // TODO: setpoint from mocap
      else
      {
        vehicle_local_position_setpoint_s local_pos_sp{};
        local_pos_sp.timestamp = hrt_absolute_time();
        _vehicle_local_position_setpoint_pub.publish(local_pos_sp);

        _neural_control.setPositionSetpoint(Vector3f(_trajectory_setpoint.position));

        _pd_position_control.setPositionSetpoint(Vector3f(_trajectory_setpoint.position));
        _pd_position_control.setLinearVelocitySetpoint(Vector3f(_trajectory_setpoint.velocity));
        _pd_position_control.setLinearAcceleration(Vector3f(_trajectory_setpoint.acceleration));
        _pd_position_control.setYawSetpoint(_trajectory_setpoint.yaw);

        switch (_param_controller.get())
        {
          case NONLINEAR_PD:
            _pd_position_control.updatePD(thrust_setpoint, attitude_setpoint);
            break;

          case NEURAL:
            break;

          default:
            PX4_ERR("Invalid controller selected: %i", _param_controller.get());
            break;
        }
        if (_param_verbose.get())
        {
          PX4_INFO("thrust setpoint: %f", (double)thrust_setpoint);
        }
        thrust_setpoint /= _param_thrust_max.get();

        // run attitude controller
        _pd_attitude_control.setAngularVelocitySetpoint(Vector3f(0.0f, 0.0f, _trajectory_setpoint.yawspeed));
        _pd_attitude_control.setAngularAccelerationSetpoint(Vector3f(0.0f, 0.0f, 0.0f));
        _pd_attitude_control.setAttitudeSetpoint(attitude_setpoint);
      }

      // run attitude controller
      Vector3f torque_setpoint;
      switch (_param_controller.get())
      {
        case NONLINEAR_PD:
          torque_setpoint = _pd_attitude_control.updatePD();
          
          break;

        case NEURAL:
          _neural_control.updateNeural(thrust_setpoint, attitude_setpoint);
          break;

        default:
          PX4_ERR("Invalid controller selected: %i", _param_controller.get());
          break;
      }
      if (_param_verbose.get())
      {
        PX4_INFO("torque setpoint: %f %f %f", (double)torque_setpoint(0), (double)torque_setpoint(1),
                 (double)torque_setpoint(2));
      }
      torque_setpoint(0) /= _param_moment_rp_max.get();
      torque_setpoint(1) /= _param_moment_rp_max.get();
      torque_setpoint(2) /= _param_moment_y_max.get();

      // publish thrust and attitude setpoints
      vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
      vehicle_torque_setpoint_s vehicle_torque_setpoint{};

      // set default thrust to hover percentage
      thrust_setpoint = PX4_ISFINITE(thrust_setpoint) ? thrust_setpoint : _param_hover.get();
      thrust_setpoint = -constrain(thrust_setpoint, 0.0f, 1.0f);
      for (int i = 0; i < 3; i++)
      {
        torque_setpoint(i) = PX4_ISFINITE(torque_setpoint(i)) ? torque_setpoint(i) : 0.f;
        vehicle_torque_setpoint.xyz[i] = constrain(torque_setpoint(i), -1.f, 1.f);
      }

      // PX4_INFO("thrust setpoint (normalized): %f", (double)thrust_setpoint);
      vehicle_thrust_setpoint.xyz[0] = 0.0f;
      vehicle_thrust_setpoint.xyz[1] = 0.0f;
      vehicle_thrust_setpoint.xyz[2] = thrust_setpoint;

			// scale setpoints by battery status if enabled
			if (_param_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

      vehicle_attitude_setpoint_s vehicle_attitude_setpoint{};
      vehicle_attitude_setpoint.timestamp = hrt_absolute_time();
      vehicle_attitude_setpoint.thrust_body[0] = vehicle_thrust_setpoint.xyz[0];
      vehicle_attitude_setpoint.thrust_body[1] = vehicle_thrust_setpoint.xyz[1];
      vehicle_attitude_setpoint.thrust_body[2] = vehicle_thrust_setpoint.xyz[2];
      attitude_setpoint.copyTo(vehicle_attitude_setpoint.q_d);
      const Eulerf euler{ attitude_setpoint };
      vehicle_attitude_setpoint.roll_body = euler.phi();
      vehicle_attitude_setpoint.pitch_body = euler.theta();
      vehicle_attitude_setpoint.yaw_body = euler.psi();
      _vehicle_attitude_setpoint_pub.publish(vehicle_attitude_setpoint);

      vehicle_thrust_setpoint.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
      vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
      _vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

      vehicle_torque_setpoint.timestamp_sample = vehicle_angular_velocity.timestamp_sample;
      vehicle_torque_setpoint.timestamp = hrt_absolute_time();
      _vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);
    }
  }
  perf_end(_loop_perf);
}

int MulticopterNeuralControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterNeuralControl *instance = new MulticopterNeuralControl(vtol);

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

int MulticopterNeuralControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterNeuralControl::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("mc_neural_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_neural_control_main(int argc, char *argv[])
{
	return MulticopterNeuralControl::main(argc, argv);
}
