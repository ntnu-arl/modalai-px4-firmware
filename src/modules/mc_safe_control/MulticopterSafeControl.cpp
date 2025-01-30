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

#include "MulticopterSafeControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <math.h>

using namespace matrix;

MulticopterSafeControl::MulticopterSafeControl(bool vtol) :
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

MulticopterSafeControl::~MulticopterSafeControl()
{
  perf_free(_loop_perf);
}

bool MulticopterSafeControl::init()
{
  if (!_vehicle_angular_velocity_sub.registerCallback())
  {
    PX4_ERR("callback registration failed");
    return false;
  }

  return true;
}

void MulticopterSafeControl::parameters_updated()
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

  // setup CBF filter parameters
  _cbf_safety_filter.setEpsilon(_param_cbf_epsilon.get());
  _cbf_safety_filter.setPole0(_param_cbf_pole0.get());
  _cbf_safety_filter.setKappa(_param_cbf_kappa.get());
  _cbf_safety_filter.setGamma(_param_cbf_gamma.get());
  _cbf_safety_filter.setAlpha(_param_cbf_alpha.get());
}

float MulticopterSafeControl::throttle_curve(float throttle_stick_input)
{
  return (throttle_stick_input + 1.0f) / 2.0f;
}

void MulticopterSafeControl::generateFailsafeTrajectory(trajectory_setpoint_s& traj_sp, const Vector3f& position,
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

void MulticopterSafeControl::_accelerationControl(const Vector3f& acc_sp, const float hover_thrust, Vector3f& thr_sp)
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	Vector3f body_z = Vector3f(-acc_sp(0), -acc_sp(1), CONSTANTS_ONE_G).normalized();
	limitTilt(body_z, Vector3f(0, 0, 1), radians(45.0f));
	// Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = acc_sp(2) * (hover_thrust / CONSTANTS_ONE_G) - hover_thrust;
	// Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	// collective_thrust = math::min(collective_thrust, -_lim_thr_min);
  thr_sp = body_z * collective_thrust;
}

void MulticopterSafeControl::Run()
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

    _pd_attitude_control.setAngularVelocity(Vector3f(vehicle_angular_velocity.xyz));
    _pd_attitude_control.setAngularAcceleration(Vector3f(vehicle_angular_velocity.xyz));

    // update vehicle attitude
    if (_vehicle_attitude_sub.updated())
    {
      vehicle_attitude_s vehicle_attitude;
      if (_vehicle_attitude_sub.copy(&vehicle_attitude))
      {
        _attitude = Quatf(vehicle_attitude.q);

        _pd_position_control.setAttitude(_attitude);
        _pd_attitude_control.setAttitude(_attitude);

        _cbf_safety_filter.setAttitude(_attitude);
      }
    }

    // update vehicle acceleration
    if (_vehicle_acceleration_sub.update(&vehicle_acceleration)) {
      _current_z_acceleration = Vector3f{vehicle_acceleration.xyz}(2)
    }

    // update position
    vehicle_local_position_s vehicle_local_position;
    if (_vehicle_local_position_sub.update(&vehicle_local_position))
    {
        // PX4_INFO("vehicle_local_position: %f | %f | %f", (double)vehicle_local_position.x,
        // (double)vehicle_local_position.y, (double)vehicle_local_position.z);

      _pd_position_control.setPosition(
          Vector3f(vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z));
      _pd_position_control.setLinearVelocity(
          Vector3f(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz));
      _pd_position_control.setLinearAcceleration(
          Vector3f(vehicle_local_position.ax, vehicle_local_position.ay, vehicle_local_position.az));

      _cbf_safety_filter.setPosition(
          Vector3f(vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z));
      _cbf_safety_filter.setLinearVelocity(
          Vector3f(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz));
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
          // generateFailsafeTrajectory(_trajectory_setpoint, _pd_position_control.getPosition(),
                                    //  _pd_position_control.getAttitude());
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

    hover_thrust_estimate_s hte{};
    hte.valid = false;
    if (_hover_thrust_estimate_sub.update(&hte))
    {
      if (hte.valid)
      {
        hte.hover_thrust = math::constrain(hte.hover_thrust, 0.1f, 0.9f);
      }
    }

    // if (_trajectory_setpoint_sub.updated()){

    //   // trajectory_setpoint_s trajectory_setpoint;
    //   _trajectory_setpoint_sub.copy(&_trajectory_setpoint);
    //   // if(_trajectory_setpoint_sub.copy(&_trajectory_setpoint))
    //   // {
    //     // PX4_INFO("setpoint received: %f %f %f", double(_trajectory_setpoint.position[0]),
    //     //       double(_trajectory_setpoint.position[1]), double(_trajectory_setpoint.position[2]));
    //   // }
    // }
    // // _trajectory_setpoint_sub.update(&_trajectory_setpoint);
    // if (_vehicle_control_mode.flag_control_offboard_enabled)
    // {
    //   // set failsafe setpoint if there hasn't been a new
    //   // trajectory setpoint since offboard control started
    //   if ((_trajectory_setpoint.timestamp < _time_offboard_enabled) &&
    //       (vehicle_angular_velocity.timestamp_sample > _time_offboard_enabled))
    //   {
    //     PX4_WARN("invalid setpoint, impl failsafe: %f %f %f", double(_trajectory_setpoint.position[0]),
    //              double(_trajectory_setpoint.position[1]), double(_trajectory_setpoint.position[2]));
    //     generateFailsafeTrajectory(_trajectory_setpoint, _pd_position_control.getPosition(),
    //                                _pd_position_control.getAttitude());
    //     _trajectory_setpoint.timestamp = vehicle_angular_velocity.timestamp_sample;
    //   }
    //   else {
    //     // PX4_INFO("setpoint: %f %f %f", double(_trajectory_setpoint.position[0]),
    //     //         double(_trajectory_setpoint.position[1]), double(_trajectory_setpoint.position[2]));
    //     // PX4_INFO("vel: %f %f %f", double(_trajectory_setpoint.velocity[0]),
    //     //         double(_trajectory_setpoint.velocity[1]), double(_trajectory_setpoint.velocity[2]));
    //     // PX4_INFO("acc: %f %f %f", double(_trajectory_setpoint.acceleration[0]),
    //     //         double(_trajectory_setpoint.acceleration[1]), double(_trajectory_setpoint.acceleration[2]));
    //     //         PX4_INFO("yaw yawspeed: %f %f", double(_trajectory_setpoint.yaw), double(_trajectory_setpoint.yawspeed));
    //   }
    // }

    // PX4_INFO("Checking for tof_obstacles_chunk update");
    // collect detected obstacles
    tof_obstacles_chunk_s tof_obstacles_chunk;
    if (_tof_obstacles_chunk_sub.update(&tof_obstacles_chunk))
    {
      PX4_INFO("Received chunk %d/%d with %d points (total %d)",
        (int)tof_obstacles_chunk.chunk_id + 1,
        (int)tof_obstacles_chunk.num_chunks,
        (int)tof_obstacles_chunk.num_points_chunk,
        (int)tof_obstacles_chunk.num_points_total
      );

      std::vector<Vector3f>& obstacles = _cbf_safety_filter.obstacles();
      if (_prev_obstacles_chunk_id < 0 || (int)tof_obstacles_chunk.chunk_id <= _prev_obstacles_chunk_id) {
        obstacles.clear();
      }
      _prev_obstacles_chunk_id = (int)tof_obstacles_chunk.chunk_id;
      for (int i = 0; i < tof_obstacles_chunk.num_points_chunk; i++) {
        obstacles.emplace_back(
          tof_obstacles_chunk.points_x[i],
          tof_obstacles_chunk.points_y[i],
          tof_obstacles_chunk.points_z[i]
        );
      }

      PX4_INFO("Number of obstacles: %d", (int)obstacles.size());
      for (std::size_t i = 0; i < std::min<std::size_t>(10, obstacles.size()); i++) {
        PX4_INFO("Obstacle %d: %f, %f, %f", (int)i, (double)obstacles[i](0), (double)obstacles[i](1), (double)obstacles[i](2));
      }
    }

    // =================================
    // publish offboard control commands
    // =================================
    offboard_control_mode_s ocm{};
    ocm.position = false;
    ocm.velocity = false;
    ocm.acceleration = false;
    ocm.attitude = true;
    ocm.body_rate = false;
    ocm.actuator = false;
    // ocm.thrust_and_torque = true;
    // ocm.direct_actuator = false;
    ocm.timestamp = hrt_absolute_time();
    _offboard_control_mode_pub.publish(ocm);

    // {
    // std::vector<Vector3f>& obstacles = _cbf_safety_filter.obstacles();
    // obstacles.clear();
    // Vector3f obs_global = Vector3f(0.f, 0.f, -15.f);
    // Dcmf R_IB(_attitude);
    // Dcmf R_BI = R_IB.transpose();
    // Vector3f obs_local = R_BI * (obs_global - _pd_position_control.getPosition());
    // obstacles.push_back(obs_local);
    // }

    // compute acceleration setpoint
    Vector3f acceleration_setpoint;
    if (_vehicle_control_mode.flag_control_offboard_enabled &&
        (_param_manual_ctrl.get()))
    {
      // float thrust_setpoint = 0.0f;
      // Quatf attitude_setpoint {};

      float yawspeed_ref = 0.f;

      // ====================================
      // manual input feedthrough
      // ====================================
      // vehicle_local_position_setpoint_s local_pos_sp{};
      // local_pos_sp.timestamp = hrt_absolute_time();
      // _vehicle_local_position_setpoint_pub.publish(local_pos_sp);

      // float y_ref = 1.f * _manual_roll;
      // float x_ref = -1.f * _manual_pitch;
      // float z_ref = -1.f * _manual_thrust;
      // yawspeed_ref = 1.f * _manual_yaw * M_PI_2_F;
      // Vector3f target = _pd_position_control.getPosition()
      //   + _pd_position_control.getAttitude().rotateVector(Vector3f(x_ref, y_ref, z_ref));

      // _pd_position_control.setPositionSetpoint(target);
      // _pd_position_control.setLinearVelocitySetpoint(Vector3f(0.0f, 0.0f, 0.0f));
      // _pd_position_control.setLinearAcceleration(Vector3f(0.0f, 0.0f, 0.0f));

      // _pd_position_control.updatePD(acceleration_setpoint);

      Vector3f accel_joystick(-1.f * _manual_pitch, 1.f * _manual_roll, -1.f * _manual_thrust);
      accel_joystick(0) *= 3.f;
      accel_joystick(1) *= 3.f;
      accel_joystick(2) *= 5.f;

      yawspeed_ref = 1.f * _manual_yaw * M_PI_2_F;

      Eulerf euler_current(_attitude);
      Eulerf euler_vehicle(0.f, 0.f, euler_current.psi());
      Dcmf R_vehicle(euler_vehicle);

      acceleration_setpoint = R_vehicle * accel_joystick;
      // acceleration_setpoint(2) -= 9.81f;

      // PX4_INFO("Manual control: %f, %f, %f",
      //   (double)acceleration_setpoint(0),
      //   (double)acceleration_setpoint(1),
      //   (double)acceleration_setpoint(2)
      // );

      // TODO: drive with attitude estimate instead of attitude rate
      static int count = 0;
      if (count >= 8)
      {
        count = 0;
      }
      else
      {
        count++;
        return;
      }

      [[maybe_unused]] hrt_abstime tic = hrt_absolute_time();
      _cbf_safety_filter.update(acceleration_setpoint, hrt_absolute_time());
      [[maybe_unused]] hrt_abstime toc = hrt_absolute_time();
      // thrust_setpoint = _pd_position_control.calculateThrust(acceleration_setpoint);
      // attitude_setpoint = _pd_position_control.calculateAttitude(acceleration_setpoint);
      // PX4_WARN("cbf_filter duration: %llu", toc - tic);

      acceleration_setpoint(0) = math::constrain(acceleration_setpoint(0), -3.f, 3.f);
      acceleration_setpoint(1) = math::constrain(acceleration_setpoint(1), -3.f, 3.f);
      acceleration_setpoint(2) = math::constrain(acceleration_setpoint(2), -5.f, 5.f);

      //       acceleration_setpoint(2) -= 9.81f*2.f;

      // PX4_INFO("accel setpoint: %f, %f, %f",
      //   (double)acceleration_setpoint(0),
      //   (double)acceleration_setpoint(1),
      //   (double)acceleration_setpoint(2)
      // );

      // convert to thrust and attitude setpoint
      // if (hte.valid)
      // {
        vehicle_attitude_setpoint_s vehicle_attitude_setpoint{};

        // convert to thrust sp
        Vector3f thr_sp;

        float hover_thrust = _hover_thrust_estimate * _thrust_compensation_factor;
        _accelerationControl(acceleration_setpoint, hover_thrust, thr_sp);

        // PX4_INFO("accel_sp: %f, %f, %f", (double)acceleration_setpoint(0), (double)acceleration_setpoint(1), (double)acceleration_setpoint(2));
        // PX4_INFO("hover thrust: %f", (double)hte.hover_thrust);
        // PX4_INFO("thr_sp: %f, %f, %f", (double)thr_sp(0), (double)thr_sp(1), (double)thr_sp(2));

        // update attitude sp
        thrustToAttitude(thr_sp, euler_current.psi(), vehicle_attitude_setpoint);
        vehicle_attitude_setpoint.yaw_sp_move_rate = yawspeed_ref;

        // run hover thrust estimator
        _current_thrust_estimate = thr_sp / _hover_thrust_estimate * CONSTANTS_ONE_G;
        _thrust_compensation_factor = 0.01f * (_current_z_acceleration / _current_thrust_estimate) + 0.99f* _thrust_compensation_factor;
        _thrust_compensation_factor = math::constrain(_thrust_compensation_factor, -1.3f, 1.3f);
        PX4_INFO("hover thrust: %f", (double)hover_thrust);

        vehicle_attitude_setpoint.timestamp = hrt_absolute_time();
        _vehicle_attitude_setpoint_pub.publish(vehicle_attitude_setpoint);
      // }
    }
  }
  perf_end(_loop_perf);
}

int MulticopterSafeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterSafeControl *instance = new MulticopterSafeControl(vtol);

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

int MulticopterSafeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterSafeControl::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("mc_safe_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_safe_control_main(int argc, char *argv[])
{
	return MulticopterSafeControl::main(argc, argv);
}
