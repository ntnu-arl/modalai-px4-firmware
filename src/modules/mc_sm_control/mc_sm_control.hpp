/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>

#include <uORB/topics/vehicle_status.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/slew_rate/SlewRate.hpp>
#include <uORB/topics/offboard_control_mode.h>

#include <SMPositionControl.hpp>
#include <SMAttitudeControl.hpp>

using namespace time_literals;

class MulticopterSMControl : public ModuleBase<MulticopterSMControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	MulticopterSMControl(bool vtol = false);
	~MulticopterSMControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	SMPositionControl _position_control; /**< class for position control calculations */
	SMAttitudeControl _attitude_control;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};


	//uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};



	//uORB::Publication<vehicle_rates_setpoint_s>     _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};    /**< rate setpoint publication */
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<offboard_control_mode_s>	_offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	// =================================================




	//manual_control_setpoint_s       _manual_control_setpoint {};    /**< manual control setpoint */
	vehicle_control_mode_s          _vehicle_control_mode {};       /**< vehicle control mode */

	perf_counter_t  _loop_perf;             /**< loop duration performance counter */

	matrix::Vector3f _thrust_setpoint_body; /**< body frame 3D thrust vector */

	float _man_yaw_sp{0.f};                 /**< current yaw setpoint in manual mode */

	hrt_abstime _last_run{0};
	hrt_abstime _last_vehicle_local_position_setpoint{0};

	bool _spooled_up{false}; ///< used to make sure the vehicle cannot take off during the spoolup time

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SM_POS_LAM_X>)		_param_pos_lam_x,
		(ParamFloat<px4::params::SM_POS_LAM_Y>)		_param_pos_lam_y,
		(ParamFloat<px4::params::SM_POS_LAM_Z>)		_param_pos_lam_z,
		(ParamFloat<px4::params::SM_POS_GAIN_X>)	_param_pos_gain_x,
		(ParamFloat<px4::params::SM_POS_GAIN_Y>)	_param_pos_gain_y,
		(ParamFloat<px4::params::SM_POS_GAIN_Z>)	_param_pos_gain_z,
		(ParamFloat<px4::params::SM_POS_TANH>)		_param_pos_tanh_factor,
		(ParamFloat<px4::params::SM_POS_MASS>)		_param_mass,
		(ParamFloat<px4::params::SM_POS_T_MAX>)		_param_thrust_max,
		(ParamFloat<px4::params::SM_ATT_LAM_X>)		_param_att_lam_x,
		(ParamFloat<px4::params::SM_ATT_LAM_Y>)		_param_att_lam_y,
		(ParamFloat<px4::params::SM_ATT_LAM_Z>)		_param_att_lam_z,
		(ParamFloat<px4::params::SM_ATT_GAIN_X>)	_param_att_gain_x,
		(ParamFloat<px4::params::SM_ATT_GAIN_Y>)	_param_att_gain_y,
		(ParamFloat<px4::params::SM_ATT_GAIN_Z>)	_param_att_gain_z,
		(ParamFloat<px4::params::SM_ATT_TANH>)		_param_att_tanh_factor,
		(ParamFloat<px4::params::SM_ATT_I_XX>)		_param_inertia_xx,
		(ParamFloat<px4::params::SM_ATT_I_YY>)		_param_inertia_yy,
		(ParamFloat<px4::params::SM_ATT_I_ZZ>)		_param_inertia_zz
	)
};

