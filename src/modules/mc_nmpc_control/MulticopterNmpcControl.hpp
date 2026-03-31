#pragma once

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/nmpc_state_data.h>
#include <uORB/topics/nmpc_control_data.h>

#include "protocol.h"

using namespace time_literals;

class MulticopterNmpcControl : public ModuleBase<MulticopterNmpcControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	MulticopterNmpcControl();
	~MulticopterNmpcControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	static constexpr hrt_abstime TRAJECTORY_SETPOINT_TIMEOUT{500_ms};
	static constexpr hrt_abstime MOTOR_FEEDBACK_TIMEOUT{200_ms};

	void Run() override;
	void parameters_updated();
	void pack_state(state_packet_t *pkt);
	void publish_actuator_motors(const control_packet_t *pkt);
	void update_motor_feedback(const esc_status_s &esc_status);
	void generateFailsafeTrajectory(trajectory_setpoint_s &traj_sp,
					 const matrix::Vector3f &position,
					 const matrix::Quatf &attitude);
	bool load_allocation_matrix();

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};

	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<nmpc_state_data_s> _nmpc_state_pub{ORB_ID(nmpc_state_data)};
	uORB::Subscription _nmpc_control_sub{ORB_ID(nmpc_control_data)};

	vehicle_control_mode_s _vehicle_control_mode{};
	trajectory_setpoint_s _trajectory_setpoint{};

	matrix::Quatf _attitude;
	matrix::Vector3f _position;
	matrix::Vector3f _velocity;
	matrix::Vector3f _angular_velocity;

	float _motor_rps[4]{0.f, 0.f, 0.f, 0.f};
	hrt_abstime _motor_rps_timestamp[4]{0, 0, 0, 0};
	double _alloc_matrix[24]{};

	perf_counter_t _loop_perf;

	hrt_abstime _last_run{0};
	hrt_abstime _time_offboard_enabled{0};

	uint32_t _seq{0};
	bool _need_reinit{true};
	bool _has_new_control{false};
	control_packet_t _latest_control{};
	param_t _voxl_esc_rpm_min_handle{PARAM_INVALID};
	param_t _voxl_esc_rpm_max_handle{PARAM_INVALID};
	int32_t _output_min_rpm{0};
	int32_t _output_max_rpm{0};
	bool _has_output_rpm_limits{false};
	bool _warned_output_rpm_mismatch{false};
	bool _warned_hover_rpm_limit{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_NMPC_MASS>)  _param_mass,
		(ParamFloat<px4::params::MC_NMPC_IXX>)   _param_ixx,
		(ParamFloat<px4::params::MC_NMPC_IXY>)   _param_ixy,
		(ParamFloat<px4::params::MC_NMPC_IXZ>)   _param_ixz,
		(ParamFloat<px4::params::MC_NMPC_IYY>)   _param_iyy,
		(ParamFloat<px4::params::MC_NMPC_IYZ>)   _param_iyz,
		(ParamFloat<px4::params::MC_NMPC_IZZ>)   _param_izz,
		(ParamFloat<px4::params::MC_NMPC_KF1>)   _param_kf1,
		(ParamFloat<px4::params::MC_NMPC_KF2>)   _param_kf2,
		(ParamFloat<px4::params::MC_NMPC_KF3>)   _param_kf3,
		(ParamFloat<px4::params::MC_NMPC_KF4>)   _param_kf4,
		(ParamFloat<px4::params::MC_NMPC_TC1>)   _param_tc1,
		(ParamFloat<px4::params::MC_NMPC_TC2>)   _param_tc2,
		(ParamFloat<px4::params::MC_NMPC_TC3>)   _param_tc3,
		(ParamFloat<px4::params::MC_NMPC_TC4>)   _param_tc4,
		(ParamFloat<px4::params::MC_NMPC_COMX>)  _param_com_x,
		(ParamFloat<px4::params::MC_NMPC_COMY>)  _param_com_y,
		(ParamFloat<px4::params::MC_NMPC_COMZ>)  _param_com_z,
		(ParamInt<px4::params::MC_NMPC_MINRPM>)  _param_min_rpm,
		(ParamInt<px4::params::MC_NMPC_MAXRPM>)  _param_max_rpm,
		(ParamBool<px4::params::MC_NMPC_VERBOSE>) _param_verbose
	)
};
