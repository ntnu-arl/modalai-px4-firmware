#pragma once

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
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
#include <parameters/param.h>

#include "protocol.h"

using namespace time_literals;

// Setpoint in the NMPC/ENU frame: x=East, y=North, z=Up.
// Use the trajectory helpers to generate setpoints without touching PX4 NED
// internals.
struct NmpcSetpoint {
	float pos[3]; // ENU: [East, North, Up]  (m)
	float vel[3]; // ENU: [East, North, Up]  (m/s)
	uint8_t cost_weight_set{NMPC_COST_WEIGHT_SET_REGULAR_FLIGHT};
	uint8_t control_mode{0};
};

struct NmpcSetpointPair {
	NmpcSetpoint nominal{};
	NmpcSetpoint solver{};
};

class MulticopterNmpcControl : public ModuleBase<MulticopterNmpcControl>, public px4::WorkItem
{
public:
	MulticopterNmpcControl();
	~MulticopterNmpcControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	bool pack_state(state_packet_t *pkt);
	void publish_actuator_motors(const control_packet_t *pkt);
	void publish_offboard_control_mode(bool use_px4_position_control);
	void publish_trajectory_setpoint(const trajectory_setpoint_s &traj_sp);
	bool loadActuatorMappingParams();
	void updateEscTelemetryCache();
	int mapEscReportToMotorIndex(const esc_report_s &report) const;
	void generateFailsafeTrajectory(trajectory_setpoint_s &traj_sp,
					 const matrix::Vector3f &position,
					 const matrix::Quatf &attitude);

	// Trajectory setpoint sequence — all inputs/outputs in NMPC/ENU frame.
	// initial_pos_enu: drone position at offboard-enable time, in ENU (m).
	// t_now drives time-based setpoint advancement.
	NmpcSetpointPair get_setpoint_sequence(const matrix::Vector3f &initial_pos_enu,
					      const matrix::Vector3f &current_pos_enu,
					      hrt_abstime t_now);

	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};

	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<nmpc_state_data_s> _nmpc_state_pub{ORB_ID(nmpc_state_data)};
	uORB::Subscription _nmpc_control_sub{ORB_ID(nmpc_control_data)};

	vehicle_control_mode_s _vehicle_control_mode{};
	trajectory_setpoint_s _trajectory_setpoint{};

	matrix::Quatf _attitude;
	matrix::Vector3f _position;
	matrix::Vector3f _velocity;
	matrix::Vector3f _angular_velocity;
	matrix::Vector3f _initial_position{0.f, 0.f, 0.f};

	perf_counter_t _loop_perf;

	hrt_abstime _last_run{0};
	hrt_abstime _position_velocity_timestamp_us{0};
	hrt_abstime _attitude_timestamp_us{0};
	hrt_abstime _angular_velocity_timestamp_us{0};
	hrt_abstime _last_state_error_report_us{0};
	double _motor_rps_meas[NU] {};
	uint64_t _motor_rps_timestamp_us[NU] {};

	param_t _param_thr_mdl_fac{PARAM_INVALID};
	param_t _param_voxl_esc_rpm_min{PARAM_INVALID};
	param_t _param_voxl_esc_rpm_max{PARAM_INVALID};
	float _actuator_thr_mdl_fac{NAN};
	float _actuator_rpm_min{NAN};
	float _actuator_rpm_max{NAN};

	uint32_t _seq{0};
	uint32_t _min_valid_control_seq{0};
	uint32_t _collision_setpoint_index{0};
	bool _need_reinit{true};
	bool _has_valid_control{false};
	bool _using_px4_position_control{false};
	uint8_t _active_cost_weight_set{NMPC_COST_WEIGHT_SET_REGULAR_FLIGHT};
	hrt_abstime _collision_setpoint_start{0};
	float _collision_prev_rel_x_enu{NAN};
	float _px4_position_control_yaw{NAN};
	control_packet_t _latest_control{};

};
