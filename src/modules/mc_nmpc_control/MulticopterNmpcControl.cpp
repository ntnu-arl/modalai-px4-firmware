#include "MulticopterNmpcControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <stdio.h>

using namespace matrix;

MulticopterNmpcControl::MulticopterNmpcControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	parameters_updated();
}

MulticopterNmpcControl::~MulticopterNmpcControl()
{
	perf_free(_loop_perf);
}

bool MulticopterNmpcControl::load_allocation_matrix()
{
	FILE *f = fopen("/home/model_files/alloc_matrix.csv", "r");
	if (!f) {
		PX4_ERR("alloc_matrix.csv not found at /home/model_files/alloc_matrix.csv");
		return false;
	}
	for (int i = 0; i < 24; i++) {
		if (fscanf(f, " %lf", &_alloc_matrix[i]) != 1) {
			PX4_ERR("alloc_matrix.csv: failed to read entry %d", i);
			fclose(f);
			return false;
		}
		int c = fgetc(f);
		if (c != ',') { ungetc(c, f); }
	}
	fclose(f);

	// Convert allocation matrix from FLU body frame (simulation) to FRD body frame (PX4).
	// Negate rows 1,2,4,5 (Fy, Fz, Ty, Tz) of the 6xN column-major matrix.
	for (int col = 0; col < 4; col++) {
		_alloc_matrix[6 * col + 1] = -_alloc_matrix[6 * col + 1];
		_alloc_matrix[6 * col + 2] = -_alloc_matrix[6 * col + 2];
		_alloc_matrix[6 * col + 4] = -_alloc_matrix[6 * col + 4];
		_alloc_matrix[6 * col + 5] = -_alloc_matrix[6 * col + 5];
	}

	PX4_INFO("alloc_matrix loaded (converted FLU->FRD)");
	return true;
}

bool MulticopterNmpcControl::init()
{
	if (!load_allocation_matrix()) {
		return false;
	}

	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// _vehicle_angular_velocity_sub.set_interval_us(10_ms);

	return true;
}

void MulticopterNmpcControl::parameters_updated()
{
	if (_param_min_rpm.get() <= 0 || _param_max_rpm.get() <= 0 || _param_max_rpm.get() <= _param_min_rpm.get()) {
		PX4_ERR("MC_NMPC_MINRPM=%d MC_NMPC_MAXRPM=%d are not available or invalid, cannot proceed",
			_param_min_rpm.get(), _param_max_rpm.get());
	}
}

void MulticopterNmpcControl::generateFailsafeTrajectory(trajectory_setpoint_s &traj_sp,
							 const Vector3f &position,
							 const Quatf &attitude)
{
	position.copyTo(traj_sp.position);
	traj_sp.yaw = Eulerf(attitude).psi();

	const Vector3f zero(0.0f, 0.0f, 0.0f);
	zero.copyTo(traj_sp.velocity);
	zero.copyTo(traj_sp.acceleration);
	traj_sp.yawspeed = 0.0f;
}

void MulticopterNmpcControl::pack_state(state_packet_t *pkt)
{
	pkt->seq = _seq++;
	pkt->flags = _need_reinit ? FLAG_REINIT : 0;
	pkt->pad[0] = pkt->pad[1] = pkt->pad[2] = 0;

	// x0[0:3] = position
	pkt->x0[0] = (double)_position(0);
	pkt->x0[1] = (double)_position(1);
	pkt->x0[2] = (double)_position(2);

	// x0[3:6] = linear velocity
	pkt->x0[3] = (double)_velocity(0);
	pkt->x0[4] = (double)_velocity(1);
	pkt->x0[5] = (double)_velocity(2);

	// x0[6:10] = quaternion (w, x, y, z)
	pkt->x0[6] = (double)_attitude(0);
	pkt->x0[7] = (double)_attitude(1);
	pkt->x0[8] = (double)_attitude(2);
	pkt->x0[9] = (double)_attitude(3);

	// x0[10:13] = body angular velocity
	pkt->x0[10] = (double)_angular_velocity(0);
	pkt->x0[11] = (double)_angular_velocity(1);
	pkt->x0[12] = (double)_angular_velocity(2);

	// x0[13:17] = motor RPS states
	for (int i = 0; i < 4; i++) {
		pkt->x0[13 + i] = (double)_motor_rps[i];
	}

	// p[0] = mass
	pkt->p[0] = (double)_param_mass.get();

	// p[1:7] = inertia elements (Ixx, Ixy, Ixz, Iyy, Iyz, Izz)
	pkt->p[1] = (double)_param_ixx.get();
	pkt->p[2] = (double)_param_ixy.get();
	pkt->p[3] = (double)_param_ixz.get();
	pkt->p[4] = (double)_param_iyy.get();
	pkt->p[5] = (double)_param_iyz.get();
	pkt->p[6] = (double)_param_izz.get();

	// p[7:10] = gravity vector
	pkt->p[7] = 0.0;
	pkt->p[8] = 0.0;
	pkt->p[9] = 9.81;

	// p[10:13] = setpoint position
	pkt->p[10] = (double)_trajectory_setpoint.position[0];
	pkt->p[11] = (double)_trajectory_setpoint.position[1];
	pkt->p[12] = (double)_trajectory_setpoint.position[2];

	// p[13:16] = setpoint velocity
	pkt->p[13] = (double)_trajectory_setpoint.velocity[0];
	pkt->p[14] = (double)_trajectory_setpoint.velocity[1];
	pkt->p[15] = (double)_trajectory_setpoint.velocity[2];

	// p[16:20] = setpoint quaternion (derived from yaw setpoint)
	Quatf q_sp(Eulerf(0.0f, 0.0f, _trajectory_setpoint.yaw));
	pkt->p[16] = (double)q_sp(0);
	pkt->p[17] = (double)q_sp(1);
	pkt->p[18] = (double)q_sp(2);
	pkt->p[19] = (double)q_sp(3);

	// p[20:44] = allocation matrix (24 values loaded from /home/model_files/alloc_matrix.csv)
	memcpy(&pkt->p[20], _alloc_matrix, 24 * sizeof(double));

	// p[44:48] = thrust coefficients
	pkt->p[44] = (double)_param_kf1.get();
	pkt->p[45] = (double)_param_kf2.get();
	pkt->p[46] = (double)_param_kf3.get();
	pkt->p[47] = (double)_param_kf4.get();

	// p[48:52] = motor time constants
	pkt->p[48] = (double)_param_tc1.get();
	pkt->p[49] = (double)_param_tc2.get();
	pkt->p[50] = (double)_param_tc3.get();
	pkt->p[51] = (double)_param_tc4.get();

	// p[52:55] = COM offset
	pkt->p[52] = (double)_param_com_x.get();
	pkt->p[53] = (double)_param_com_y.get();
	pkt->p[54] = (double)_param_com_z.get();

	// hover force: mass * gravity
	pkt->hover_force = (double)_param_mass.get() * 9.81 / 4.0;
}

void MulticopterNmpcControl::publish_actuator_motors(const control_packet_t *pkt)
{
	actuator_motors_s actuator_motors{};
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _last_run;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}

	const float max_rpm = (float)_param_max_rpm.get();
	const float min_rpm = (float)_param_min_rpm.get();
	const float rpm_range = max_rpm - min_rpm;

	static constexpr int motor_map[4] = {0, 2, 3, 1};

	static constexpr float a = 0.8f;
	static constexpr float b = 1.0f - a;
	static constexpr float tmp1 = b / (2.0f * a);
	static constexpr float tmp2 = b * b / (4.0f * a * a);

	for (int i = 0; i < 4; i++) {
		const float desired_rpm = math::max((float)pkt->u[motor_map[i]], 0.0f) * 60.0f;
		const float cmd = (desired_rpm * 2.0f - max_rpm - min_rpm) / rpm_range;
		const float x = (cmd + 1.0f) / 2.0f;
		const float control = a * ((x + tmp1) * (x + tmp1) - tmp2);
		actuator_motors.control[i] = PX4_ISFINITE(control) ? math::constrain(control, 0.0f, 1.0f) : NAN;
	}

	_actuator_motors_pub.publish(actuator_motors);
}

void MulticopterNmpcControl::update_motor_feedback(const esc_status_s &esc_status)
{
	for (int i = 0; i < math::min((int)esc_status.esc_count, (int)esc_status_s::CONNECTED_ESC_MAX); i++) {
		const esc_report_s &esc = esc_status.esc[i];
		int motor_index = -1;

		if ((esc.actuator_function >= actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1)
		    && (esc.actuator_function < actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 + 4)) {
			motor_index = esc.actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;

		} else if ((esc.actuator_function == 0) && (esc.esc_address >= 1) && (esc.esc_address <= 4)) {
			motor_index = esc.esc_address - 1;
		}

		if ((motor_index >= 0) && (motor_index < 4) && (esc.timestamp > 0)) {
			_motor_rps[motor_index] = math::max(esc.esc_rpm / 60.f, 0.f);
			_motor_rps_timestamp[motor_index] = esc.timestamp;
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	for (int i = 0; i < 4; i++) {
		if ((_motor_rps_timestamp[i] == 0) || ((now - _motor_rps_timestamp[i]) > MOTOR_FEEDBACK_TIMEOUT)) {
			_motor_rps[i] = 0.f;
		}
	}
}

void MulticopterNmpcControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		parameters_updated();
	}

	vehicle_angular_velocity_s vehicle_angular_velocity;
	if (_vehicle_angular_velocity_sub.update(&vehicle_angular_velocity)) {
		_last_run = vehicle_angular_velocity.timestamp_sample;
		_angular_velocity = Vector3f(vehicle_angular_velocity.xyz);

		if (_vehicle_attitude_sub.updated()) {
			vehicle_attitude_s vehicle_attitude;
			if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
				_attitude = Quatf(vehicle_attitude.q);
			}
		}

		vehicle_local_position_s vehicle_local_position;
		if (_vehicle_local_position_sub.update(&vehicle_local_position)) {
			_position = Vector3f(vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z);
			_velocity = Vector3f(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		}

		if (_esc_status_sub.updated()) {
			esc_status_s esc_status;
			if (_esc_status_sub.copy(&esc_status)) {
				update_motor_feedback(esc_status);
			}
		}

		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_offboard_enabled = _vehicle_control_mode.flag_control_offboard_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!previous_offboard_enabled && _vehicle_control_mode.flag_control_offboard_enabled) {
					_time_offboard_enabled = _vehicle_control_mode.timestamp;
					_need_reinit = true;
				} else if (previous_offboard_enabled && !_vehicle_control_mode.flag_control_offboard_enabled) {
					generateFailsafeTrajectory(_trajectory_setpoint, _position, _attitude);
					_need_reinit = true;
				}
			}
		}

		if (_trajectory_setpoint_sub.updated()) {
			_trajectory_setpoint_sub.copy(&_trajectory_setpoint);
		}

		// Publish offboard control mode (actuator direct)
		offboard_control_mode_s ocm{};
		ocm.position = false;
		ocm.velocity = false;
		ocm.acceleration = false;
		ocm.attitude = false;
		ocm.body_rate = false;
		ocm.actuator = true;
		ocm.timestamp = hrt_absolute_time();
		_offboard_control_mode_pub.publish(ocm);

		if (_vehicle_control_mode.flag_control_offboard_enabled) {

			if ((_trajectory_setpoint.timestamp < _time_offboard_enabled)
			    || (hrt_elapsed_time(&_trajectory_setpoint.timestamp) > TRAJECTORY_SETPOINT_TIMEOUT)) {
				_trajectory_setpoint.position[0] = _position(0);
				_trajectory_setpoint.position[1] = _position(1);
				_trajectory_setpoint.position[2] = _position(2);
				_trajectory_setpoint.velocity[0] = 0.0f;
				_trajectory_setpoint.velocity[1] = 0.0f;
				_trajectory_setpoint.velocity[2] = 0.0f;
				_trajectory_setpoint.acceleration[0] = 0.0f;
				_trajectory_setpoint.acceleration[1] = 0.0f;
				_trajectory_setpoint.acceleration[2] = 0.0f;
				_trajectory_setpoint.yaw = matrix::Eulerf(_attitude).psi();
				_trajectory_setpoint.yawspeed = 0.0f;
				_trajectory_setpoint.timestamp = _last_run;
			}

			state_packet_t pkt_state;
			pack_state(&pkt_state);

			nmpc_state_data_s state_msg{};
			state_msg.timestamp = hrt_absolute_time();
			state_msg.seq = pkt_state.seq;
			state_msg.flags = pkt_state.flags;
			memcpy(state_msg.x0, pkt_state.x0, sizeof(pkt_state.x0));
			memcpy(state_msg.p, pkt_state.p, sizeof(pkt_state.p));
			state_msg.hover_force = pkt_state.hover_force;
			_nmpc_state_pub.publish(state_msg);
			_need_reinit = false;

			nmpc_control_data_s ctrl_msg;
			if (_nmpc_control_sub.update(&ctrl_msg)) {
				_latest_control.seq = ctrl_msg.seq;
				_latest_control.status = ctrl_msg.status;
				memcpy(_latest_control.u, ctrl_msg.u, sizeof(ctrl_msg.u));
				_latest_control.solve_time_us = ctrl_msg.solve_time_us;
				memcpy(_latest_control.quat_next, ctrl_msg.quat_next, sizeof(ctrl_msg.quat_next));
				if (ctrl_msg.status != 0) {
					_need_reinit = true;
				}
				_has_new_control = true;
			}

			if (_has_new_control) {
				publish_actuator_motors(&_latest_control);
				_has_new_control = false;
			}
		}
	}

	perf_end(_loop_perf);
}

int MulticopterNmpcControl::task_spawn(int argc, char *argv[])
{
	MulticopterNmpcControl *instance = new MulticopterNmpcControl();

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

int MulticopterNmpcControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterNmpcControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
NMPC controller bridge for multicopters. Packs vehicle state, sends to
NMPC solver running on apps processor via pipe, and publishes the
resulting motor commands.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_nmpc_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_nmpc_control_main(int argc, char *argv[])
{
	return MulticopterNmpcControl::main(argc, argv);
}
