#include "MulticopterNmpcControl.hpp"

#include <drivers/drv_hrt.h>
#include <math.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <stdio.h>

using namespace matrix;

namespace
{
// Allocation matrix B (6x4) in column-major order for CasADi.
// Maps motor forces to body wrench [Fx, Fy, Fz, Tx, Ty, Tz].
// Motor positions (FLU): M0=[0.16,-0.16,0], M1=[-0.16,-0.16,0], M2=[-0.16,0.16,0], M3=[0.16,0.16,0]
// Thrust dirs: all [0,0,1]. Motor dirs: [1,-1,1,-1]. cq=0.01.
// Column k = [0, 0, 1, cross(pos_k,[0,0,1]) + dir_k*0.01*[0,0,1]]
static constexpr double ALLOC_MATRIX_COLMAJOR[24] = {
	0.0,  0.0,  1.0, -0.16, -0.16,  0.01,   // motor 0
	0.0,  0.0,  1.0, -0.16,  0.16, -0.01,   // motor 1
	0.0,  0.0,  1.0,  0.16,  0.16,  0.01,   // motor 2
	0.0,  0.0,  1.0,  0.16, -0.16, -0.01,   // motor 3
};

// Hardcoded NMPC physical parameters — change here and reflash.
static constexpr float NMPC_MASS        = 0.363f;          // [kg] total vehicle mass
static constexpr float NMPC_IXX         = 0.0004933f;      // [kg·m²] inertia
static constexpr float NMPC_IXY         = 0.0f;            // [kg·m²] inertia cross term
static constexpr float NMPC_IXZ         = 0.0f;            // [kg·m²] inertia cross term
static constexpr float NMPC_IYY         = 0.0005977f;      // [kg·m²] inertia
static constexpr float NMPC_IYZ         = 0.0f;            // [kg·m²] inertia cross term
static constexpr float NMPC_IZZ         = 0.0008339f;      // [kg·m²] inertia
static constexpr float NMPC_KF1         = 0.00001286412f;  // [N/(rad/s)²] thrust coefficient motor 1
static constexpr float NMPC_KF2         = 0.00001286412f;  // [N/(rad/s)²] thrust coefficient motor 2
static constexpr float NMPC_KF3         = 0.00001286412f;  // [N/(rad/s)²] thrust coefficient motor 3
static constexpr float NMPC_KF4         = 0.00001286412f;  // [N/(rad/s)²] thrust coefficient motor 4
static constexpr float NMPC_TC1         = 0.047f;          // [s] motor time constant 1 (motor index 5)
static constexpr float NMPC_TC2         = 0.047f;          // [s] motor time constant 2 (motor index 5)
static constexpr float NMPC_TC3         = 0.047f;          // [s] motor time constant 3 (motor index 5)
static constexpr float NMPC_TC4         = 0.047f;          // [s] motor time constant 4 (motor index 5)
static constexpr float NMPC_COM_X       = 0.0f;            // [m] COM offset from base link, body frame x
static constexpr float NMPC_COM_Y       = 0.0f;            // [m] COM offset from base link, body frame y
static constexpr float NMPC_COM_Z       = 0.0f;            // [m] COM offset from base link, body frame z

#define NMPC_USE_CALIBRATED_DISTURBANCE
#ifdef NMPC_USE_CALIBRATED_DISTURBANCE
static constexpr double NMPC_DIST_FORCE_BX  = -0.088633000;
static constexpr double NMPC_DIST_FORCE_BY  = 0.101264000;
static constexpr double NMPC_DIST_FORCE_BZ  = 0.062792000;
static constexpr double NMPC_DIST_TORQUE_BX = -0.000712000;
static constexpr double NMPC_DIST_TORQUE_BY = -0.008205000;
static constexpr double NMPC_DIST_TORQUE_BZ = 0.002793000;
#endif
}

MulticopterNmpcControl::MulticopterNmpcControl() :
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

MulticopterNmpcControl::~MulticopterNmpcControl()
{
	perf_free(_loop_perf);
}

bool MulticopterNmpcControl::init()
{
	_param_thr_mdl_fac = param_find("THR_MDL_FAC");
	_param_voxl_esc_rpm_min = param_find("VOXL_ESC_RPM_MIN");
	_param_voxl_esc_rpm_max = param_find("VOXL_ESC_RPM_MAX");

	if (_param_thr_mdl_fac == PARAM_INVALID || _param_voxl_esc_rpm_min == PARAM_INVALID
	    || _param_voxl_esc_rpm_max == PARAM_INVALID) {
		PX4_ERR("required actuator mapping params missing");
		return false;
	}

	if (!loadActuatorMappingParams()) {
		return false;
	}

	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// _vehicle_angular_velocity_sub.set_interval_us(10_ms);

	return true;
}

bool MulticopterNmpcControl::loadActuatorMappingParams()
{
	int32_t rpm_min_i = 0;
	int32_t rpm_max_i = 0;

	if (param_get(_param_thr_mdl_fac, &_actuator_thr_mdl_fac) != PX4_OK
	    || param_get(_param_voxl_esc_rpm_min, &rpm_min_i) != PX4_OK
	    || param_get(_param_voxl_esc_rpm_max, &rpm_max_i) != PX4_OK) {
		PX4_ERR("failed to read actuator mapping params");
		return false;
	}

	_actuator_rpm_min = (float)rpm_min_i;
	_actuator_rpm_max = (float)rpm_max_i;

	if (!PX4_ISFINITE(_actuator_thr_mdl_fac) || !PX4_ISFINITE(_actuator_rpm_min) || !PX4_ISFINITE(_actuator_rpm_max)) {
		PX4_ERR("actuator mapping params not finite");
		return false;
	}

	if (_actuator_thr_mdl_fac < 0.0f || _actuator_thr_mdl_fac > 1.0f) {
		PX4_ERR("invalid THR_MDL_FAC %.3f", (double)_actuator_thr_mdl_fac);
		return false;
	}

	if (!(_actuator_rpm_max > _actuator_rpm_min) || _actuator_rpm_min < 0.0f) {
		PX4_ERR("invalid ESC RPM range min=%.1f max=%.1f",
			(double)_actuator_rpm_min, (double)_actuator_rpm_max);
		return false;
	}

	return true;
}

// ----------------------------------------------------------------------------
// Trajectory generators — ENU (NMPC) frame throughout.
// ----------------------------------------------------------------------------

NmpcSetpoint MulticopterNmpcControl::get_setpoint_initial_position(const Vector3f &initial_pos_enu)
{
	NmpcSetpoint sp{};
	sp.pos[0] = initial_pos_enu(0); // East
	sp.pos[1] = initial_pos_enu(1); // North
	sp.pos[2] = initial_pos_enu(2); // Up
	sp.vel[0] = 0.f;
	sp.vel[1] = 0.f;
	sp.vel[2] = 0.f;
	return sp;
}

NmpcSetpoint MulticopterNmpcControl::get_setpoint_circle(const Vector3f &initial_pos_enu,
							  hrt_abstime t_start, hrt_abstime t_now)
{
	// ---- tuneable parameters ----
	static constexpr float circle_diameter = 0.2f; // [m] full diameter of the circle
	static constexpr float cycle_time      = 4.0f; // [s] duration of one full revolution
	static constexpr float settle_time     = 4.0f; // [s] hold before starting motion
	static constexpr int   n_cycles        = 3;    // stop tracking after this many loops
	// -----------------------------

	const float r     = circle_diameter * 0.5f;
	const float omega = 2.f * M_PI_F / cycle_time; // [rad/s]

	// Elapsed time since offboard was enabled [s]
	const float t = (t_now > t_start) ? (float)(t_now - t_start) * 1e-6f : 0.f;

	// Circle center is offset so that theta=0 lands exactly on initial_pos_enu,
	// giving a continuous position at the settle->fly transition.
	//   pos(theta) = [cx + r*cos(theta),  cy + r*sin(theta),  cz]
	//   pos(0)     = [cx + r, cy, cz]  =>  cx = initial(0) - r
	const float cx = initial_pos_enu(0) - r; // East  component of center
	const float cy = initial_pos_enu(1);      // North component of center
	const float cz = initial_pos_enu(2);      // Up    component of center (constant altitude)

	NmpcSetpoint sp{};
	sp.pos[2] = cz;
	sp.vel[2] = 0.f;

	if (t < settle_time) {
		// Hold at initial position (= circle start, theta=0) — no motion.
		sp.pos[0] = cx + r; // == initial_pos_enu(0)
		sp.pos[1] = cy;     // == initial_pos_enu(1)
		sp.vel[0] = 0.f;
		sp.vel[1] = 0.f;

	} else {
		const float t_fly           = t - settle_time;
		const float total_fly_time  = (float)n_cycles * cycle_time;

		if (t_fly >= total_fly_time) {
			// After n_cycles the drone holds at the end position.
			// n_cycles full revolutions bring theta back to 0, i.e. initial_pos_enu.
			sp.pos[0] = cx + r;
			sp.pos[1] = cy;
			sp.vel[0] = 0.f;
			sp.vel[1] = 0.f;

		} else {
			// Active circle tracking.
			// theta increases CCW in the ENU x-y (East-North) plane.
			const float theta = omega * t_fly;
			const float c = cosf(theta);
			const float s = sinf(theta);
			sp.pos[0] = cx + r * c;
			sp.pos[1] = cy + r * s;
			// Tangential velocity (d/dt of position, CCW):
			sp.vel[0] = -r * omega * s;
			sp.vel[1] =  r * omega * c;
		}
	}

	return sp;
}

// ----------------------------------------------------------------------------

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
	pkt->flags = 0;
	pkt->sample_timestamp_us = _last_run;

	if (_need_reinit) {
		pkt->flags |= FLAG_REINIT;
	}

	pkt->pad[0] = pkt->pad[1] = pkt->pad[2] = 0;

	// Frame transforms matching mc_neural_control (PX4 -> simulation):
	// frame_transf = diag(1,-1,-1)                  (FRD -> FLU body frame)
	// frame_transf_2 = [[0,1,0],[-1,0,0],[0,0,1]]   (-90 deg about Z)
	// Position/velocity: frame_transf * frame_transf_2 => (x,y,z) -> (y, x, -z)  (NED -> ENU)
	// Angular velocity:  frame_transf => (x,y,z) -> (x, -y, -z)                  (FRD -> FLU)
	// Attitude: frame_transf * (frame_transf_2 * R) * frame_transf^T
	//   equivalent quaternion: q_local = (q_ft * q_ft2) * q_body * conj(q_ft)

	// rigid_body_state[0:3] = position (NED -> ENU)
	pkt->rigid_body_state[0] = (double)_position(1);
	pkt->rigid_body_state[1] = (double)_position(0);
	pkt->rigid_body_state[2] = (double)(-_position(2));

	// rigid_body_state[3:6] = linear velocity (NED -> ENU)
	pkt->rigid_body_state[3] = (double)_velocity(1);
	pkt->rigid_body_state[4] = (double)_velocity(0);
	pkt->rigid_body_state[5] = (double)(-_velocity(2));

	// rigid_body_state[6:10] = quaternion (FRD -> FLU)
	static const Quatf q_ft(0.0f, 1.0f, 0.0f, 0.0f);
	static const Quatf q_ft2(0.7071068f, 0.0f, 0.0f, -0.7071068f);
	static const Quatf q_ft_conj(0.0f, -1.0f, 0.0f, 0.0f);
	static const Quatf q_combined = q_ft * q_ft2;
	Quatf q_local = q_combined * _attitude * q_ft_conj;
	pkt->rigid_body_state[6] = (double)q_local(0);
	pkt->rigid_body_state[7] = (double)q_local(1);
	pkt->rigid_body_state[8] = (double)q_local(2);
	pkt->rigid_body_state[9] = (double)q_local(3);

	// rigid_body_state[10:13] = body angular velocity (FRD -> FLU)
	pkt->rigid_body_state[10] = (double)_angular_velocity(0);
	pkt->rigid_body_state[11] = (double)(-_angular_velocity(1));
	pkt->rigid_body_state[12] = (double)(-_angular_velocity(2));

	// p[0] = mass
	pkt->p[0] = (double)NMPC_MASS;

	// p[1:7] = inertia elements (Ixx, Ixy, Ixz, Iyy, Iyz, Izz)
	pkt->p[1] = (double)NMPC_IXX;
	pkt->p[2] = (double)NMPC_IXY;
	pkt->p[3] = (double)NMPC_IXZ;
	pkt->p[4] = (double)NMPC_IYY;
	pkt->p[5] = (double)NMPC_IYZ;
	pkt->p[6] = (double)NMPC_IZZ;

	// p[7:10] = gravity vector (NED -> ENU)
	pkt->p[7] = 0.0;
	pkt->p[8] = 0.0;
	pkt->p[9] = -9.81;

	// p[10:13] = setpoint position (NED -> ENU)
	pkt->p[10] = (double)_trajectory_setpoint.position[1];
	pkt->p[11] = (double)_trajectory_setpoint.position[0];
	pkt->p[12] = (double)(-_trajectory_setpoint.position[2]);

	// p[13:16] = setpoint velocity (NED -> ENU)
	pkt->p[13] = (double)_trajectory_setpoint.velocity[1];
	pkt->p[14] = (double)_trajectory_setpoint.velocity[0];
	pkt->p[15] = (double)(-_trajectory_setpoint.velocity[2]);

	// p[16:20] = setpoint quaternion, built directly in ENU frame.
	// _trajectory_setpoint.yaw is ENU yaw: 0 = face East (+x), pi/2 = face North (+y).
	// Matches Python NMPC: q_ref = [cos(yaw/2), 0, 0, sin(yaw/2)].
	const float half_yaw = _trajectory_setpoint.yaw * 0.5f;
	pkt->p[16] = (double)cosf(half_yaw);
	pkt->p[17] = 0.0;
	pkt->p[18] = 0.0;
	pkt->p[19] = (double)sinf(half_yaw);

	// p[20:44] = allocation matrix (6x4, column-major for CasADi reshape)
	memcpy(&pkt->p[20], ALLOC_MATRIX_COLMAJOR, 24 * sizeof(double));

	// p[44:48] = thrust coefficients
	pkt->p[44] = (double)NMPC_KF1;
	pkt->p[45] = (double)NMPC_KF2;
	pkt->p[46] = (double)NMPC_KF3;
	pkt->p[47] = (double)NMPC_KF4;

	// p[48:52] = motor time constants
	pkt->p[48] = (double)NMPC_TC1;
	pkt->p[49] = (double)NMPC_TC2;
	pkt->p[50] = (double)NMPC_TC3;
	pkt->p[51] = (double)NMPC_TC4;

	// p[52:55] = COM offset
	pkt->p[52] = (double)NMPC_COM_X;
	pkt->p[53] = (double)NMPC_COM_Y;
	pkt->p[54] = (double)NMPC_COM_Z;

#ifdef NMPC_USE_CALIBRATED_DISTURBANCE
	pkt->p[55] = NMPC_DIST_FORCE_BX;
	pkt->p[56] = NMPC_DIST_FORCE_BY;
	pkt->p[57] = NMPC_DIST_FORCE_BZ;
	pkt->p[58] = NMPC_DIST_TORQUE_BX;
	pkt->p[59] = NMPC_DIST_TORQUE_BY;
	pkt->p[60] = NMPC_DIST_TORQUE_BZ;
#else
	pkt->p[55] = 0.0;
	pkt->p[56] = 0.0;
	pkt->p[57] = 0.0;
	pkt->p[58] = 0.0;
	pkt->p[59] = 0.0;
	pkt->p[60] = 0.0;
#endif
}

void MulticopterNmpcControl::publish_actuator_motors(const control_packet_t *pkt)
{
	actuator_motors_s actuator_motors{};
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _last_run;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}

	const float max_rpm = _actuator_rpm_max;
	const float min_rpm = _actuator_rpm_min;
	const float rpm_range = max_rpm - min_rpm;
	const float thrust_factor = _actuator_thr_mdl_fac;

	for (int i = 0; i < 4; i++) {
		const float desired_rpm = math::max((float)pkt->u[i], 0.0f) * 60.0f;
		const float cmd = (desired_rpm * 2.0f - max_rpm - min_rpm) / rpm_range;
		const float x = (cmd + 1.0f) / 2.0f;
		float control = x;

		if (thrust_factor > 0.0f && thrust_factor <= 1.0f) {
			const float a = thrust_factor;
			const float b = 1.0f - a;
			const float tmp1 = b / (2.0f * a);
			const float tmp2 = b * b / (4.0f * a * a);
			control = a * ((x + tmp1) * (x + tmp1) - tmp2);
		}

		actuator_motors.control[i] = PX4_ISFINITE(control) ? math::constrain(control, 0.0f, 1.0f) : NAN;
	}

	_actuator_motors_pub.publish(actuator_motors);
}

void MulticopterNmpcControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

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

		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_offboard_enabled = _vehicle_control_mode.flag_control_offboard_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!previous_offboard_enabled && _vehicle_control_mode.flag_control_offboard_enabled) {
					_time_offboard_enabled = _vehicle_control_mode.timestamp;
					_initial_position = _position;
					_min_valid_control_seq = _seq;
					_need_reinit = true;
					_has_valid_control = false;
				} else if (previous_offboard_enabled && !_vehicle_control_mode.flag_control_offboard_enabled) {
					generateFailsafeTrajectory(_trajectory_setpoint, _position, _attitude);
					_min_valid_control_seq = _seq;
					_need_reinit = true;
					_has_valid_control = false;
				}
			}
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
			// NED->ENU: ENU_x=NED_y (East), ENU_y=NED_x (North), ENU_z=-NED_z (Up)
			const Vector3f initial_pos_enu(
				_initial_position(1),   // East  = NED_y
				_initial_position(0),   // North = NED_x
				-_initial_position(2)   // Up    = -NED_z
			);

			// NOTE: swap get_setpoint_circle <-> get_setpoint_initial_position to change mode.
			// const NmpcSetpoint sp = get_setpoint_circle(initial_pos_enu, _time_offboard_enabled, _last_run);
			const NmpcSetpoint sp = get_setpoint_initial_position(initial_pos_enu);

			// Convert ENU setpoint back to NED for _trajectory_setpoint.
			// ENU->NED: NED_x=ENU_y (North), NED_y=ENU_x (East), NED_z=-ENU_z (Down)
			_trajectory_setpoint.position[0] = sp.pos[1];   // North = ENU_y
			_trajectory_setpoint.position[1] = sp.pos[0];   // East  = ENU_x
			_trajectory_setpoint.position[2] = -sp.pos[2];  // Down  = -ENU_z
			_trajectory_setpoint.velocity[0] = sp.vel[1];   // North = ENU_y
			_trajectory_setpoint.velocity[1] = sp.vel[0];   // East  = ENU_x
			_trajectory_setpoint.velocity[2] = -sp.vel[2];  // Down  = -ENU_z
			_trajectory_setpoint.acceleration[0] = 0.0f;
			_trajectory_setpoint.acceleration[1] = 0.0f;
			_trajectory_setpoint.acceleration[2] = 0.0f;
			_trajectory_setpoint.yaw = 0.0f;
			_trajectory_setpoint.yawspeed = 0.0f;
			_trajectory_setpoint.timestamp = _last_run;

			nmpc_control_data_s ctrl_msg{};
			bool have_control_update = false;

			while (_nmpc_control_sub.update(&ctrl_msg)) {
				have_control_update = true;
			}

			if (have_control_update && ctrl_msg.seq >= _min_valid_control_seq) {
				_latest_control.seq = ctrl_msg.seq;
				_latest_control.status = ctrl_msg.status;
				memcpy(_latest_control.u, ctrl_msg.u, sizeof(ctrl_msg.u));
				_latest_control.solve_time_us = ctrl_msg.solve_time_us;
				memcpy(_latest_control.quat_next, ctrl_msg.quat_next, sizeof(ctrl_msg.quat_next));
				if (ctrl_msg.status != 0) {
					_need_reinit = true;
					_has_valid_control = false;
				} else {
					_has_valid_control = true;
				}
			}

			if (_has_valid_control) {
				publish_actuator_motors(&_latest_control);
			}

			state_packet_t pkt_state;
			pack_state(&pkt_state);

			nmpc_state_data_s state_msg{};
			state_msg.timestamp = hrt_absolute_time();
			state_msg.seq = pkt_state.seq;
			state_msg.flags = pkt_state.flags;
			state_msg.sample_timestamp_us = pkt_state.sample_timestamp_us;
			memcpy(state_msg.rigid_body_state, pkt_state.rigid_body_state, sizeof(pkt_state.rigid_body_state));
			memcpy(state_msg.p, pkt_state.p, sizeof(pkt_state.p));
			_nmpc_state_pub.publish(state_msg);
			_need_reinit = false;
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
