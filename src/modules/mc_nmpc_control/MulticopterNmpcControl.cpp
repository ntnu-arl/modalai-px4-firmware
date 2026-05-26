#include "MulticopterNmpcControl.hpp"

#include <drivers/drv_hrt.h>
#include <math.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <stdio.h>
#include <string.h>

using namespace matrix;

namespace
{
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->seq) == sizeof(((state_packet_t *)nullptr)->seq),
	      "nmpc_state_data.seq size must match state_packet_t.seq");
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->flags) == sizeof(((state_packet_t *)nullptr)->flags),
	      "nmpc_state_data.flags size must match state_packet_t.flags");
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->sample_timestamp_us)
		      == sizeof(((state_packet_t *)nullptr)->sample_timestamp_us),
	      "nmpc_state_data.sample_timestamp_us size must match state_packet_t.sample_timestamp_us");
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->position_velocity_timestamp_us)
		      == sizeof(((state_packet_t *)nullptr)->position_velocity_timestamp_us),
	      "nmpc_state_data.position_velocity_timestamp_us size must match state_packet_t.position_velocity_timestamp_us");
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->attitude_timestamp_us)
		      == sizeof(((state_packet_t *)nullptr)->attitude_timestamp_us),
	      "nmpc_state_data.attitude_timestamp_us size must match state_packet_t.attitude_timestamp_us");
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->angular_velocity_timestamp_us)
		      == sizeof(((state_packet_t *)nullptr)->angular_velocity_timestamp_us),
	      "nmpc_state_data.angular_velocity_timestamp_us size must match state_packet_t.angular_velocity_timestamp_us");
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->rigid_body_state)
		      == sizeof(((state_packet_t *)nullptr)->rigid_body_state),
	      "nmpc_state_data.rigid_body_state layout must match state_packet_t.rigid_body_state");
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->p) == sizeof(((state_packet_t *)nullptr)->p),
	      "nmpc_state_data.p layout must match state_packet_t.p");
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->motor_rps_meas)
		      == sizeof(((state_packet_t *)nullptr)->motor_rps_meas),
	      "nmpc_state_data.motor_rps_meas layout must match state_packet_t.motor_rps_meas");
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->motor_rps_timestamp_us)
		      == sizeof(((state_packet_t *)nullptr)->motor_rps_timestamp_us),
	      "nmpc_state_data.motor_rps_timestamp_us layout must match state_packet_t.motor_rps_timestamp_us");
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->motor_rps_valid_mask)
		      == sizeof(((state_packet_t *)nullptr)->motor_rps_valid_mask),
	      "nmpc_state_data.motor_rps_valid_mask size must match state_packet_t.motor_rps_valid_mask");
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->reserved0)
		      == sizeof(((state_packet_t *)nullptr)->reserved0),
	      "nmpc_state_data.reserved0 size must match state_packet_t.reserved0");
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->nominal_position_enu)
		      == sizeof(((state_packet_t *)nullptr)->nominal_position_enu),
	      "nmpc_state_data.nominal_position_enu layout must match state_packet_t.nominal_position_enu");
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->nominal_velocity_enu)
		      == sizeof(((state_packet_t *)nullptr)->nominal_velocity_enu),
	      "nmpc_state_data.nominal_velocity_enu layout must match state_packet_t.nominal_velocity_enu");

static_assert(sizeof(((nmpc_control_data_s *)nullptr)->seq) == sizeof(((control_packet_t *)nullptr)->seq),
	      "nmpc_control_data.seq size must match control_packet_t.seq");
static_assert(sizeof(((nmpc_control_data_s *)nullptr)->status) == sizeof(((control_packet_t *)nullptr)->status),
	      "nmpc_control_data.status size must match control_packet_t.status");
static_assert(sizeof(((nmpc_control_data_s *)nullptr)->u) == sizeof(((control_packet_t *)nullptr)->u),
	      "nmpc_control_data.u layout must match control_packet_t.u");
static_assert(sizeof(((nmpc_control_data_s *)nullptr)->solve_time_us)
		      == sizeof(((control_packet_t *)nullptr)->solve_time_us),
	      "nmpc_control_data.solve_time_us size must match control_packet_t.solve_time_us");
static_assert(sizeof(((nmpc_control_data_s *)nullptr)->quat_next)
		      == sizeof(((control_packet_t *)nullptr)->quat_next),
	      "nmpc_control_data.quat_next layout must match control_packet_t.quat_next");

enum TrajectoryControlMode : uint8_t {
	NMPC_DIRECT_ACTUATOR = 0,
	PX4_POSITION_CONTROL = 1,
};

static constexpr size_t MAX_COLLISION_SETPOINTS = 4;

// Allocation matrix B (6x4) in column-major order for CasADi..
// Maps motor forces to body wrench [Fx, Fy, Fz, Tx, Ty, Tz].
// Motor positions and thrust directions are in the solver body frame (FLU).
// Column k = [thrust_dir_k, cross(pos_k, thrust_dir_k) + motor_dir_k*0.01*thrust_dir_k]
static constexpr double ALLOC_MATRIX_COLMAJOR[24] = {
	 0.16655232523072261, -0.0052755595681712869, 0.9860185046090576,  -0.0016655232150798805, -0.12498569655684248,   -0.010529186268929696, // motor 0
	-0.0019413838124934801, 0.3989773984220194,    0.91695870494656939,  0.12640214708521766,    0.0014794431557976725,  0.01052951504271659,  // motor 1
	-0.1665674759845783,    0.0052907433326074838, 0.9860158639591533,   0.001665674722615062,   0.12498609967044985,   -0.010531090112415764, // motor 2
	 0.001957027819017243, -0.39899377456418594,   0.91695154610324936, -0.12640267618589821,   -0.0014802161330492048,  0.010531392142931346, // motor 3
};

// Hardcoded NMPC physical parameters — change here and reflash.
static constexpr float NMPC_MASS        = 0.371f;          // [kg] total vehicle mass
static constexpr float NMPC_IXX         = 0.001130051f;   // [kg·m²] inertia
static constexpr float NMPC_IXY         = 0.0000177f;     // [kg·m²] inertia cross term
static constexpr float NMPC_IXZ         = -0.00000068f;   // [kg·m²] inertia cross term
static constexpr float NMPC_IYY         = 0.001299153f;   // [kg·m²] inertia
static constexpr float NMPC_IYZ         = -0.0000481047f; // [kg·m²] inertia cross term
static constexpr float NMPC_IZZ         = 0.00161747f;    // [kg·m²] inertia
static constexpr float NMPC_KF1         = 0.00001434f;     // [N/(rad/s)²] thrust coefficient motor 1
static constexpr float NMPC_KF2         = 0.00001434f;     // [N/(rad/s)²] thrust coefficient motor 2
static constexpr float NMPC_KF3         = 0.00001434f;     // [N/(rad/s)²] thrust coefficient motor 3
static constexpr float NMPC_KF4         = 0.00001434f;     // [N/(rad/s)²] thrust coefficient motor 4
static constexpr float NMPC_TC1         = 0.06314;          // [s] motor time constant 1 (motor index 5)
static constexpr float NMPC_TC2         = 0.06314;          // [s] motor time constant 2 (motor index 5)
static constexpr float NMPC_TC3         = 0.06314;          // [s] motor time constant 3 (motor index 5)
static constexpr float NMPC_TC4         = 0.06314;          // [s] motor time constant 4 (motor index 5)
static constexpr float NMPC_COM_X       = 0.0f;            // [m] COM offset from base link, body frame x
static constexpr float NMPC_COM_Y       = 0.0f;            // [m] COM offset from base link, body frame y
static constexpr float NMPC_COM_Z       = 0.0f;            // [m] COM offset from base link, body frame z
static constexpr double NMPC_DIST_FORCE_BX  = 0.0;
static constexpr double NMPC_DIST_FORCE_BY  = 0.0;
static constexpr double NMPC_DIST_FORCE_BZ  = 0.0;
static constexpr double NMPC_DIST_TORQUE_BX = 0.0;
static constexpr double NMPC_DIST_TORQUE_BY = 0.0;
static constexpr double NMPC_DIST_TORQUE_BZ = 0.0;
static constexpr double NMPC_HOVER_FORCE_PER_MOTOR = (double)NMPC_MASS * 9.81 / 4.0;
static constexpr uint64_t MOTOR_RPS_MEAS_TIMEOUT_US = 50000ULL;

uint8_t sanitizeTrajectoryControlMode(uint8_t control_mode)
{
	return control_mode <= PX4_POSITION_CONTROL ? control_mode : NMPC_DIRECT_ACTUATOR;
}

bool readRequiredParamFloat(const char *name, float *value)
{
	const param_t handle = param_find(name);

	if (handle == PARAM_INVALID) {
		PX4_ERR("required NMPC parameter missing: %s", name);
		return false;
	}

	float parsed = NAN;

	if (param_get(handle, &parsed) != PX4_OK) {
		PX4_ERR("failed to read NMPC parameter: %s", name);
		return false;
	}

	if (!PX4_ISFINITE(parsed) || fabsf(parsed) > 1.0e20f) {
		PX4_ERR("NMPC parameter must be set to a valid finite value: %s", name);
		return false;
	}

	*value = parsed;
	return true;
}

bool readRequiredParamInt(const char *name, int32_t *value)
{
	const param_t handle = param_find(name);

	if (handle == PARAM_INVALID) {
		PX4_ERR("required NMPC parameter missing: %s", name);
		return false;
	}

	int32_t parsed = -1;

	if (param_get(handle, &parsed) != PX4_OK) {
		PX4_ERR("failed to read NMPC parameter: %s", name);
		return false;
	}

	*value = parsed;
	return true;
}
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

bool MulticopterNmpcControl::loadNmpcControllerConfig()
{
	float gap_ref1_mm[3] {};
	float gap_ref2_mm[3] {};
	int32_t use_absolute_position = -1;
	int32_t parsed_setpoint_count = -1;

	if (!readRequiredParamInt("NMPC_ABS_POS", &use_absolute_position)
	    || !readRequiredParamFloat("NMPC_G1_XMM", &gap_ref1_mm[0])
	    || !readRequiredParamFloat("NMPC_G1_YMM", &gap_ref1_mm[1])
	    || !readRequiredParamFloat("NMPC_G1_ZMM", &gap_ref1_mm[2])
	    || !readRequiredParamFloat("NMPC_G2_XMM", &gap_ref2_mm[0])
	    || !readRequiredParamFloat("NMPC_G2_YMM", &gap_ref2_mm[1])
	    || !readRequiredParamFloat("NMPC_G2_ZMM", &gap_ref2_mm[2])
	    || !readRequiredParamInt("NMPC_SP_COUNT", &parsed_setpoint_count)) {
		return false;
	}

	if (use_absolute_position != 0 && use_absolute_position != 1) {
		PX4_ERR("invalid NMPC_ABS_POS=%ld", (long)use_absolute_position);
		return false;
	}

	_use_absolute_position = use_absolute_position == 1;

	const float gap_x = 0.5f * (gap_ref1_mm[0] + gap_ref2_mm[0]) / 1000.0f;
	const float gap_y = 0.5f * (gap_ref1_mm[1] + gap_ref2_mm[1]) / 1000.0f;
	const float gap_z = 0.5f * (gap_ref1_mm[2] + gap_ref2_mm[2]) / 1000.0f;

	if (parsed_setpoint_count <= 0 || parsed_setpoint_count > static_cast<int32_t>(MAX_COLLISION_SETPOINTS)) {
		PX4_ERR("invalid NMPC_SP_COUNT=%ld", (long)parsed_setpoint_count);
		return false;
	}

	for (int32_t i = 0; i < parsed_setpoint_count; i++) {
		char name[80];
		NmpcCollisionSetpoint setpoint {};
		int32_t has_time = -1;
		int32_t has_x_limit = -1;
		int32_t control_mode = -1;
		int32_t nmpc_mode = -1;

		snprintf(name, sizeof(name), "NMPC_S%ld_PX", (long)i);
		if (!readRequiredParamFloat(name, &setpoint.pos_rel_enu[0])) { return false; }
		snprintf(name, sizeof(name), "NMPC_S%ld_PY", (long)i);
		if (!readRequiredParamFloat(name, &setpoint.pos_rel_enu[1])) { return false; }
		snprintf(name, sizeof(name), "NMPC_S%ld_PZ", (long)i);
		if (!readRequiredParamFloat(name, &setpoint.pos_rel_enu[2])) { return false; }
		snprintf(name, sizeof(name), "NMPC_S%ld_VX", (long)i);
		if (!readRequiredParamFloat(name, &setpoint.vel_enu[0])) { return false; }
		snprintf(name, sizeof(name), "NMPC_S%ld_VY", (long)i);
		if (!readRequiredParamFloat(name, &setpoint.vel_enu[1])) { return false; }
		snprintf(name, sizeof(name), "NMPC_S%ld_VZ", (long)i);
		if (!readRequiredParamFloat(name, &setpoint.vel_enu[2])) { return false; }
		snprintf(name, sizeof(name), "NMPC_S%ld_HAS_T", (long)i);
		if (!readRequiredParamInt(name, &has_time)) { return false; }
		snprintf(name, sizeof(name), "NMPC_S%ld_T", (long)i);
		if (!readRequiredParamFloat(name, &setpoint.setpoint_time_s)) { return false; }
		snprintf(name, sizeof(name), "NMPC_S%ld_HAS_X", (long)i);
		if (!readRequiredParamInt(name, &has_x_limit)) { return false; }
		snprintf(name, sizeof(name), "NMPC_S%ld_XLIM", (long)i);
		if (!readRequiredParamFloat(name, &setpoint.waypoint_x_limit_rel_enu)) { return false; }
		snprintf(name, sizeof(name), "NMPC_S%ld_CTRL", (long)i);
		if (!readRequiredParamInt(name, &control_mode)) { return false; }
		snprintf(name, sizeof(name), "NMPC_S%ld_MODE", (long)i);
		if (!readRequiredParamInt(name, &nmpc_mode)) { return false; }

		if ((has_time != 0 && has_time != 1) || (has_x_limit != 0 && has_x_limit != 1)
		    || control_mode < 0 || control_mode > PX4_POSITION_CONTROL
		    || nmpc_mode < 0 || nmpc_mode > NMPC_RECOVERY_MODE) {
			PX4_ERR("invalid NMPC setpoint mode at index %ld", (long)i);
			return false;
		}

		if (!has_time) {
			setpoint.setpoint_time_s = NAN;
		}

		if (!has_x_limit) {
			setpoint.waypoint_x_limit_rel_enu = NAN;
		}

		setpoint.control_mode = (uint8_t)control_mode;
		setpoint.nmpc_mode = (NmpcFlightMode)nmpc_mode;

		if (_use_absolute_position) {
			setpoint.pos_rel_enu[0] += gap_x;
			setpoint.pos_rel_enu[1] += gap_y;

			if (PX4_ISFINITE(setpoint.waypoint_x_limit_rel_enu)) {
				setpoint.waypoint_x_limit_rel_enu += gap_x;
			}
		}

		_collision_setpoints[i] = setpoint;
	}

	_collision_setpoint_count = parsed_setpoint_count;

	PX4_INFO("loaded %u NMPC collision setpoints from PX4 params gap=[%.3f %.3f %.3f] absolute=%d",
		 (unsigned)_collision_setpoint_count,
		 (double)gap_x, (double)gap_y, (double)gap_z, (int)_use_absolute_position);
	return true;
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

	if (!loadNmpcControllerConfig()) {
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
// Trajectory setpoint sequence — ENU (NMPC) frame throughout.
// ----------------------------------------------------------------------------

NmpcSetpointPair MulticopterNmpcControl::get_setpoint_sequence(const Vector3f &initial_pos_enu,
							       const Vector3f &current_pos_enu,
							       hrt_abstime t_now)
{
	NmpcSetpointPair refs{};

	if (_collision_setpoint_start == 0) {
		_collision_setpoint_start = t_now;
	}

	const float current_x_enu = _use_absolute_position ? current_pos_enu(0) : current_pos_enu(0) - initial_pos_enu(0);
	const float prev_x_enu = PX4_ISFINITE(_collision_prev_rel_x_enu) ? _collision_prev_rel_x_enu : current_x_enu;

	while (_collision_setpoint_index + 1 < _collision_setpoint_count) {
		const NmpcCollisionSetpoint &cfg = _collision_setpoints[_collision_setpoint_index];
		const float elapsed_s = (t_now > _collision_setpoint_start) ? (float)(t_now - _collision_setpoint_start) * 1e-6f : 0.0f;
		const bool time_elapsed = PX4_ISFINITE(cfg.setpoint_time_s) && elapsed_s >= cfg.setpoint_time_s;
		const bool x_crossed = PX4_ISFINITE(cfg.waypoint_x_limit_rel_enu)
				       && prev_x_enu < cfg.waypoint_x_limit_rel_enu
				       && current_x_enu >= cfg.waypoint_x_limit_rel_enu;

		if (!time_elapsed && !x_crossed) {
			break;
		}

		++_collision_setpoint_index;
		_collision_setpoint_start = time_elapsed
					    ? _collision_setpoint_start + (hrt_abstime)(cfg.setpoint_time_s * 1e6f)
					    : t_now;
	}

	const NmpcCollisionSetpoint *active_cfg = &_collision_setpoints[_collision_setpoint_index];
	_collision_prev_rel_x_enu = current_x_enu;

	if (_use_absolute_position) {
		(void)initial_pos_enu;
		refs.nominal.pos[0] = active_cfg->pos_rel_enu[0];
		refs.nominal.pos[1] = active_cfg->pos_rel_enu[1];
		refs.nominal.pos[2] = active_cfg->pos_rel_enu[2];
	} else {
		refs.nominal.pos[0] = initial_pos_enu(0) + active_cfg->pos_rel_enu[0];
		refs.nominal.pos[1] = initial_pos_enu(1) + active_cfg->pos_rel_enu[1];
		refs.nominal.pos[2] = initial_pos_enu(2) + active_cfg->pos_rel_enu[2];
	}
	refs.nominal.vel[0] = active_cfg->vel_enu[0];
	refs.nominal.vel[1] = active_cfg->vel_enu[1];
	refs.nominal.vel[2] = active_cfg->vel_enu[2];
	refs.nominal.control_mode = sanitizeTrajectoryControlMode(active_cfg->control_mode);
	refs.nominal.nmpc_mode = active_cfg->nmpc_mode;
	refs.solver = refs.nominal;

	if (active_cfg->nmpc_mode == NMPC_RECOVERY_MODE) {
		refs.solver.pos[0] = current_pos_enu(0);
		refs.solver.pos[1] = current_pos_enu(1);
		refs.solver.vel[0] = 0.0f;
		refs.solver.vel[1] = 0.0f;
		refs.solver.vel[2] = 0.0f;
	}

	return refs;
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

int MulticopterNmpcControl::mapEscReportToMotorIndex(const esc_report_s &report) const
{
	if (report.actuator_function >= actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1
	    && report.actuator_function < actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 + NU) {
		return report.actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;
	}

	if (report.actuator_function == 0 && report.esc_address >= 1 && report.esc_address <= NU) {
		return report.esc_address - 1;
	}

	return -1;
}

void MulticopterNmpcControl::updateEscTelemetryCache()
{
	esc_status_s esc_status{};

	while (_esc_status_sub.update(&esc_status)) {
		const uint8_t esc_count = math::min(esc_status.esc_count, esc_status_s::CONNECTED_ESC_MAX);

		for (uint8_t esc_index = 0; esc_index < esc_count; esc_index++) {
			const esc_report_s &report = esc_status.esc[esc_index];
			const int motor_index = mapEscReportToMotorIndex(report);

			if (motor_index < 0 || motor_index >= NU || report.timestamp == 0) {
				continue;
			}

			const double measured_rps = (double)report.esc_rpm / 60.0;

			if (!PX4_ISFINITE((float)measured_rps)) {
				continue;
			}

			_motor_rps_meas[motor_index] = measured_rps;
			_motor_rps_timestamp_us[motor_index] = report.timestamp;
		}
	}
}

bool MulticopterNmpcControl::pack_state(state_packet_t *pkt)
{
	const hrt_abstime now = hrt_absolute_time();

	if (_position_velocity_timestamp_us == 0 || _attitude_timestamp_us == 0 || _angular_velocity_timestamp_us == 0) {
		if (_last_state_error_report_us == 0 || now - _last_state_error_report_us > 1_s) {
			PX4_ERR("state snapshot incomplete pos=%llu att=%llu ang=%llu",
				(unsigned long long)_position_velocity_timestamp_us,
				(unsigned long long)_attitude_timestamp_us,
				(unsigned long long)_angular_velocity_timestamp_us);
			_last_state_error_report_us = now;
		}

		return false;
	}

	if (_angular_velocity_timestamp_us != _last_run) {
		if (_last_state_error_report_us == 0 || now - _last_state_error_report_us > 1_s) {
			PX4_ERR("angular velocity timestamp mismatch run=%llu ang=%llu",
				(unsigned long long)_last_run,
				(unsigned long long)_angular_velocity_timestamp_us);
			_last_state_error_report_us = now;
		}

		return false;
	}

	if (_position_velocity_timestamp_us > _last_run || _attitude_timestamp_us > _last_run) {
		if (_last_state_error_report_us == 0 || now - _last_state_error_report_us > 1_s) {
			PX4_ERR("state timestamp ahead of packet pos=%llu att=%llu pkt=%llu",
				(unsigned long long)_position_velocity_timestamp_us,
				(unsigned long long)_attitude_timestamp_us,
				(unsigned long long)_last_run);
			_last_state_error_report_us = now;
		}

		return false;
	}

	pkt->seq = _seq++;
	pkt->flags = 0;
	pkt->sample_timestamp_us = _last_run;
	pkt->position_velocity_timestamp_us = _position_velocity_timestamp_us;
	pkt->attitude_timestamp_us = _attitude_timestamp_us;
	pkt->angular_velocity_timestamp_us = _angular_velocity_timestamp_us;

	if (_need_reinit) {
		pkt->flags |= FLAG_REINIT;
	}

	if (_active_nmpc_mode == NMPC_RECOVERY_MODE) {
		pkt->flags |= FLAG_RECOVERY;
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

	pkt->p[55] = NMPC_DIST_FORCE_BX;
	pkt->p[56] = NMPC_DIST_FORCE_BY;
	pkt->p[57] = NMPC_DIST_FORCE_BZ;
	pkt->p[58] = NMPC_DIST_TORQUE_BX;
	pkt->p[59] = NMPC_DIST_TORQUE_BY;
	pkt->p[60] = NMPC_DIST_TORQUE_BZ;

	pkt->p[61] = _has_valid_control ? (double)NMPC_KF1 * (double)_latest_control.u[0] * (double)_latest_control.u[0] : NMPC_HOVER_FORCE_PER_MOTOR;
	pkt->p[62] = _has_valid_control ? (double)NMPC_KF2 * (double)_latest_control.u[1] * (double)_latest_control.u[1] : NMPC_HOVER_FORCE_PER_MOTOR;
	pkt->p[63] = _has_valid_control ? (double)NMPC_KF3 * (double)_latest_control.u[2] * (double)_latest_control.u[2] : NMPC_HOVER_FORCE_PER_MOTOR;
	pkt->p[64] = _has_valid_control ? (double)NMPC_KF4 * (double)_latest_control.u[3] * (double)_latest_control.u[3] : NMPC_HOVER_FORCE_PER_MOTOR;

	pkt->motor_rps_valid_mask = 0;
	pkt->reserved0 = 0;

	for (int motor_index = 0; motor_index < NU; motor_index++) {
		pkt->motor_rps_meas[motor_index] = 0.0;
		pkt->motor_rps_timestamp_us[motor_index] = 0;

		const uint64_t measurement_timestamp_us = _motor_rps_timestamp_us[motor_index];
		const double measured_rps = _motor_rps_meas[motor_index];

		if (measurement_timestamp_us == 0 || !PX4_ISFINITE((float)measured_rps) || measurement_timestamp_us > _last_run) {
			continue;
		}

		// voxl_esc reports one ESC per actuator cycle in round-robin, so each motor
		// needs its own timestamp and a slightly relaxed freshness check.
		if (_last_run - measurement_timestamp_us > MOTOR_RPS_MEAS_TIMEOUT_US) {
			continue;
		}

		pkt->motor_rps_meas[motor_index] = measured_rps;
		pkt->motor_rps_timestamp_us[motor_index] = measurement_timestamp_us;
		pkt->motor_rps_valid_mask |= (1u << motor_index);
	}

	return true;
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

void MulticopterNmpcControl::publish_offboard_control_mode(bool use_px4_position_control)
{
	offboard_control_mode_s ocm{};
	ocm.position = use_px4_position_control;
	ocm.velocity = use_px4_position_control;
	ocm.acceleration = false;
	ocm.attitude = false;
	ocm.body_rate = false;
	ocm.actuator = !use_px4_position_control;
	ocm.timestamp = hrt_absolute_time();
	_offboard_control_mode_pub.publish(ocm);
}

void MulticopterNmpcControl::publish_trajectory_setpoint(const trajectory_setpoint_s &traj_sp)
{
	_trajectory_setpoint_pub.publish(traj_sp);
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
		_angular_velocity_timestamp_us = vehicle_angular_velocity.timestamp_sample;
		_angular_velocity = Vector3f(vehicle_angular_velocity.xyz);

		if (_vehicle_attitude_sub.updated()) {
			vehicle_attitude_s vehicle_attitude;
			if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
				_attitude = Quatf(vehicle_attitude.q);
				_attitude_timestamp_us = vehicle_attitude.timestamp_sample;
			}
		}

		vehicle_local_position_s vehicle_local_position;
		if (_vehicle_local_position_sub.update(&vehicle_local_position)) {
			_position = Vector3f(vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z);
			_velocity = Vector3f(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
			_position_velocity_timestamp_us = vehicle_local_position.timestamp_sample;
		}

		updateEscTelemetryCache();

		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_offboard_enabled = _vehicle_control_mode.flag_control_offboard_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!previous_offboard_enabled && _vehicle_control_mode.flag_control_offboard_enabled) {
					_initial_position = _position;
					_min_valid_control_seq = _seq;
					_collision_setpoint_index = 0;
					_collision_setpoint_start = 0;
					_collision_prev_rel_x_enu = NAN;
					_need_reinit = true;
					_has_valid_control = false;
					_using_px4_position_control = false;
					_active_nmpc_mode = NMPC_FLIGHT_MODE;
					_px4_position_control_yaw = NAN;
				} else if (previous_offboard_enabled && !_vehicle_control_mode.flag_control_offboard_enabled) {
					generateFailsafeTrajectory(_trajectory_setpoint, _position, _attitude);
					_min_valid_control_seq = _seq;
					_collision_setpoint_index = 0;
					_collision_setpoint_start = 0;
					_collision_prev_rel_x_enu = NAN;
					_need_reinit = true;
					_has_valid_control = false;
					_using_px4_position_control = false;
					_active_nmpc_mode = NMPC_FLIGHT_MODE;
					_px4_position_control_yaw = NAN;
				}
			}
		}

		publish_offboard_control_mode(_using_px4_position_control);

		if (_vehicle_control_mode.flag_control_offboard_enabled) {
			// NED->ENU: ENU_x=NED_y (East), ENU_y=NED_x (North), ENU_z=-NED_z (Up)
			const Vector3f initial_pos_enu(
				_initial_position(1),   // East  = NED_y
				_initial_position(0),   // North = NED_x
				-_initial_position(2)   // Up    = -NED_z
			);
			const Vector3f current_pos_enu(
				_position(1),   // East  = NED_y
				_position(0),   // North = NED_x
				-_position(2)   // Up    = -NED_z
			);

			const NmpcSetpointPair refs = get_setpoint_sequence(initial_pos_enu, current_pos_enu, _last_run);
			const bool use_px4_position_control = refs.solver.control_mode == PX4_POSITION_CONTROL;
			_active_nmpc_mode = refs.solver.nmpc_mode;

			if (use_px4_position_control != _using_px4_position_control) {
				_using_px4_position_control = use_px4_position_control;

				if (_using_px4_position_control) {
					_px4_position_control_yaw = Eulerf(_attitude).psi();
				} else {
					_need_reinit = true;
					_has_valid_control = false;
					_min_valid_control_seq = _seq;
					_px4_position_control_yaw = NAN;
				}
			}

			publish_offboard_control_mode(_using_px4_position_control);

			// Convert ENU setpoint back to NED for _trajectory_setpoint.
			// ENU->NED: NED_x=ENU_y (North), NED_y=ENU_x (East), NED_z=-ENU_z (Down)
			_trajectory_setpoint.position[0] = refs.solver.pos[1];   // North = ENU_y
			_trajectory_setpoint.position[1] = refs.solver.pos[0];   // East  = ENU_x
			_trajectory_setpoint.position[2] = -refs.solver.pos[2];  // Down  = -ENU_z
			_trajectory_setpoint.velocity[0] = refs.solver.vel[1];   // North = ENU_y
			_trajectory_setpoint.velocity[1] = refs.solver.vel[0];   // East  = ENU_x
			_trajectory_setpoint.velocity[2] = -refs.solver.vel[2];  // Down  = -ENU_z
			_trajectory_setpoint.acceleration[0] = 0.0f;
			_trajectory_setpoint.acceleration[1] = 0.0f;
			_trajectory_setpoint.acceleration[2] = 0.0f;
			_trajectory_setpoint.yaw = 0.0f;
			_trajectory_setpoint.yawspeed = 0.0f;
			_trajectory_setpoint.timestamp = _last_run;

			if (_using_px4_position_control) {
				trajectory_setpoint_s px4_trajectory_setpoint = _trajectory_setpoint;
				px4_trajectory_setpoint.yaw = _px4_position_control_yaw;
				px4_trajectory_setpoint.yawspeed = 0.0f;
				px4_trajectory_setpoint.timestamp = hrt_absolute_time();
				publish_trajectory_setpoint(px4_trajectory_setpoint);
			}

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

			state_packet_t pkt_state{};
			if (!pack_state(&pkt_state)) {
				_need_reinit = true;
				_has_valid_control = false;
				perf_end(_loop_perf);
				return;
			}

			if (_has_valid_control && !_using_px4_position_control) {
				publish_actuator_motors(&_latest_control);
			}

			nmpc_state_data_s state_msg{};
			state_msg.timestamp = hrt_absolute_time();
			state_msg.seq = pkt_state.seq;
			state_msg.flags = pkt_state.flags;
			state_msg.sample_timestamp_us = pkt_state.sample_timestamp_us;
			state_msg.position_velocity_timestamp_us = pkt_state.position_velocity_timestamp_us;
			state_msg.attitude_timestamp_us = pkt_state.attitude_timestamp_us;
			state_msg.angular_velocity_timestamp_us = pkt_state.angular_velocity_timestamp_us;
			memcpy(state_msg.rigid_body_state, pkt_state.rigid_body_state, sizeof(pkt_state.rigid_body_state));
			memcpy(state_msg.p, pkt_state.p, sizeof(pkt_state.p));
			memcpy(state_msg.motor_rps_meas, pkt_state.motor_rps_meas, sizeof(pkt_state.motor_rps_meas));
			memcpy(state_msg.motor_rps_timestamp_us, pkt_state.motor_rps_timestamp_us, sizeof(pkt_state.motor_rps_timestamp_us));
			state_msg.motor_rps_valid_mask = pkt_state.motor_rps_valid_mask;
			state_msg.reserved0 = pkt_state.reserved0;
			memcpy(state_msg.nominal_position_enu, refs.nominal.pos, sizeof(refs.nominal.pos));
			memcpy(state_msg.nominal_velocity_enu, refs.nominal.vel, sizeof(refs.nominal.vel));
			memcpy(state_msg.solver_position_enu, refs.solver.pos, sizeof(refs.solver.pos));
			memcpy(state_msg.solver_velocity_enu, refs.solver.vel, sizeof(refs.solver.vel));
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
