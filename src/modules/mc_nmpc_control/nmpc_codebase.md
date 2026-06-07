# NMPC Codebase Notes

This note documents the current `mc_nmpc_control` architecture and how it interacts with the VOXL-side solver code in the separate repository:

- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code`

It is limited to the current runtime path used by this PX4 module.

## Purpose

`mc_nmpc_control` does not link acados directly.

Its job is to:

1. Read PX4 vehicle state and ESC telemetry.
2. Pack the solver input into the shared NMPC packet format.
3. Publish that data on uORB as `nmpc_state_data`.
4. Receive solver output on uORB as `nmpc_control_data`.
5. Convert valid solver output into `actuator_motors`.
6. Optionally hand the final segment over to PX4 position control by publishing `trajectory_setpoint` and changing `offboard_control_mode`.

The actual userspace solver runs outside PX4 on the VOXL apps processor.

## Main Files In This Directory

- `MulticopterNmpcControl.cpp`
- `MulticopterNmpcControl.hpp`
- `protocol.h`
- `mc_nmpc_control_params.c`
- `CMakeLists.txt`
- `Kconfig`

## External Files This Module Depends On

### In the other repository

- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/voxl_src/protocol.h`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/voxl_src/generate_nmpc_solver.py`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/voxl_src/nmpc_pipe_solver.c`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/voxl_src/Makefile.voxl`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/src/nmpc_controller.py`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/c_generated_code/`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/c_generated_code_recovery/`

### Elsewhere in this PX4 repo

- `msg/NmpcStateData.msg`
- `msg/NmpcControlData.msg`
- `boards/modalai/voxl2/src/drivers/nmpc_io_bridge/nmpc_io_bridge.cpp`
- `boards/modalai/voxl2/target/voxl-px4-start`

## Runtime Architecture

The current real-hardware runtime chain is:

1. `mc_nmpc_control` samples PX4 state and builds a packed NMPC state packet.
2. `mc_nmpc_control` publishes `nmpc_state_data`.
3. `nmpc_io_bridge` forwards `nmpc_state_data` to `/run/mpa/nmpc_state`.
4. `/usr/local/bin/nmpc_pipe_solver` reads the FIFO, runs the generated acados solver, and writes `control_packet_t` to `/run/mpa/nmpc_control`.
5. `nmpc_io_bridge` republishes that output as `nmpc_control_data`.
6. `mc_nmpc_control` consumes `nmpc_control_data` and either:
   - publishes `actuator_motors` directly, or
   - publishes `trajectory_setpoint` and enables PX4 position control for the configured segment.

This module is therefore a PX4-side bridge, not the solver itself.

## Shared Packet ABI

The shared packed ABI is defined locally in `protocol.h` and must stay compatible with:

- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/voxl_src/protocol.h`

Current constants:

- `NX = 17`
- `NU = 4`
- `NRIGID = 13`
- `NP = 65`
- `N_HORIZON = 20`

Current flags:

- `FLAG_REINIT = 0x01`
- `FLAG_RECOVERY = 0x02`

### `state_packet_t`

- `seq`
- `flags`
- `sample_timestamp_us`
- `position_velocity_timestamp_us`
- `attitude_timestamp_us`
- `angular_velocity_timestamp_us`
- `rigid_body_state[13]`
- `p[65]`
- `motor_rps_meas[4]`
- `motor_rps_timestamp_us[4]`
- `motor_rps_valid_mask`
- `reserved0`
- `nominal_position_enu[3]`
- `nominal_velocity_enu[3]`

### `control_packet_t`

- `seq`
- `status`
- `u[4]`
- `solve_time_us`
- `quat_next[4]`

`MulticopterNmpcControl.cpp` uses `static_assert`s to verify that the `nmpc_state_data` and `nmpc_control_data` uORB message layouts remain compatible with these packet structs.

## What `mc_nmpc_control` Publishes And Subscribes

### Subscriptions

- `vehicle_local_position`
- `vehicle_attitude`
- `vehicle_angular_velocity`
- `vehicle_control_mode`
- `vehicle_status`
- `esc_status`
- `nmpc_control_data`

### Publications

- `offboard_control_mode`
- `actuator_motors`
- `trajectory_setpoint`
- `nmpc_state_data`

## State Packing Details

`MulticopterNmpcControl::pack_state()` is the key packing function.

### Measured rigid-body state

`rigid_body_state[0:13]` is built from PX4 state after frame conversion:

- position: PX4 NED -> solver ENU
- linear velocity: PX4 NED -> solver ENU
- attitude quaternion: PX4 FRD -> solver FLU convention
- body angular velocity: PX4 FRD -> solver FLU

### Parameter vector `p[65]`

The parameter vector currently contains:

- `p[0]`: mass
- `p[1:7]`: inertia terms
- `p[7:10]`: gravity vector in solver frame
- `p[10:13]`: position setpoint in solver frame
- `p[13:16]`: velocity setpoint in solver frame
- `p[16:20]`: quaternion setpoint
- `p[20:44]`: allocation matrix
- `p[44:48]`: thrust coefficients
- `p[48:52]`: motor time constants
- `p[52:55]`: COM offset
- `p[55:58]`: fixed body-force disturbance
- `p[58:61]`: fixed body-torque disturbance
- `p[61:65]`: previous motor forces

### ESC telemetry side channel

Fresh ESC RPM telemetry is forwarded separately through:

- `motor_rps_meas[4]`
- `motor_rps_timestamp_us[4]`
- `motor_rps_valid_mask`

This allows the userspace solver to correct its motor-state estimate with asynchronous ESC updates.

### Reference metadata

`nmpc_state_data` carries more than the raw packet:

- `nominal_position_enu`
- `nominal_velocity_enu`
- `solver_position_enu`
- `solver_velocity_enu`

The nominal reference is the intended task-level reference. The solver reference can differ during recovery mode.

## NMPC Modes And Handoff Behavior

This module currently supports two NMPC modes:

- nominal flight mode
- recovery mode

The active mode is chosen from the PX4 `NMPC_S*_MODE` parameters and encoded in the outgoing packet via `FLAG_RECOVERY`.

This module also supports two control styles per setpoint segment:

- direct NMPC actuator control
- PX4 position-control handoff

The control style is chosen from `NMPC_S*_CTRL`.

Behavior:

- In direct actuator mode, valid solver output is converted to `actuator_motors`.
- In PX4 position-control mode, `offboard_control_mode.position` and `.velocity` are enabled, `offboard_control_mode.actuator` is disabled, and a `trajectory_setpoint` is published instead.

When switching back from PX4 position control to direct actuator control, the solver is marked for reinitialization.

## PX4 Parameters Used By This Module

### Standard PX4 actuator mapping parameters

- `THR_MDL_FAC`
- `VOXL_ESC_RPM_MIN`
- `VOXL_ESC_RPM_MAX`

These are used to map solver-returned motor RPS back to PX4 normalized `actuator_motors.control`.

### NMPC sequence parameters

Defined in `mc_nmpc_control_params.c`:

- `NMPC_ABS_POS`
- `NMPC_G1_XMM`, `NMPC_G1_YMM`, `NMPC_G1_ZMM`
- `NMPC_G2_XMM`, `NMPC_G2_YMM`, `NMPC_G2_ZMM`
- `NMPC_SP_COUNT`
- `NMPC_S0_*`
- `NMPC_S1_*`
- `NMPC_S2_*`
- `NMPC_S3_*`

These parameters describe up to four collision/setpoint segments, including:

- ENU setpoint position and velocity
- optional time-based transition
- optional x-limit-based transition
- control mode
- NMPC nominal/recovery mode

## Where The Solver Actually Comes From

The solver logic used by `nmpc_pipe_solver` is generated from the Python OCP definition in:

- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/src/nmpc_controller.py`

The current generator script is:

- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/voxl_src/generate_nmpc_solver.py`

That generator currently emits:

- nominal solver code under `c_generated_code/`
- recovery solver code under `c_generated_code_recovery/`
- `acados_ocp_quadrotor.json`
- `acados_ocp_quadrotor_recovery.json`

This PX4 module never calls acados directly. It only packs data for the external solver process and consumes the returned control packet.

## Important Synchronization Requirements

When changing this module, keep these contracts aligned with the other repo:

1. `protocol.h` layout must match `voxl_src/protocol.h`.
2. Packet dimensions and parameter indexing must match the generated solver expectations.
3. Any change to packed state fields must be reflected in the VOXL-side solver and analysis scripts.
4. Any change to `MulticopterNmpcControl.cpp` constants can affect:
   - `voxl_src/extract_px4_motor_log.py`
   - `voxl_src/plot_hitl.py`
   - `voxl_src/plot_px4_handoff_control_errors.py`
   - `voxl_src/real_flight_replica/replica.py`
5. Any change to the NMPC sequence parameter schema must stay consistent with the deployment scripts that write those PX4 params.
