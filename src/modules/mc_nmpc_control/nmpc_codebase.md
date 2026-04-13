# NMPC Codebase Notes

## Purpose

This note explains where the NMPC solver code used by `mc_nmpc_control` actually comes from, and how data flows from PX4 state to the acados solver and back to motor commands.

The short version is:

- `mc_nmpc_control` does not link acados directly.
- It publishes a packed NMPC state over uORB.
- `nmpc_io_bridge` forwards that state to `/run/mpa/nmpc_state`.
- `/usr/local/bin/nmpc_pipe_solver` reads that pipe, calls the generated acados solver, and writes results to `/run/mpa/nmpc_control`.
- `nmpc_io_bridge` republishes the result on uORB.
- `mc_nmpc_control` consumes that result and publishes `actuator_motors`.

## Main Files

### PX4 side

- `src/modules/mc_nmpc_control/MulticopterNmpcControl.cpp`
- `src/modules/mc_nmpc_control/MulticopterNmpcControl.hpp`
- `src/modules/mc_nmpc_control/protocol.h`
- `msg/NmpcStateData.msg`
- `msg/NmpcControlData.msg`
- `boards/modalai/voxl2/src/drivers/nmpc_io_bridge/nmpc_io_bridge.cpp`
- `boards/modalai/voxl2/target/voxl-px4-start`

### Host / VOXL solver side

- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/src/nmpc_controller.py`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/acados_ocp_quadrotor.json`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/c_generated_code/acados_solver_quadrotor_nmpc.c`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/c_generated_code/acados_solver_quadrotor_nmpc.h`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/voxl_src/Makefile.voxl`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/voxl_src/nmpc_pipe_solver.c`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/voxl_src/nmpc_udp_solver.c`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/voxl_src/voxl_nmpc_bridge.py`

## Where the acados solver comes from

The solver-specific acados code is generated from the Python OCP definition in:

- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/src/nmpc_controller.py`

That file defines:

- model name: `quadrotor_nmpc`
- state dimension: 17
- control dimension: 4
- parameter dimension: 55
- horizon: 20
- cost type: `NONLINEAR_LS`
- solver type: `SQP_RTI`
- QP solver: `PARTIAL_CONDENSING_HPIPM`
- integrator: `ERK`

The generated export metadata is stored in:

- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/acados_ocp_quadrotor.json`

That JSON points to the generated code directory:

- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/c_generated_code`

The actual generated solver sources used by the VOXL solver wrapper are:

- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/c_generated_code/acados_solver_quadrotor_nmpc.c`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/c_generated_code/acados_solver_quadrotor_nmpc.h`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/c_generated_code/quadrotor_nmpc_model/...`
- `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/c_generated_code/quadrotor_nmpc_cost/...`

## Important build detail

`voxl_src/Makefile.voxl` builds the onboard solver from `../c_generated_code` and links against acados from:

- `/home/paran/acados`
- `/home/paran/acados/build_aarch64`

So the actual onboard solver build path is:

1. OCP defined in `code/src/nmpc_controller.py`
2. acados generates C into `code/c_generated_code`
3. `voxl_src/nmpc_pipe_solver.c` wraps that generated solver
4. `voxl_src/Makefile.voxl` compiles the wrapper plus generated code into `nmpc_pipe_solver`
5. PX4 startup launches `/usr/local/bin/nmpc_pipe_solver`

Note: `voxl_src/build_voxl.sh` builds the vendored `voxl_src/acados` tree, but `voxl_src/Makefile.voxl` is configured to use `/home/paran/acados`, not `voxl_src/acados`. That means the active build currently depends on the external `/home/paran/acados` installation.

## Runtime process layout

On boot, `boards/modalai/voxl2/target/voxl-px4-start` does two relevant things:

1. starts `/usr/local/bin/nmpc_pipe_solver`
2. starts `nmpc_io_bridge`

This means the runtime architecture is:

- PX4 module on the flight side: `mc_nmpc_control`
- bridge on the VOXL board side: `nmpc_io_bridge`
- userspace solver process on the apps processor: `nmpc_pipe_solver`

## Data structures

The packet ABI shared between PX4 and the solver wrapper is in:

- `src/modules/mc_nmpc_control/protocol.h`

It defines:

- `NX = 17`
- `NU = 4`
- `NP = 55`
- `N_HORIZON = 20`

### `state_packet_t`

- `seq`
- `flags`
- `sample_timestamp_us`
- `rigid_body_state[13]`
- `p[55]`

### `control_packet_t`

- `seq`
- `status`
- `u[4]`
- `solve_time_us`
- `quat_next[4]`

The corresponding uORB topics are:

- `msg/NmpcStateData.msg`
- `msg/NmpcControlData.msg`

## Meaning of `rigid_body_state`, `x0`, and `p`

`MulticopterNmpcControl::pack_state()` packs the solver inputs.

### Measured state `rigid_body_state[13]`

- `rigid_body_state[0:3]`: position, PX4 NED converted to solver ENU
- `rigid_body_state[3:6]`: linear velocity, PX4 NED converted to solver ENU
- `rigid_body_state[6:10]`: quaternion, PX4 FRD converted to solver FLU convention
- `rigid_body_state[10:13]`: body angular velocity, PX4 FRD converted to solver FLU

### Solver state `x0[17]`

- `x0[0:13]`: copied from `rigid_body_state`
- `x0[13:17]`: internal motor-state estimate owned by the solver runtime

### Parameters `p[55]`

- `p[0]`: mass
- `p[1:7]`: inertia terms
- `p[7:10]`: gravity vector
- `p[10:13]`: position setpoint
- `p[13:16]`: velocity setpoint
- `p[16:20]`: quaternion setpoint
- `p[20:44]`: 6x4 allocation matrix, column-major
- `p[44:48]`: thrust coefficients `kf`
- `p[48:52]`: motor time constants `tau`
- `p[52:55]`: COM offset

## Actual data flow

### 1. PX4 collects vehicle state

`mc_nmpc_control` reads:

- `vehicle_local_position`
- `vehicle_attitude`
- `vehicle_angular_velocity`
- `trajectory_setpoint`
- `vehicle_control_mode`

PX4 does not estimate solver motor RPS.

### 2. PX4 packs solver input

`MulticopterNmpcControl::pack_state()` builds a `state_packet_t` from:

- vehicle state
- sample timestamp
- setpoint
- parameters
- allocation matrix
- thrust coefficients
- motor time constants
- COM offset

If offboard just started, or the solver needs resetting, it sets `FLAG_REINIT`.

### 3. PX4 publishes `nmpc_state_data`

`mc_nmpc_control` publishes the packed state on uORB topic:

- `nmpc_state_data`

### 4. `nmpc_io_bridge` forwards state to FIFO

`boards/modalai/voxl2/src/drivers/nmpc_io_bridge/nmpc_io_bridge.cpp` subscribes to `nmpc_state_data`, copies it into `state_packet_t`, and writes it to:

- `/run/mpa/nmpc_state`

### 5. `nmpc_pipe_solver` reads the state packet

`voxl_src/nmpc_pipe_solver.c` blocks on:

- `/run/mpa/nmpc_state`

For each received packet it:

1. resets or propagates its internal motor-state estimate from the packet timestamp and previous commanded RPS
2. assembles full solver state `x0[17]` locally
3. optionally warm-starts on `FLAG_REINIT`
4. fixes the stage-0 state with `lbx = ubx = x0`
5. updates parameter vector `p` for all shooting stages `0..N`
6. calls `quadrotor_nmpc_acados_solve()`

### 6. acados runs the generated solver

The wrapper uses the generated capsule from:

- `acados_solver_quadrotor_nmpc.c`
- `acados_solver_quadrotor_nmpc.h`

This is the actual acados solver used in flight.

The wrapper then extracts:

- `u_opt = u(stage 0)`
- `x1 = x(stage 1)`

### 7. Solver wrapper converts force command to RPS

The generated solver optimizes motor force commands.

`nmpc_pipe_solver.c` converts those forces to motor RPS using:

- `rps = sqrt(force / kf)`

That converted RPS becomes `control_packet_t.u`.

So the meaning of solver output at the PX4 side is:

- `u[4]` = desired motor speed in RPS

not raw force.

### 8. `nmpc_pipe_solver` writes control packet

The wrapper writes `control_packet_t` to:

- `/run/mpa/nmpc_control`

It also includes:

- solver `status`
- `solve_time_us`
- predicted next quaternion `quat_next`

### 9. `nmpc_io_bridge` republishes control to uORB

`nmpc_io_bridge` receives data from `/run/mpa/nmpc_control` and republishes:

- `nmpc_control_data`

### 10. `mc_nmpc_control` consumes the result

`mc_nmpc_control` reads `nmpc_control_data`.

If `status != 0`:

- it marks the solver for reinit
- it does not use the returned command

If `status == 0`:

- it stores the commanded motor RPS
- it publishes `actuator_motors`

### 11. PX4 maps RPS to normalized actuator command

`publish_actuator_motors()` converts motor RPS to:

- RPM
- then PX4 normalized actuator command in `[0, 1]`

using:

- `MC_NMPC_MINRPM`
- `MC_NMPC_MAXRPM`
- `THR_MDL_FAC`

The result is published as:

- `actuator_motors`

## End-to-end flow summary

```text
vehicle state + setpoint + params
    -> mc_nmpc_control
    -> nmpc_state_data (uORB)
    -> nmpc_io_bridge
    -> /run/mpa/nmpc_state
    -> nmpc_pipe_solver
    -> generated acados solver (quadrotor_nmpc)
    -> u_opt(0) in motor force
    -> wrapper converts force to motor RPS
    -> /run/mpa/nmpc_control
    -> nmpc_io_bridge
    -> nmpc_control_data (uORB)
    -> mc_nmpc_control
    -> actuator_motors
```

## Relationship to `voxl_src/voxl_nmpc_bridge.py`

`voxl_src/voxl_nmpc_bridge.py` uses the same packet format, but over TCP for host-side experiments and simulation. It is not the in-flight PX4 pipe path.

That path is:

- host-side Python controller
- TCP socket
- `voxl_src/nmpc_udp_solver.c`

Despite the filename, `nmpc_udp_solver.c` actually runs a TCP server.

## Practical answer to "where does the solver source come from?"

If the question is about the solver logic used by `MulticopterNmpcControl.cpp`, the answer is:

1. model and OCP are defined in `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/src/nmpc_controller.py`
2. acados generates C code into `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/c_generated_code`
3. `/home/paran/Dropbox/NTNU/13_optimizing_soft_drones/code/voxl_src/nmpc_pipe_solver.c` is the runtime wrapper around that generated solver
4. PX4 talks to that wrapper through `nmpc_io_bridge` and the `/run/mpa/nmpc_*` pipes

So the acados solver source ultimately used by `mc_nmpc_control` comes from `code/c_generated_code`, which itself is generated from `code/src/nmpc_controller.py`.
