#ifndef VOXL_NMPC_PROTOCOL_H_
#define VOXL_NMPC_PROTOCOL_H_

#include <stdint.h>

#define NX 17
#define NU 4
#define NRIGID 13
#define NP 65
#define N_HORIZON 20

#define FLAG_REINIT 0x01

enum {
    NMPC_COST_WEIGHT_SET_REGULAR_FLIGHT = 0,
    NMPC_COST_WEIGHT_SET_RECOVERY = 1,
    NMPC_COST_WEIGHT_SET_COUNT = 2,
};

#pragma pack(push, 1)

typedef struct {
    uint32_t seq;
    uint8_t  flags;
    uint8_t  pad[3];
    uint64_t sample_timestamp_us;
    uint64_t position_velocity_timestamp_us;
    uint64_t attitude_timestamp_us;
    uint64_t angular_velocity_timestamp_us;
    double   rigid_body_state[NRIGID];
    double   p[NP];
    double   motor_rps_meas[NU];
    uint64_t motor_rps_timestamp_us[NU];
    uint8_t  motor_rps_valid_mask;
    uint8_t  cost_weight_set;
    float    nominal_position_enu[3];
    float    nominal_velocity_enu[3];
} state_packet_t;

typedef struct {
    uint32_t seq;
    int32_t  status;
    double   u[NU];
    double   solve_time_us;
    double   quat_next[4];
} control_packet_t;

#pragma pack(pop)

#endif
