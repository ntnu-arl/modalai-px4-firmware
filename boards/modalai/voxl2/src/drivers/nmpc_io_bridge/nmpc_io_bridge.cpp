/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include <fcntl.h>
#include <semaphore.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <limits.h>
#include <errno.h>

#include <px4_log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/topics/nmpc_state_data.h>
#include <uORB/topics/nmpc_control_data.h>

#include "modal_pipe_sink.h"
#include "protocol.h"

// Solver creates a sink at STATE_SINK_PATH; bridge opens this FIFO for writing
#define STATE_SINK_PATH  (MODAL_PIPE_DEFAULT_BASE_DIR "nmpc_state")
// Bridge creates a sink at CONTROL_SINK_PATH; solver opens this FIFO for writing
#define CONTROL_SINK_PATH (MODAL_PIPE_DEFAULT_BASE_DIR "nmpc_control")

#define PIPE_SIZE    (64*1024)
#define READ_BUF_LEN (sizeof(control_packet_t) * 4)
#define CONTROL_SINK_CH 1
#define CONTROL_PACKET_LEN ((int)sizeof(control_packet_t))

// _mkdir_recursive is provided by modal_io_bridge for this board build.
extern "C" int _mkdir_recursive(const char *dir);

extern "C" { __EXPORT int nmpc_io_bridge_main(int argc, char *argv[]); }

namespace nmpc_io_bridge
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
static_assert(sizeof(((nmpc_state_data_s *)nullptr)->cost_weight_set)
		      == sizeof(((state_packet_t *)nullptr)->cost_weight_set),
	      "nmpc_state_data.cost_weight_set size must match state_packet_t.cost_weight_set");
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

bool _initialized = false;
bool _is_running = false;
bool _debug = false;

static px4_task_t _task_handle = -1;

// Shared buffer for control data from sink callback -> main task
static control_packet_t _ctrl_buf;
static px4_sem_t _ctrl_sem;
static bool _ctrl_valid = false;
static uint8_t _ctrl_rx_buf[READ_BUF_LEN + sizeof(control_packet_t)];
static int _ctrl_rx_buf_len = 0;

uORB::Subscription _nmpc_state_sub{ORB_ID(nmpc_state_data)};
uORB::Publication<nmpc_control_data_s> _nmpc_control_pub{ORB_ID(nmpc_control_data)};

static void control_sink_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
    if (bytes <= 0) {
        if (_debug) PX4_WARN("control_sink_cb: read returned %d", bytes);
        return;
    }

    if (_ctrl_rx_buf_len + bytes > (int)sizeof(_ctrl_rx_buf)) {
        PX4_ERR("control_sink_cb overflow: %d buffered + %d new > %zu", _ctrl_rx_buf_len, bytes, sizeof(_ctrl_rx_buf));
        _ctrl_rx_buf_len = 0;
        return;
    }

    memcpy(&_ctrl_rx_buf[_ctrl_rx_buf_len], data, bytes);
    _ctrl_rx_buf_len += bytes;

    const int packet_count = _ctrl_rx_buf_len / CONTROL_PACKET_LEN;

    if (packet_count <= 0) {
        return;
    }

    memcpy(&_ctrl_buf, &_ctrl_rx_buf[(packet_count - 1) * CONTROL_PACKET_LEN], CONTROL_PACKET_LEN);
    _ctrl_valid = true;

    const int consumed = packet_count * CONTROL_PACKET_LEN;
    const int remaining = _ctrl_rx_buf_len - consumed;

    if (remaining > 0) {
        memmove(_ctrl_rx_buf, &_ctrl_rx_buf[consumed], remaining);
    }

    _ctrl_rx_buf_len = remaining;
    px4_sem_post(&_ctrl_sem);
}

int initialize()
{
    if (_initialized) return 0;

    px4_sem_init(&_ctrl_sem, 0, 0);

    if (pipe_sink_create(CONTROL_SINK_CH, CONTROL_SINK_PATH, SINK_FLAG_EN_SIMPLE_HELPER, PIPE_SIZE, READ_BUF_LEN)) {
        PX4_ERR("failed to create control sink at %s", CONTROL_SINK_PATH);
        return -1;
    }
    pipe_sink_set_simple_cb(CONTROL_SINK_CH, &control_sink_cb, NULL);

    _initialized = true;
    return 0;
}

void nmpc_io_bridge_task()
{
    _is_running = true;
    PX4_INFO("NMPC IO Bridge starting");

    int state_fd = -1;

    while (true) {
        // Reconnect state FIFO if needed (solver may not be running yet)
        if (state_fd < 0) {
            state_fd = open(STATE_SINK_PATH, O_WRONLY | O_NONBLOCK);
            if (state_fd < 0 && _debug) {
                PX4_WARN("state FIFO not available: %s", STATE_SINK_PATH);
            }
        }

        // Forward new state from uORB -> state FIFO
        nmpc_state_data_s state_msg;
        if (_nmpc_state_sub.update(&state_msg)) {
            if (state_fd >= 0) {
                state_packet_t pkt;
                pkt.seq    = state_msg.seq;
                pkt.flags  = state_msg.flags;
                pkt.pad[0] = pkt.pad[1] = pkt.pad[2] = 0;
                pkt.sample_timestamp_us = state_msg.sample_timestamp_us;
                pkt.position_velocity_timestamp_us = state_msg.position_velocity_timestamp_us;
                pkt.attitude_timestamp_us = state_msg.attitude_timestamp_us;
                pkt.angular_velocity_timestamp_us = state_msg.angular_velocity_timestamp_us;
                memcpy(pkt.rigid_body_state, state_msg.rigid_body_state, sizeof(pkt.rigid_body_state));
                memcpy(pkt.p,  state_msg.p,  sizeof(pkt.p));
                memcpy(pkt.motor_rps_meas, state_msg.motor_rps_meas, sizeof(pkt.motor_rps_meas));
                memcpy(pkt.motor_rps_timestamp_us, state_msg.motor_rps_timestamp_us, sizeof(pkt.motor_rps_timestamp_us));
                pkt.motor_rps_valid_mask = state_msg.motor_rps_valid_mask;
                pkt.cost_weight_set = state_msg.cost_weight_set;
                memcpy(pkt.nominal_position_enu, state_msg.nominal_position_enu, sizeof(pkt.nominal_position_enu));
                memcpy(pkt.nominal_velocity_enu, state_msg.nominal_velocity_enu, sizeof(pkt.nominal_velocity_enu));
                int ret = write(state_fd, &pkt, sizeof(pkt));
                if (ret < 0) {
                    if (_debug) PX4_WARN("state FIFO write failed, will retry");
                    close(state_fd);
                    state_fd = -1;
                }
            }
        }

        // Forward control from sink -> uORB (non-blocking check)
        bool have_control = false;

        while (px4_sem_trywait(&_ctrl_sem) == 0) {
            have_control = true;
        }

        if (have_control && _ctrl_valid) {
            nmpc_control_data_s ctrl_msg{};
            ctrl_msg.timestamp    = hrt_absolute_time();
            ctrl_msg.seq          = _ctrl_buf.seq;
            ctrl_msg.status       = _ctrl_buf.status;
            memcpy(ctrl_msg.u,         _ctrl_buf.u,         sizeof(_ctrl_buf.u));
            ctrl_msg.solve_time_us = _ctrl_buf.solve_time_us;
            memcpy(ctrl_msg.quat_next, _ctrl_buf.quat_next, sizeof(_ctrl_buf.quat_next));
            _nmpc_control_pub.publish(ctrl_msg);
            _ctrl_valid = false;
        }

        usleep(200);  // ~5kHz poll
    }

    if (state_fd >= 0) close(state_fd);
}

int start(int argc, char *argv[])
{
    int ch;
    int myoptind = 1;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "d", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'd':
            _debug = true;
            break;
        default:
            break;
        }
    }

    if (!_initialized) {
        if (initialize()) return -1;
    }

    if (_is_running) {
        PX4_WARN("Already started");
        return 0;
    }

    _task_handle = px4_task_spawn_cmd("nmpc_io_bridge",
                                      SCHED_DEFAULT,
                                      SCHED_PRIORITY_DEFAULT,
                                      2000,
                                      (px4_main_t) &nmpc_io_bridge_task,
                                      (char *const *)argv);
    if (_task_handle < 0) {
        PX4_ERR("task start failed");
        return -1;
    }
    return 0;
}

void usage()
{
    PX4_INFO("Usage: nmpc_io_bridge start [options]");
    PX4_INFO("Options: -d    enable debug output");
}

} // namespace nmpc_io_bridge

int nmpc_io_bridge_main(int argc, char *argv[])
{
    int myoptind = 1;
    if (argc <= 1) {
        nmpc_io_bridge::usage();
        return -1;
    }
    const char *verb = argv[myoptind];
    if (!strcmp(verb, "start")) {
        return nmpc_io_bridge::start(argc - 1, argv + 1);
    }
    nmpc_io_bridge::usage();
    return -1;
}
