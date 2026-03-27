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

// _mkdir_recursive is provided by modal_io_bridge for this board build.
extern "C" int _mkdir_recursive(const char *dir);

extern "C" { __EXPORT int nmpc_io_bridge_main(int argc, char *argv[]); }

namespace nmpc_io_bridge
{

bool _initialized = false;
bool _is_running = false;
bool _debug = false;

static px4_task_t _task_handle = -1;

// Shared buffer for control data from sink callback -> main task
static control_packet_t _ctrl_buf;
static px4_sem_t _ctrl_sem;
static bool _ctrl_valid = false;

uORB::Subscription _nmpc_state_sub{ORB_ID(nmpc_state_data)};
uORB::Publication<nmpc_control_data_s> _nmpc_control_pub{ORB_ID(nmpc_control_data)};

static void control_sink_cb(int ch, char* data, int bytes, __attribute__((unused)) void* context)
{
    if (bytes != (int)sizeof(control_packet_t)) {
        if (_debug) PX4_WARN("control_sink_cb: unexpected size %d (expected %zu)", bytes, sizeof(control_packet_t));
        return;
    }
    memcpy(&_ctrl_buf, data, sizeof(control_packet_t));
    _ctrl_valid = true;
    px4_sem_post(&_ctrl_sem);
}

int initialize()
{
    if (_initialized) return 0;

    px4_sem_init(&_ctrl_sem, 0, 0);

    if (pipe_sink_create(0, CONTROL_SINK_PATH, SINK_FLAG_EN_SIMPLE_HELPER, PIPE_SIZE, READ_BUF_LEN)) {
        PX4_ERR("failed to create control sink at %s", CONTROL_SINK_PATH);
        return -1;
    }
    pipe_sink_set_simple_cb(0, &control_sink_cb, NULL);

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
                memcpy(pkt.x0, state_msg.x0, sizeof(pkt.x0));
                memcpy(pkt.p,  state_msg.p,  sizeof(pkt.p));
                pkt.hover_force = state_msg.hover_force;
                int ret = write(state_fd, &pkt, sizeof(pkt));
                if (ret < 0) {
                    if (_debug) PX4_WARN("state FIFO write failed, will retry");
                    close(state_fd);
                    state_fd = -1;
                }
            }
        }

        // Forward control from sink -> uORB (non-blocking check)
        if (px4_sem_trywait(&_ctrl_sem) == 0 && _ctrl_valid) {
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
