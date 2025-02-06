/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef CBF_DEBUG_HPP
#define CBF_DEBUG_HPP

#include <uORB/topics/cbf_debug.h>

class MavlinkStreamCbfDebug : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCbfDebug(mavlink); }

	static constexpr const char *get_name_static() { return "CBF_DEBUG"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CBF_DEBUG; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		static constexpr unsigned size_per_batch = MAVLINK_MSG_ID_CBF_DEBUG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return _cbf_debug_sub.advertised() ? size_per_batch * _number_of_batches : 0;
	}

private:
	explicit MavlinkStreamCbfDebug(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _cbf_debug_sub{ORB_ID(cbf_debug)};
	uint8_t _number_of_batches{0};

	bool send() override
	{
		cbf_debug_s cbf_debug;

		if (_cbf_debug_sub.update(&cbf_debug)) {
			mavlink_cbf_debug_t msg{};

			msg.time_usec = cbf_debug.timestamp;
			msg.cbf_duration = cbf_debug.cbf_duration;
			msg.qp_fail = cbf_debug.qp_fail;
			msg.h = cbf_debug.h;
			msg.h1 = cbf_debug.h1;
			msg.h2 = cbf_debug.h2;
			msg.input[0] = cbf_debug.input[0];
			msg.input[1] = cbf_debug.input[1];
			msg.input[2] = cbf_debug.input[2];
			msg.output[0] = cbf_debug.output[0];
			msg.output[1] = cbf_debug.output[1];
			msg.output[2] = cbf_debug.output[2];
			msg.slack[0] = cbf_debug.slack[0];
			msg.slack[1] = cbf_debug.slack[1];

			mavlink_msg_cbf_debug_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // CBF_DEBUG_HPP
