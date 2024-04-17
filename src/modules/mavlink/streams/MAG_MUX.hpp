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

#ifndef MAG_MUX_HPP
#define MAG_MUX_HPP

// #include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_mag_mux.h>

class MavlinkStreamMagMagMux : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMagMagMux(mavlink); }

	static constexpr const char *get_name_static() { return "MAG_MUX"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MAG_MUX; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		static constexpr unsigned size_per_batch = MAVLINK_MSG_ID_MAG_MUX_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return _sensor_mag_mux_sub.advertised() ? size_per_batch * _number_of_batches : 0;
	}

private:
	explicit MavlinkStreamMagMagMux(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_mag_mux_sub{ORB_ID(sensor_mag_mux)};
	uint8_t _number_of_batches{0};

	bool send() override
	{
		// static constexpr uint8_t batch_size = MAVLINK_MSG_MAG_MUX_FIELD_TEMPERATURE_LEN;
		sensor_mag_mux_s sensor_mag_mux;

		if (_sensor_mag_mux_sub.update(&sensor_mag_mux)) {
			mavlink_mag_mux_t msg{};

			msg.time_usec = sensor_mag_mux.timestamp;
			// PX4_INFO("%lu ", msg.time_usec);

			for(int i=0; i<4; i++)
			{
				msg.mags[i*3] =  sensor_mag_mux.mags[i].x;
				msg.mags[i*3 + 1] =  sensor_mag_mux.mags[i].y;
				msg.mags[i*3 + 2] =  sensor_mag_mux.mags[i].z;
			}

			mavlink_msg_mag_mux_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // MAG_MUX_HPP
