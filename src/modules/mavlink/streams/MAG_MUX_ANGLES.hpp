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

#ifndef MAG_MUX_ANGLES_HPP
#define MAG_MUX_ANGLES_HPP

// #include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_angles.h>

class MavlinkStreamMagMuxAngles : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMagMuxAngles(mavlink); }

	static constexpr const char *get_name_static() { return "MAG_MUX_ANGLES"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MAG_MUX_ANGLES; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		static constexpr unsigned size_per_batch = MAVLINK_MSG_ID_MAG_MUX_ANGLES_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return _sensor_angles_sub.advertised() ? size_per_batch * _number_of_batches : 0;
	}

private:
	explicit MavlinkStreamMagMuxAngles(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_angles_sub{ORB_ID(sensor_angles)};
	uint8_t _number_of_batches{0};

	bool send() override
	{
		// static constexpr uint8_t batch_size = MAVLINK_MSG_MAG_MUX_FIELD_TEMPERATURE_LEN;
		sensor_angles_s sensor_angles;

		if (_sensor_angles_sub.update(&sensor_angles)) {
			mavlink_mag_mux_angles_t msg{};

			msg.time_usec = sensor_angles.timestamp;

			for(int i=0; i<4; i++)
			{
				msg.azimuth[i] =  sensor_angles.azimuth[i];
				msg.elevation[i] =  sensor_angles.elevation[i];
			}

			mavlink_msg_mag_mux_angles_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // MAG_MUX_ANGLES_HPP
