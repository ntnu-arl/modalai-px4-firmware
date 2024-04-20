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

#ifndef MAG_MUX_CALIB_HPP
#define MAG_MUX_CALIB_HPP

// #include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_mag_mux_calib.h>

class MavlinkStreamMagMuxCalib : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMagMuxCalib(mavlink); }

	static constexpr const char *get_name_static() { return "MAG_MUX_CALIB"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MAG_MUX_CALIB; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		static constexpr unsigned size_per_batch = MAVLINK_MSG_ID_MAG_MUX_CALIB_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		return _sensor_mag_mux_calib_sub.advertised() ? size_per_batch * _number_of_batches : 0;
	}

private:
	explicit MavlinkStreamMagMuxCalib(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_mag_mux_calib_sub{ORB_ID(sensor_mag_mux_calib)};
	uint8_t _number_of_batches{0};

	bool send() override
	{
		// static constexpr uint8_t batch_size = MAVLINK_MSG_MAG_MUX_CALIB_FIELD_TEMPERATURE_LEN;
		sensor_mag_mux_calib_s sensor_mag_mux_calib;

		if (_sensor_mag_mux_calib_sub.update(&sensor_mag_mux_calib)) {
			mavlink_mag_mux_calib_t msg{};

			msg.time_usec = sensor_mag_mux_calib.timestamp;
			// PX4_INFO("%lu ", msg.time_usec);

			for(int i=0; i<4; i++)
			{
				msg.xyz[i*3] =  sensor_mag_mux_calib.mags[i].xyz[0];
				msg.xyz[i*3 + 1] =  sensor_mag_mux_calib.mags[i].xyz[1];
				msg.xyz[i*3 + 2] =  sensor_mag_mux_calib.mags[i].xyz[2];
				msg.center[i*3] =  sensor_mag_mux_calib.mags[i].center[0];
				msg.center[i*3 + 1] =  sensor_mag_mux_calib.mags[i].center[1];
				msg.center[i*3 + 2] =  sensor_mag_mux_calib.mags[i].center[2];
				msg.max[i*3] =  sensor_mag_mux_calib.mags[i].max[0];
				msg.max[i*3 + 1] =  sensor_mag_mux_calib.mags[i].max[1];
				msg.max[i*3 + 2] =  sensor_mag_mux_calib.mags[i].max[2];
			}

			mavlink_msg_mag_mux_calib_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // MAG_MUX_CALIB_HPP
