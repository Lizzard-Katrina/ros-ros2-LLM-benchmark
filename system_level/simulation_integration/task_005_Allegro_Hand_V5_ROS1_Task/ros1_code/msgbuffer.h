#pragma once

#include <cassert>
#include "mavlink_include.h"

namespace gazebo {
/**
 * @brief Message buffer for internal use in libmavconn
 */
struct MsgBuffer {
	//! Maximum buffer size with padding for CRC bytes (280 + padding)
	static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
	uint8_t data[MAX_SIZE];
	ssize_t len;
	ssize_t pos;

	MsgBuffer() :
		pos(0),
		len(0)
	{ }

	explicit MsgBuffer(const mavlink_message_t *msg) :
		pos(0)
	{
		len = mavlink_msg_to_send_buffer(data, msg);
		assert(len < MAX_SIZE);
	}

	MsgBuffer(const uint8_t *bytes, ssize_t nbytes) :
		pos(0),
		len(nbytes)
	{
		assert(0 < nbytes && nbytes < MAX_SIZE);
		memcpy(data, bytes, nbytes);
	}

	virtual ~MsgBuffer() {
		pos = 0;
		len = 0;
	}

	uint8_t *dpos() {
		return data + pos;
	}

	ssize_t nbytes() {
		return len - pos;
	}
};
}	// namespace gazebo
