#pragma once

// Stub mavlink include for build compatibility
// In a real system this would include the actual MAVLink headers

#ifndef MAVLINK_MAX_PACKET_LEN
#define MAVLINK_MAX_PACKET_LEN 280
#endif

#ifndef MAVLINK_FRAMING_INCOMPLETE
#define MAVLINK_FRAMING_INCOMPLETE 0
#endif
#ifndef MAVLINK_FRAMING_OK
#define MAVLINK_FRAMING_OK 1
#endif
#ifndef MAVLINK_FRAMING_BAD_CRC
#define MAVLINK_FRAMING_BAD_CRC 2
#endif
#ifndef MAVLINK_FRAMING_BAD_SIGNATURE
#define MAVLINK_FRAMING_BAD_SIGNATURE 3
#endif

typedef struct __mavlink_message {
    uint16_t checksum;
    uint8_t magic;
    uint8_t len;
    uint8_t incompat_flags;
    uint8_t compat_flags;
    uint8_t seq;
    uint8_t sysid;
    uint8_t compid;
    uint32_t msgid;
    uint8_t payload64[280];
    uint8_t ck[2];
    uint8_t signature[13];
} mavlink_message_t;

static inline uint16_t mavlink_msg_to_send_buffer(uint8_t *buf, const mavlink_message_t *msg)
{
    (void)msg;
    // Stub: in real code this serializes the message
    // Return a small valid length for testing
    if (buf) {
        buf[0] = 0xFE;
    }
    return 10;
}