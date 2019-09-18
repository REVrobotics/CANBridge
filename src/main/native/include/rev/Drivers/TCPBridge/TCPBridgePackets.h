#pragma once

#define HEADER_TOKEN    0xAA
#define TRAILER_TOKEN   0x55

constexpr size_t MAXBUFFERSIZE = 1024;
constexpr size_t BYTEBUFSIZE = 30;
constexpr uint8_t READSTREAM_PACKET_SIZE = 9;
constexpr uint8_t OPENSTREAM_PACKET_SIZE = 17;
constexpr uint8_t READMSG_PACKET_SIZE = 14;
constexpr uint8_t SENDMSG_PACKET_SIZE = 22;

namespace rev {
namespace usb {

struct receive_message_packet_t {
    uint32_t message_id;
	uint32_t message_mask;
    uint8_t data_size;
};

struct send_message_packet_t {
    uint32_t message_id;
	uint32_t period_ms;
    uint8_t data_size;
    uint8_t data[8];
};

struct open_stream_packet_t {
	uint32_t message_id;
	uint32_t message_mask;
	uint32_t max_messages;
};

struct read_stream_packet_t {
	uint32_t max_messages;
};

enum canstreamer_cmd {
    CLOSE_STREAM = 0,
    READ_STREAM = 1,
    OPEN_STREAM = 2,
	RCV_MSG = 3,
	SEND_MSG = 4,
};

// inline void serialize_uint32(uint32_t val, uint8_t *buf);
// int check_packet(uint8_t *buf, int *packet_size);
// void serialize_readstream_packet(struct read_stream_packet_t *pkt, uint8_t *sendbuf);
// void serialize_openstream_packet(struct open_stream_packet_t *pkt, uint8_t *sendbuf);
// void serialize_read_msg_packet(struct receive_message_packet_t *pkt, uint8_t *sendbuf);
// void serialize_send_msg_packet(struct send_message_packet_t *pkt, uint8_t *sendbuf);
}
}