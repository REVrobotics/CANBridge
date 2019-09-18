#include <stdio.h>
#include <stdint.h>
#include <memory.h>

#include "rev/Drivers/TCPBridge/TCPBridgePackets.h"

// namespace CANStreamerPackets {
// struct receive_message_packet_t {
//     uint32_t message_id;
// 	uint32_t message_mask;
//     uint8_t data_size;
// };

// struct send_message_packet_t {
//     uint32_t message_id;
// 	uint32_t period_ms;
//     uint8_t data_size;
//     uint8_t data[8];
// };

// struct open_stream_packet_t {
// 	uint32_t message_id;
// 	uint32_t message_mask;
// 	uint32_t max_messages;
// };

// struct read_stream_packet_t {
// 	uint32_t max_messages;
// };
// }

namespace rev {
namespace usb {

// inline void serialize_uint32(uint32_t val, uint8_t *buf) {
//     buf[0] = (val & 0xFF);
//     buf[1] = (val >> 8) & 0xFF;
//     buf[2] = (val >> 16) & 0xFF;
//     buf[3] = (val >> 24) & 0xFF;
// }

// int check_packet(uint8_t *buf, int *packet_size) {
//     if(buf[0] != HEADER_TOKEN) {
//         printf("Invalid command packet rec'd\n");
//         return -1;
//     }

//     *packet_size = buf[1];

//     if((*packet_size > MAXBUFFERSIZE) || (buf[*packet_size-1] != TRAILER_TOKEN)) {
//         printf("Invalid packet size rec'd: %d\n", *packet_size-1);
//         return -2;
//     }

//     return 0;
// }

// void serialize_readstream_packet(struct read_stream_packet_t *pkt, uint8_t *sendbuf) {
// 	sendbuf[0] = HEADER_TOKEN;
// 	sendbuf[1] = READSTREAM_PACKET_SIZE;
// 	sendbuf[2] = READ_STREAM;
// 	sendbuf[3] = 0x00;
	
//     serialize_uint32(pkt->max_messages, &sendbuf[4]);
	
// 	sendbuf[8] = TRAILER_TOKEN;
// }

// void serialize_openstream_packet(struct open_stream_packet_t *pkt, uint8_t *sendbuf) {
// 	sendbuf[0] = HEADER_TOKEN;
// 	sendbuf[1] = OPENSTREAM_PACKET_SIZE;
// 	sendbuf[2] = OPEN_STREAM;
// 	sendbuf[3] = 0x00;

//     serialize_uint32(pkt->message_id, &sendbuf[4]);
// 	serialize_uint32(pkt->message_mask, &sendbuf[8]);
//     serialize_uint32(pkt->max_messages, &sendbuf[12]);
	
// 	sendbuf[16] = TRAILER_TOKEN;
// }

// void serialize_read_msg_packet(struct receive_message_packet_t *pkt, uint8_t *sendbuf) {
// 	sendbuf[0] = HEADER_TOKEN;
// 	sendbuf[1] = READMSG_PACKET_SIZE;
// 	sendbuf[2] = RCV_MSG;
// 	sendbuf[3] = 0;
	
// 	serialize_uint32(pkt->message_id, &sendbuf[4]);
//     serialize_uint32(pkt->message_mask, &sendbuf[8]);
// 	sendbuf[12] = pkt->data_size;
	
// 	sendbuf[13] = TRAILER_TOKEN;
// }

// void serialize_send_msg_packet(struct send_message_packet_t *pkt, uint8_t *sendbuf) {
// 	sendbuf[0] = HEADER_TOKEN;
// 	sendbuf[1] = SENDMSG_PACKET_SIZE;
// 	sendbuf[2] = SEND_MSG;
// 	sendbuf[3] = 0;
	
// 	serialize_uint32(pkt->message_id, &sendbuf[4]);
// 	serialize_uint32(pkt->period_ms, &sendbuf[8]);
// 	sendbuf[12] = pkt->data_size;
// 	memcpy_s(sendbuf + 13, BYTEBUFSIZE, pkt->data, pkt->data_size);
	
// 	sendbuf[21] = TRAILER_TOKEN;
// }

}
}