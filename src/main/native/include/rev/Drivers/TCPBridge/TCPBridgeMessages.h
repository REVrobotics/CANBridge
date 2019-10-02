#pragma once

#include <stdint.h>

#ifdef _WIN32
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((__packed__))
#endif

/** Message packet definitions for CAN SparkMax Server */

extern "C" {
/** Commands */
enum canstreamer_cmd {
    CLOSE_STREAM_CMD = 0,
    READ_STREAM_CMD = 1,
    OPEN_STREAM_CMD = 2,
    READ_MSG_CMD = 3,
    SEND_MSG_CMD = 4,
    CAN_MSG_CMD = 5,
    INVALID_CMD
};

constexpr int HEADER_TOKEN = 0xAA;
constexpr int TRAILER_TOKEN = 0x55;

constexpr size_t MAXBUFFERSIZE = 1024;


/** Header Defs */
constexpr int _CAN_SERVER_HEADER_IDX = 0;
constexpr int _CAN_SERVER_SIZE_IDX = 1;
constexpr int _CAN_SERVER_CMD_IDX = 2;
constexpr int _CAN_SERVER_RES_IDX = 3;
constexpr int _CAN_SERVER_HEADER_SIZE = 4;

typedef struct PACKED {
    uint8_t headerToken;
    uint8_t packetSize;
    uint8_t commandId;
    uint8_t res;
} canStreamerHeader_t;

typedef struct PACKED {
    uint8_t trailerToken;
} canStreamerTrailer_t;

/** ReadStream Packet */
typedef struct PACKED {
    canStreamerHeader_t header;
    uint32_t maxMessages;
    canStreamerTrailer_t trailer;
} canStreamer_readStream_t;

/** OpenStream Packet */
typedef struct PACKED {
    canStreamerHeader_t header;
    uint32_t messageId;
    uint32_t messageMask;
    uint32_t maxMessages;
    canStreamerTrailer_t trailer;
} canStreamer_openStream_t;

/** ReadMessage Packet */
typedef struct PACKED {
    canStreamerHeader_t header;
    uint32_t messageId;
    uint32_t messageMask;
    canStreamerTrailer_t trailer;
} canStreamer_readMessage_t;

/** SendMessage Packet */
typedef struct PACKED {
    canStreamerHeader_t header;
    uint32_t messageId;
    uint32_t periodMs;
    uint8_t datasize;
    uint8_t data[8];
    canStreamerTrailer_t trailer;
} canStreamer_sendMessage_t;

/** CANMessage Packet */
constexpr int _CAN_SERVER_CANMSG_MSG_ID_IDX = 4;
constexpr int _CAN_SERVER_CANMSG_TIMESTAMP_IDX = 8;
constexpr int _CAN_SERVER_CANMSG_DATASIZE_IDX = 12;
constexpr int _CAN_SERVER_CANMSG_DATA_IDX = 13;
constexpr int _CAN_SERVER_CANMSG_MSG_ID_SIZE = 4;
constexpr int _CAN_SERVER_CANMSG_TIMESTAMP_SIZE = 4;
constexpr int _CAN_SERVER_CANMSG_DATASIZE_SIZE = 1;
constexpr int _CAN_SERVER_CANMSG_DATA_SIZE = 8;
constexpr int _CAN_SERVER_CANMSG_PACKET_SIZE = 22;

typedef struct PACKED {
    canStreamerHeader_t header;
    uint32_t messageId;
    uint32_t timestamp;
    uint8_t datasize;
    uint8_t data[8];
    canStreamerTrailer_t trailer;
} canStreamer_canMessage_t;

constexpr int _CAN_SERVER_CANMSG_MAX_PACKET_SIZE = 30;

#ifdef _WIN32
#pragma pack(pop)
#endif

#undef PACKED
}