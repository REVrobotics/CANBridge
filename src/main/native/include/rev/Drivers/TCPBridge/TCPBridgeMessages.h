#pragma once

#include <stdint.h>
#include <cstring>

#ifdef _WIN32
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((__packed__))
#endif

constexpr uint8_t HEADER_TOKEN = 0xAA;
constexpr uint8_t TRAILER_TOKEN = 0x55;

/** Header Defs */
typedef struct PACKED {
    uint8_t headerToken;
    uint8_t packetSize;
    uint8_t commandId;
    uint8_t res;
} canStreamerHeader_t;

typedef struct PACKED {
    uint8_t trailerToken;
} canStreamerTrailer_t;

/** CloseStream Packet */
typedef struct PACKED {
    canStreamerHeader_t header;
    uint32_t handle;
    canStreamerTrailer_t trailer;
} canStreamer_closeStream_t;

/** OpenStream Packet */
typedef struct PACKED {
    canStreamerHeader_t header;
    uint32_t handle;
    uint32_t messageId;
    uint32_t messageMask;
    uint32_t maxMessages;
    int32_t status;
    canStreamerTrailer_t trailer;
} canStreamer_openStream_t;

/** ReadStream Packet */
typedef struct PACKED {
    canStreamerHeader_t header;
    uint32_t handle;
    uint32_t messagesToRead;
    uint32_t messagesRead;
    int32_t status;
    canStreamerTrailer_t trailer;
} canStreamer_readStream_t;

/** ReceiveMessage Packet */
typedef struct PACKED {
    canStreamerHeader_t header;
    uint32_t messageId;
    uint32_t messageMask;
    uint8_t data[8];
    uint8_t dataSize;
    uint32_t timestamp;
    int32_t status;
    canStreamerTrailer_t trailer;
} canStreamer_readMessage_t;

/** SendMessage Packet */
typedef struct PACKED {
    canStreamerHeader_t header;
    uint32_t messageId;
    uint8_t data[8];
    uint8_t datasize;
    int32_t periodMs;
    int32_t status;
    canStreamerTrailer_t trailer;
} canStreamer_sendMessage_t;

/** CANMessage Packet */
typedef struct PACKED {
    canStreamerHeader_t header;
    uint32_t messageId;
    uint32_t timestamp;
    uint8_t datasize;
    uint8_t data[8];
    canStreamerTrailer_t trailer;
} canStreamer_canMessage_t;

/** Bus Util Packet */
typedef struct PACKED {
    canStreamerHeader_t header;
    float utilization;
    uint32_t busOff;
    uint32_t txFull;
    uint32_t receiveErr;
    uint32_t transmitErr;
    int32_t status;
    canStreamerTrailer_t trailer;
} canStreamer_busUtil_t;

/** Status Packet */
typedef struct PACKED {
    canStreamerHeader_t header;
    uint32_t messages;
    uint32_t status;
    canStreamerTrailer_t trailer;
} canStreamer_status_t;

/** Invalid Packet */
typedef struct PACKED {
    canStreamerHeader_t header;
    canStreamerTrailer_t trailer;
} canStreamer_invalid_t;

typedef union {
    canStreamerHeader_t header;
    canStreamer_closeStream_t closeStream;
    canStreamer_openStream_t openStream;
    canStreamer_readStream_t readStream;
    canStreamer_readMessage_t readMessage;
    canStreamer_sendMessage_t sendMessage;
    canStreamer_canMessage_t canMessage;
    canStreamer_busUtil_t busUtil;
    canStreamer_status_t statusMessage;
    canStreamer_invalid_t invalidMessage;
} TCPBridgeMessages_t;

#ifdef _WIN32
#pragma pack(pop)
#endif

#undef PACKED

/** Commands */
enum TCPBridgeCommands {
    CLOSE_STREAM_CMD = 0,
    OPEN_STREAM_CMD = 1,
    READ_STREAM_CMD = 2,
    READ_MSG_CMD = 3,
    SEND_MSG_CMD = 4,
    CAN_MSG_CMD = 5,
    STATUS_CMD = 6,
    BUS_UTIL_CMD = 7,
    INVALID_CMD = 8,
};

static uint8_t TCPBridgePacketSizes[] = {
    sizeof(canStreamer_closeStream_t),
    sizeof(canStreamer_openStream_t),
    sizeof(canStreamer_readStream_t),
    sizeof(canStreamer_readMessage_t),
    sizeof(canStreamer_sendMessage_t),
    sizeof(canStreamer_canMessage_t),
    sizeof(canStreamer_status_t),
    sizeof(canStreamer_busUtil_t),
    sizeof(canStreamer_invalid_t)
};

template<typename T>
inline void serializePacket(void *buf, uint8_t commandId, T *pkt) {
    pkt->header.headerToken = HEADER_TOKEN;
    pkt->header.commandId = commandId;
    pkt->header.packetSize = sizeof(T);
    pkt->trailer.trailerToken = TRAILER_TOKEN;
    memcpy(buf, pkt, sizeof(T));
}

template<typename T>
inline void serializePacket(void *buf, T *pkt) {
    memcpy(buf, pkt, sizeof(T));
}

template<typename T>
inline void deserializePacket(const void *bytebuf, T *pkt) {
    memcpy(pkt, bytebuf, sizeof(T));
}

inline void serializeCloseStreamPacket(void *buf, uint32_t handle) {
    canStreamer_closeStream_t pkt;
    pkt.handle = handle;
    serializePacket<canStreamer_closeStream_t>(buf, CLOSE_STREAM_CMD, &pkt);
}

inline void serializeOpenStreamPacket(void *buf, 
                                      uint32_t handle, 
                                      uint32_t messageId, 
                                      uint32_t messageMask, 
                                      uint32_t maxMessages,
                                      int32_t status) {
    canStreamer_openStream_t pkt;
    pkt.handle = handle;
    pkt.messageId = messageId;
    pkt.messageMask = messageMask;
    pkt.maxMessages = maxMessages;
    pkt.status = status;
    serializePacket<canStreamer_openStream_t>(buf, OPEN_STREAM_CMD, &pkt);
}

inline void serializeReadStreamPacket(void *buf,
                                      uint32_t handle, 
                                      uint32_t messagesToRead,
                                      uint32_t messagesRead,
                                      int32_t status) {
    canStreamer_readStream_t pkt;
    pkt.handle = handle;
    pkt.messagesToRead = messagesToRead;
    pkt.messagesRead = messagesRead;
    pkt.status = status;
    serializePacket<canStreamer_readStream_t>(buf, READ_STREAM_CMD, &pkt);
}

inline void serializeReceiveMessagePacket(void *buf,
                                          uint32_t messageId, 
                                          uint32_t messageMask,
                                          const uint8_t *data,
                                          uint8_t dataSize,
                                          uint32_t timestamp,
                                          int32_t status) {
    canStreamer_readMessage_t pkt;
    pkt.messageId = messageId;
    pkt.messageMask = messageMask;
    pkt.dataSize = dataSize;
    pkt.timestamp = timestamp;
    pkt.status = status;
    memcpy(buf, data, dataSize > 8 ? 8 : dataSize);
    serializePacket<canStreamer_readMessage_t>(buf, READ_MSG_CMD, &pkt);
}

inline void serializeSendMessagePacket(void *buf, 
                                       uint32_t messageId, 
                                       uint32_t periodMs, 
                                       uint8_t datasize, 
                                       const uint8_t *data,
                                       int32_t status) {
    canStreamer_sendMessage_t pkt;
    pkt.messageId = messageId;
    pkt.periodMs = periodMs;
    pkt.datasize = datasize;
    memcpy(pkt.data, data, datasize > 8 ? 8 : datasize);
    serializePacket<canStreamer_sendMessage_t>(buf, SEND_MSG_CMD, &pkt);
}

inline void serializeCanMessagePacket(void *buf,
                                      uint32_t messageId,
                                      uint32_t timestamp,
                                      uint8_t datasize,
                                      const uint8_t *data) {
    canStreamer_canMessage_t pkt;
    pkt.messageId = messageId;
    pkt.timestamp = timestamp;
    pkt.datasize = datasize;
    memcpy(pkt.data, data, datasize > 8 ? 8 : datasize);
    serializePacket<canStreamer_canMessage_t>(buf, CAN_MSG_CMD, &pkt);
}

inline void serializeBusUtilPacket(void *buf,
                                   float utilization,
                                   uint32_t busOff, 
                                   uint32_t txFull, 
                                   uint32_t receiveErr, 
                                   uint32_t transmitErr,
                                   int32_t status) {
    canStreamer_busUtil_t pkt;
    pkt.utilization = utilization;
    pkt.busOff = busOff;
    pkt.txFull = txFull;
    pkt.receiveErr = receiveErr;
    pkt.transmitErr = transmitErr;
    pkt.status = status;
    serializePacket<canStreamer_busUtil_t>(buf, BUS_UTIL_CMD, &pkt);
}

inline void serializeStatusPacket(void *buf,
                                  uint32_t messages,
                                  uint32_t status) {
    canStreamer_status_t pkt;
    pkt.messages = messages;
    pkt.status = status;
    serializePacket<canStreamer_status_t>(buf, STATUS_CMD, &pkt);
}

inline bool checkPacket(TCPBridgeMessages_t *pkt) {
    if(pkt->header.headerToken != HEADER_TOKEN) {
        return false;
    }

    if(pkt->header.packetSize != TCPBridgePacketSizes[pkt->header.commandId]) {
        return false;
    }

    if(*((uint8_t*)pkt + pkt->header.packetSize - 1) != TRAILER_TOKEN) {
        return false;
    }

    return true;
}
