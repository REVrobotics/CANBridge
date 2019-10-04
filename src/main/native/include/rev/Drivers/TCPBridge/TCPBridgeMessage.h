#pragma once

#include <string>

#pragma once

#ifdef _WIN32
#define PACKED
#pragma pack(push, 1)
#else
#define PACKED __attribute__((__packed__))
#endif

/** Message packet definitions for CAN SparkMax Server */

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
typedef struct PACKED {
    canStreamerHeader_t header;
    uint32_t messageId;
    uint32_t timestamp;
    uint8_t datasize;
    uint8_t data[8];
    canStreamerTrailer_t trailer;
} canStreamer_canMessage_t;

#ifdef _WIN32
#pragma pack(pop)
#endif

#undef PACKED

class TCPBridgeMessage
{
    public:
    enum TCPBridgeCommands {
        CLOSE_STREAM_CMD = 0,
        READ_STREAM_CMD = 1,
        OPEN_STREAM_CMD = 2,
        READ_MSG_CMD = 3,
        SEND_MSG_CMD = 4,
        CAN_MSG_CMD = 5,
        INVALID_CMD
    };

    TCPBridgeMessage() {}
    TCPBridgeMessage(const char* bytebuf);
    TCPBridgeMessage(const TCPBridgeCommands cmd, const size_t pktSize);

    enum {
        HEADER_TOKEN = 0xAA,
        TRAILER_TOKEN = 0x55
    };

    enum {
        HEADER_IDX = 0,
        SIZE_IDX = 1,
        CMD_IDX = 2,
        RES_IDX = 3
    };

    enum {
        MAX_PKT_SIZE = 30,
        HEADER_SIZE = 4
    };

    virtual ~TCPBridgeMessage() {}

    void Serialize(char *bytebuf);
    virtual void FillData(char *bytebuf) = 0;
    bool Validate(const char *bytebuf);
    bool IsValid() const;
    static size_t GetHeaderSize();
    size_t GetSize() const;
    void SetSize(const size_t s);
    void BuildHeaderFooter(char *bytebuf) const;

    static bool ParseHeader(char *buf, TCPBridgeCommands *cmdType, size_t *pktLen);
    TCPBridgeCommands GetCommand() const;
    const canStreamerHeader_t BuildHeader(const TCPBridgeCommands cmdType, uint8_t pktLen);
    const canStreamerTrailer_t BuildTrailer();
    protected:
    bool m_isValid = false;
    size_t m_size;
    TCPBridgeCommands m_cmd = INVALID_CMD;
    uint32_t m_maxMessages = 0;
};

class ReadStreamPacket : public TCPBridgeMessage
{
    public:
    ReadStreamPacket(const char *bytebuf);
    ReadStreamPacket(const uint32_t maxMessages);

    virtual void FillData(char *bytebuf) override;
    static std::unique_ptr<TCPBridgeMessage> create(const char *bytebuf);
    uint32_t GetMaxMessages() const;

    private:
    uint32_t m_maxMessages;
    canStreamer_readStream_t m_messageStruct;
};

class OpenStreamPacket : public TCPBridgeMessage
{
    public:
    OpenStreamPacket(const char *bytebuf);
    OpenStreamPacket(const uint32_t messageID, const uint32_t messageMask, const uint32_t maxMessages);

    virtual void FillData(char *bytebuf) override;
    static std::unique_ptr<TCPBridgeMessage> create(const char *bytebuf);

    private:
    uint32_t m_messageID, m_messageMask, m_maxMessages;

    enum {
        OPEN_STREAM_PKT_SIZE = 17,
        MSG_ID_IDX = 4,
        MSG_ID_SIZE = 4,
        MSG_MASK_IDX = 8,
        MSG_MASK_SIZE = 4,
        MAX_MSGS_IDX = 12,
        MAX_MSGS_SIZE = 4
    };
};

class ReadMessagePacket : public TCPBridgeMessage
{
    public:
    ReadMessagePacket(const char *bytebuf);
    ReadMessagePacket(const uint32_t messageID, const uint32_t messageMask);

    virtual void FillData(char *bytebuf) override;
    static std::unique_ptr<TCPBridgeMessage> create(const char *bytebuf);

    private:
    uint32_t m_messageID, m_messageMask;

    enum {
        READ_MSG_PKT_SIZE = 14,
        MSG_ID_IDX = 4,
        MSG_ID_SIZE = 4,
        MSG_MASK_IDX = 8,
        MSG_MASK_SIZE = 4,
    };
};

class SendMessagePacket : public TCPBridgeMessage
{
    public:
    SendMessagePacket(const char *bytebuf);
    SendMessagePacket(const uint32_t messageID, const uint32_t periodMs, const uint8_t datasize, const uint8_t *data);

    virtual void FillData(char *bytebuf) override;
    static std::unique_ptr<TCPBridgeMessage> create(const char *bytebuf);

    private:
    uint32_t m_messageID, m_periodMs;
    uint8_t m_datasize, m_data[8];

    enum {
        SEND_MSG_PKT_SIZE = 22,
        MSG_ID_IDX = 4,
        MSG_ID_SIZE = 4,
        PERIOD_MS_IDX = 8,
        DATASIZE_IDX = 12,
        DATA_IDX = 13,
        PERIOD_MS_SIZE = 4,
    };
};

class CANMessagePacket : public TCPBridgeMessage
{
    public:
    CANMessagePacket(const char *bytebuf);
    CANMessagePacket(const uint32_t messageID, const uint32_t timestamp, const uint8_t datasize, const uint8_t *data);

    virtual void FillData(char *bytebuf) override;
    static std::unique_ptr<TCPBridgeMessage> create(const char *bytebuf);

    uint32_t GetMessageID() const;
    uint32_t GetTimestamp() const;
    uint8_t GetDatasize() const;
    const uint8_t* GetData() const;

    private:
    uint8_t m_datasize;
    uint8_t m_data[8];
    uint32_t m_messageID, m_timestamp;

    enum {
        CAN_MSG_PKT_SIZE = 22,
        MSG_ID_IDX = 4,
        MSG_ID_SIZE = 4,
        TIMESTAMP_IDX = 8,
        TIMESTAMP_SIZE = 4,
        DATASIZE_IDX = 12,
        DATASIZE_SIZE = 1,
        DATA_IDX = 13
    };
};

class InvalidPacket : public TCPBridgeMessage
{
    public:
    InvalidPacket();
    static std::unique_ptr<TCPBridgeMessage> create();
    virtual void FillData(char *bytebuf) override;
};