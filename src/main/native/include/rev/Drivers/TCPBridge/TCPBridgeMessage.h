#pragma once

#include <string>

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
    static const size_t GetHeaderSize();
    const size_t GetSize() const;
    void SetSize(const size_t s);
    void BuildHeaderFooter(char *bytebuf);
    static bool ParseHeader(char *buf, TCPBridgeCommands *cmdType, size_t *pktLen);
    TCPBridgeCommands GetCommand() const;

    private:
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

    void FillData(char *bytebuf);
    static std::unique_ptr<TCPBridgeMessage> create(const char *bytebuf);
    const uint32_t GetMaxMessages() const;

    private:
    uint32_t m_maxMessages;

    enum {
        READ_STREAM_PKT_SIZE = 9,
        MAX_MSGS_IDX = 4,
        MAX_MSGS_SIZE = 4
    };
};

class OpenStreamPacket : public TCPBridgeMessage
{
    public:
    OpenStreamPacket(const char *bytebuf);
    OpenStreamPacket(const uint32_t messageID, const uint32_t messageMask, const uint32_t maxMessages);

    void FillData(char *bytebuf);
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

    void FillData(char *bytebuf);
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

    void FillData(char *bytebuf);
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

    void FillData(char *bytebuf);
    static std::unique_ptr<TCPBridgeMessage> create(const char *bytebuf);

    const uint32_t GetMessageID() const;
    const uint32_t GetTimestamp() const;
    const uint8_t GetDatasize() const;
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