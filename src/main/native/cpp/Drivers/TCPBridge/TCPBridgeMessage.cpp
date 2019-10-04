#include <iostream>
#include <string>

#include "rev/Drivers/TCPBridge/TCPBridgeMessage.h"

/**
 * TCPBridgeMessage
 */

TCPBridgeMessage::TCPBridgeMessage(const char* bytebuf) {
    if(Validate(bytebuf))
    {
        m_isValid = true;
    }
    else 
    {
        std::cerr << "Validation failed" << std::endl;
    }
}

TCPBridgeMessage::TCPBridgeMessage(const TCPBridgeCommands cmd, const size_t pktSize)
    : m_cmd(cmd) 
{
    SetSize(pktSize);
}

bool TCPBridgeMessage::Validate(const char *bytebuf) {
    m_size = static_cast<size_t>(bytebuf[SIZE_IDX]);

    if(m_size > MAX_PKT_SIZE)
    {
        std::cout << "Size " << m_size << " is larger that max packet size" << std::endl;
        return false;
    }

    if(!(static_cast<uint8_t>(bytebuf[HEADER_IDX]) == HEADER_TOKEN) || !(static_cast<uint8_t>(bytebuf[m_size - 1]) == TRAILER_TOKEN))
    {
        printf("Header: 0x%02X\n", static_cast<uint8_t>(bytebuf[HEADER_IDX]));
        printf("Trailer: 0x%02X\n", static_cast<uint8_t>(bytebuf[m_size - 1]));
        for(auto i = 0; i < MAX_PKT_SIZE; i++)
            printf("%d: 0x%02X\n", i, static_cast<uint8_t>(bytebuf[i]));
        return false;
    }

    m_cmd = static_cast<TCPBridgeCommands>(bytebuf[CMD_IDX]);

    return true;
}

bool TCPBridgeMessage::IsValid() const {
    return m_isValid;
}

size_t TCPBridgeMessage::GetHeaderSize() {
    return HEADER_SIZE;
}

size_t TCPBridgeMessage::GetSize() const {
    return m_size;
}

void TCPBridgeMessage::SetSize(const size_t s)
{
    m_size = s > MAX_PKT_SIZE ? MAX_PKT_SIZE : s;
}

void TCPBridgeMessage::BuildHeaderFooter(char *bytebuf) const
{
    bytebuf[HEADER_IDX] = static_cast<uint8_t>(HEADER_TOKEN);
    bytebuf[SIZE_IDX] = m_size;
    bytebuf[CMD_IDX] = m_cmd;
    bytebuf[m_size-1] = static_cast<uint8_t>(TRAILER_TOKEN);
}

void TCPBridgeMessage::Serialize(char *bytebuf)
{
    BuildHeaderFooter(bytebuf);
    FillData(bytebuf);
}

bool TCPBridgeMessage::ParseHeader(char *buf, TCPBridgeCommands *cmdType, size_t *pktLen) 
{
    *cmdType = static_cast<TCPBridgeCommands>(buf[CMD_IDX]);
    *pktLen = static_cast<size_t>(buf[SIZE_IDX]);
    
    return *pktLen <= MAX_PKT_SIZE;
}

TCPBridgeMessage::TCPBridgeCommands TCPBridgeMessage::GetCommand() const 
{
    return m_cmd;
}

const canStreamerHeader_t TCPBridgeMessage::BuildHeader(const TCPBridgeCommands cmdType, uint8_t pktLen)
{
    canStreamerHeader_t header;
    header.headerToken = HEADER_TOKEN;
    header.commandId = static_cast<uint8_t>(cmdType);
    header.packetSize = pktLen;

    return header;
}

const canStreamerTrailer_t TCPBridgeMessage::BuildTrailer()
{
    canStreamerTrailer_t trailer;
    trailer.trailerToken = TRAILER_TOKEN;
    return trailer;
}

/**
 * ReadStreamPacket
 */

ReadStreamPacket::ReadStreamPacket(const char *bytebuf)
    : TCPBridgeMessage(bytebuf) {
    if(IsValid())
    {
        // memcpy(&m_maxMessages, &bytebuf[MAX_MSGS_IDX], MAX_MSGS_SIZE);
        memcpy(&m_messageStruct, bytebuf, sizeof(canStreamer_readStream_t));
    }
    else 
    {
        std::cout << "Invalid packet rec'd" << std::endl;
    }
}

ReadStreamPacket::ReadStreamPacket(const uint32_t maxMessages)
    : TCPBridgeMessage(READ_STREAM_CMD, sizeof(canStreamer_readStream_t))
{
    m_messageStruct.header = BuildHeader(READ_STREAM_CMD, sizeof(canStreamer_readStream_t));
    m_messageStruct.maxMessages = maxMessages;
    m_messageStruct.trailer = BuildTrailer();
}

std::unique_ptr<TCPBridgeMessage> ReadStreamPacket::create(const char *bytebuf) 
{ 
    return std::make_unique<ReadStreamPacket>(bytebuf); 
}

void ReadStreamPacket::FillData(char *bytebuf) 
{
    memcpy(bytebuf, &m_messageStruct, sizeof(m_messageStruct)); // TODO: implement this for all messages
}

uint32_t ReadStreamPacket::GetMaxMessages() const
{
    return m_messageStruct.maxMessages;
}

/**
 * OpenStreamPacket
 */

OpenStreamPacket::OpenStreamPacket(const char *bytebuf)
    : TCPBridgeMessage(bytebuf) {
    if(IsValid())
    {
        memcpy(&m_messageID, &bytebuf[MSG_ID_IDX], MSG_ID_SIZE);
        memcpy(&m_messageMask, &bytebuf[MSG_MASK_IDX], MSG_MASK_SIZE);
        memcpy(&m_maxMessages, &bytebuf[MAX_MSGS_IDX], MAX_MSGS_SIZE);
    }
}

OpenStreamPacket::OpenStreamPacket(const uint32_t messageID, const uint32_t messageMask, const uint32_t maxMessages)
    : TCPBridgeMessage(OPEN_STREAM_CMD, OPEN_STREAM_PKT_SIZE),
    m_messageID(messageID),
    m_messageMask(messageMask),
    m_maxMessages(maxMessages)
{

}

std::unique_ptr<TCPBridgeMessage> OpenStreamPacket::create(const char *bytebuf) 
{ 
    return std::make_unique<OpenStreamPacket>(bytebuf);
}

void OpenStreamPacket::FillData(char *bytebuf) {
    memcpy(&bytebuf[MSG_ID_IDX], &m_messageID, MSG_ID_SIZE);
    memcpy(&bytebuf[MSG_MASK_IDX], &m_messageMask, MSG_MASK_SIZE);
    memcpy(&bytebuf[MAX_MSGS_IDX], &m_maxMessages, MAX_MSGS_SIZE);
}

/**
 * ReadMessagePacket
 */

ReadMessagePacket::ReadMessagePacket(const char *bytebuf)
    : TCPBridgeMessage(bytebuf) {
    if(IsValid())
    {
        memcpy(&m_messageID, &bytebuf[MSG_ID_IDX], MSG_ID_SIZE);
        memcpy(&m_messageMask, &bytebuf[MSG_MASK_IDX], MSG_MASK_SIZE);
    }
}

ReadMessagePacket::ReadMessagePacket(const uint32_t messageID, const uint32_t messageMask)
    : TCPBridgeMessage(READ_MSG_CMD, READ_MSG_PKT_SIZE),
    m_messageID(messageID),
    m_messageMask(messageMask)
{

}

std::unique_ptr<TCPBridgeMessage> ReadMessagePacket::create(const char *bytebuf) 
{ 
    return std::make_unique<ReadMessagePacket>(bytebuf);
}

void ReadMessagePacket::FillData(char *bytebuf) {
    memcpy(&bytebuf[MSG_ID_IDX], &m_messageID, MSG_ID_SIZE);
    memcpy(&bytebuf[MSG_MASK_IDX], &m_messageMask, MSG_MASK_SIZE);
}

/**
 * SendMessagePacket
 */

SendMessagePacket::SendMessagePacket(const char *bytebuf)
    : TCPBridgeMessage(bytebuf) {
    if(IsValid())
    {
        memcpy(&m_messageID, &bytebuf[MSG_ID_IDX], MSG_ID_SIZE);
        memcpy(&m_periodMs, &bytebuf[PERIOD_MS_IDX], PERIOD_MS_SIZE);
    }
}

SendMessagePacket::SendMessagePacket(const uint32_t messageID, const uint32_t periodMs, const uint8_t datasize, const uint8_t *data)
    : TCPBridgeMessage(SEND_MSG_CMD, SEND_MSG_PKT_SIZE),
    m_messageID(messageID),
    m_periodMs(periodMs)
{
    m_datasize = datasize > 8 ? 8 : datasize;
    memcpy(m_data, data, datasize > 8 ? 8 : m_datasize);
}

std::unique_ptr<TCPBridgeMessage> SendMessagePacket::create(const char *bytebuf) 
{ 
    return std::make_unique<SendMessagePacket>(bytebuf);
}

void SendMessagePacket::FillData(char *bytebuf) {
    memcpy(&bytebuf[MSG_ID_IDX], &m_messageID, MSG_ID_SIZE);
    memcpy(&bytebuf[PERIOD_MS_IDX], &m_periodMs, PERIOD_MS_SIZE);
    bytebuf[DATASIZE_IDX] = m_datasize;
    memcpy(&bytebuf[DATA_IDX], m_data, m_datasize);
}

/**
 * CANMessagePacket
 */

CANMessagePacket::CANMessagePacket(const char *bytebuf)
    : TCPBridgeMessage(bytebuf) {
    if(IsValid())
    {
        memcpy(&m_messageID, &bytebuf[MSG_ID_IDX], MSG_ID_SIZE);
        memcpy(&m_timestamp, &bytebuf[TIMESTAMP_IDX], TIMESTAMP_SIZE);
        memcpy(&m_datasize, &bytebuf[DATASIZE_IDX], DATASIZE_SIZE);
        memcpy(&m_data, &bytebuf[DATA_IDX], m_datasize);
    }
}

CANMessagePacket::CANMessagePacket(const uint32_t messageID, const uint32_t timestamp, const uint8_t datasize, const uint8_t *data)
    : TCPBridgeMessage(CAN_MSG_CMD, CAN_MSG_PKT_SIZE),
    m_messageID(messageID),
    m_timestamp(timestamp),
    m_datasize(datasize)
{
    memcpy(&m_data, data, m_datasize);
}

std::unique_ptr<TCPBridgeMessage> CANMessagePacket::create(const char *bytebuf) 
{ 
    return std::make_unique<CANMessagePacket>(bytebuf);
}

void CANMessagePacket::FillData(char *bytebuf) {
    memcpy(&bytebuf[MSG_ID_IDX], &m_messageID, MSG_ID_SIZE);
    memcpy(&bytebuf[TIMESTAMP_IDX], &m_timestamp, TIMESTAMP_SIZE);
    memcpy(&bytebuf[DATASIZE_IDX], &m_datasize, DATASIZE_SIZE);
    memcpy(&bytebuf[DATA_IDX], &m_data, m_datasize);
}

uint32_t CANMessagePacket::GetMessageID() const
{
    return m_messageID;
}

uint32_t CANMessagePacket::GetTimestamp() const
{
    return m_timestamp;
}

uint8_t CANMessagePacket::GetDatasize() const
{
    return m_datasize;
}

const uint8_t* CANMessagePacket::GetData() const 
{
    return m_data;
}

/**
 * InvalidPacket
 */

InvalidPacket::InvalidPacket()
    : TCPBridgeMessage(INVALID_CMD, 0) 
{

}

std::unique_ptr<TCPBridgeMessage> InvalidPacket::create() 
{ 
    return std::make_unique<InvalidPacket>();
}

void InvalidPacket::FillData(char *bytebuf) 
{

}