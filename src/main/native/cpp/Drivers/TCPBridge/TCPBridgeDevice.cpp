/*
 * Copyright (c) 2019 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rev/Drivers/TCPBridge/TCPBridgeDevice.h"
#include "rev/Drivers/TCPBridge/TCPBridgeMessages.h"

#include <iostream> //TODO: Remove

#define __TCP_DEBUG__   0

#include <asio.hpp>
using asio::ip::tcp;

namespace rev {
namespace usb {

TCPBridgeDevice::TCPBridgeDevice(const std::string host, const std::string port)
    : m_host(host), m_port(port), m_sock(m_ioservice)
{
    m_isConnected = Connect();
}

TCPBridgeDevice::~TCPBridgeDevice()
{

}

bool TCPBridgeDevice::Connect()
{
    try
    {
        asio::ip::tcp::resolver resolver(m_ioservice);
        asio::ip::tcp::resolver::query query(m_host, m_port);
        asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        asio::ip::tcp::resolver::iterator end;

        asio::error_code error = asio::error::host_not_found;
        while(error && endpoint_iterator != end) {
            m_sock.close();
            m_sock.connect(*endpoint_iterator++, error);
        }
    
        if(error)
        {
            std::cerr << "Can't connect to host" << std::endl;
            return false;
        }

        asio::ip::tcp::no_delay option_d(true);
        m_sock.set_option(option_d);
        asio::socket_base::send_buffer_size option_s(256);
        m_sock.set_option(option_s);

    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "Exception occured while trying to connect." << std::endl;
        return false;
    }

    // convert the ip address we connected to into a string
    std::string ipStr = m_sock.remote_endpoint().address().to_string();
    m_descriptor = converter.from_bytes(ipStr + ":" + m_port);
    m_name = ipStr;

    return true;
}

std::string TCPBridgeDevice::GetName() const
{
    return m_name;
}


std::wstring TCPBridgeDevice::GetDescriptor() const
{
    return m_descriptor;
}

int TCPBridgeDevice::GetId() const
{
    return 0;
}

bool TCPBridgeDevice::Send(uint8_t* buf, size_t sendSize)
{
    size_t sentSize = 0;
    try
    {
        sentSize = m_sock.send(asio::buffer(buf, sendSize));
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "An exception occured while trying to send data" << std::endl;
        return false;
    }

    return (sentSize == sendSize);
}

size_t TCPBridgeDevice::Recv(size_t bytesToRead, size_t offset)
{
    asio::error_code error;
    size_t len = 0;
    
    try
    {
        if(bytesToRead)
            len = m_sock.read_some(asio::buffer(recbuf + offset, bytesToRead), error);
            std::future<size_t> fut = std::async([this, bytes, bufOffset, &error]{ return this->m_sock.read_some(asio::buffer(m_readBuf + bufOffset, bytes), error);});
        else
            len = m_sock.read_some(asio::buffer(recbuf + offset, MAXBUFFERSIZE), error);
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "An exception occured while trying to receive data" << std::endl;
        return 0;
    }
    
    #if __TCP_DEBUG__
    printf("%zd bytes rec'd\n", len);
    #endif

    if (error == asio::error::eof) {
        #if __TCP_DEBUG__
        std::cout << "Client disconnected" << std::endl;
        #endif
        m_isConnected = false;
    }

    return len;
}

void TCPBridgeDevice::SerializeSendMsgPacket(const CANMessage& msg, int periodMs) {
    canStreamer_sendMessage_t sendMsg;
    sendMsg.header.headerToken = HEADER_TOKEN;
    sendMsg.header.packetSize = sizeof(canStreamer_sendMessage_t);
    sendMsg.header.commandId = canstreamer_cmd::SEND_MSG_CMD;
    sendMsg.messageId = msg.GetMessageId();
    sendMsg.periodMs = periodMs;
    sendMsg.datasize = msg.GetSize();
    memcpy(sendMsg.data, msg.GetData(), msg.GetSize());
    sendMsg.trailer.trailerToken = TRAILER_TOKEN;

    memcpy(sendbuf, &sendMsg, sizeof(canStreamer_sendMessage_t));
}

CANStatus TCPBridgeDevice::SendCANMessage(const CANMessage& msg, int periodMs)
{
    #if __TCP_DEBUG__
    std::cout << "Sending CAN message..." << std::endl;
    #endif

    if(!IsConnected()) {
        std::cerr << "Error, not connected" << std::endl;
        return CANStatus::kError;
    }

    SerializeSendMsgPacket(msg, periodMs);

    return Send(sendbuf, sizeof(canStreamer_sendMessage_t)) ? CANStatus::kOk : CANStatus::kError;
}

void TCPBridgeDevice::SerializeRecieveCANMessage(uint32_t messageID, uint32_t messageMask)
{
    canStreamer_readMessage_t readMsg;
    readMsg.header.headerToken = HEADER_TOKEN;
    readMsg.header.packetSize = sizeof(canStreamer_readMessage_t);
    readMsg.header.commandId = READ_MSG_CMD;
    readMsg.messageId = messageID;
    readMsg.messageMask = messageMask;
    readMsg.trailer.trailerToken = TRAILER_TOKEN;

    memcpy(sendbuf, &readMsg, sizeof(canStreamer_readMessage_t));
}

CANStatus TCPBridgeDevice::RecieveCANMessage(CANMessage& msg, uint32_t messageID, uint32_t messageMask)
{
    #if __TCP_DEBUG__
    std::cout << "Receiving message..." << std::endl;
    #endif

    int numbytes = 0;
    uint32_t messagesRead = 0;
    SerializeRecieveCANMessage(messageID, messageMask);

    if(!Send(sendbuf, sizeof(canStreamer_readMessage_t)))
    {
        return CANStatus::kError;
    }

    // get response
    numbytes = Recv(sizeof(canStreamer_readStream_t));

    if(ParseStreamResponse(&messagesRead) != 0)
    {
        return CANStatus::kError;
    }

    #if __TCP_DEBUG__
    std::cout << "Messages read " << messagesRead << std::endl;
    #endif

    if(messagesRead == 0)
    {
        return CANStatus::kError;
    }

    numbytes = Recv();

    if(numbytes < sizeof(canStreamer_canMessage_t)) {
        return CANStatus::kError;
    }
    
    if(!ParseCANMessagePacket(msg)) {
        return CANStatus::kError;
    }

    return CANStatus::kOk;
}

bool TCPBridgeDevice::ParseCANMessagePacket(CANMessage &msg)
{
    canStreamer_canMessage_t canMsg;
    memcpy(&canMsg, &recbuf, sizeof(canStreamer_canMessage_t));
    if(canMsg.header.headerToken != HEADER_TOKEN) return false;
    if(canMsg.header.packetSize != sizeof(canStreamer_canMessage_t)) return false;
    if(canMsg.trailer.trailerToken != TRAILER_TOKEN) return false;

    msg = CANMessage(canMsg.messageId, canMsg.data, canMsg.datasize, canMsg.timestamp);

    #if __TCP_DEBUG__
    std::cout << "\tMessage ID: " << canMsg.messageID << std::endl;
    std::cout << "\tTimestamp: " << canMsg.timestamp << std::endl;
    printf("\tDatasize: 0x%02X\n", canMsg.datasize);
    for(uint8_t i = 0; i < canMsg.datasize; i++) {
        printf("\t0x%02X\n", canMsg.data[i]);
    }
    #endif

    return true;
}

void TCPBridgeDevice::SerializeOpenStreamMessage(CANBridge_CANFilter filter, uint32_t maxSize)
{
    canStreamer_openStream_t openPkt;
    openPkt.header.headerToken = HEADER_TOKEN;
    openPkt.header.packetSize = sizeof(canStreamer_openStream_t);
    openPkt.header.commandId = OPEN_STREAM_CMD;
    openPkt.messageId = filter.messageId;
    openPkt.messageMask = filter.messageMask;
    openPkt.maxMessages = maxSize;
    openPkt.trailer.trailerToken = TRAILER_TOKEN;

    memcpy(sendbuf, &openPkt, sizeof(canStreamer_openStream_t));
}

CANStatus TCPBridgeDevice::OpenStreamSession(uint32_t* sessionHandle,
                                             CANBridge_CANFilter filter,
                                             uint32_t maxSize)
{
    #if __TCP_DEBUG__
    std::cout << "Opening Stream:" << std::endl;
    std::cout << "\tMessage ID: " << filter.messageId << std::endl;
    std::cout << "\tMessage Mask: " << filter.messageMask << std::endl;
    #endif

    SerializeOpenStreamMessage(filter, maxSize);

    if(!Send(sendbuf, sizeof(canStreamer_openStream_t)))
    {
        std::cerr << "Error sending" << std::endl;
    }

    return CANStatus::kOk;
}
CANStatus TCPBridgeDevice::CloseStreamSession(uint32_t sessionHandle)
{
    std::cout << "Close stream not implemented yet..." << std::endl;

    return CANStatus::kOk;
}

int TCPBridgeDevice::CheckPacket(int *packet_size) {
    if((uint8_t)recbuf[0] != HEADER_TOKEN) {
        std::cerr << "Invalid packet rec'd" << std::endl;
        return -1;
    }

    *packet_size = recbuf[1];

    if((*packet_size > MAXBUFFERSIZE) || (recbuf[*packet_size-1] != TRAILER_TOKEN)) {
        std::cerr << "Invalid packet size rec'd: " << *packet_size-1 << std::endl;
        return -2;
    }

    return 0;
}

int TCPBridgeDevice::ParseStreamResponse(uint32_t* num_messages) {
	// int packet_size = 0;
	// if(CheckPacket(&packet_size) != 0) {
	// 	return -1;
	// }
	
	// uint32_t tmp = 0;
	// tmp = recbuf[4];
	// tmp |= (recbuf[5] << 8);
	// tmp |= (recbuf[6] << 16);
	// tmp |= (recbuf[7] << 24);
		
	// *num_messages = tmp;

    canStreamer_readStream_t rsPkt;
    memcpy(&rsPkt, &recbuf, sizeof(canStreamer_readStream_t));
    if(rsPkt.header.headerToken != HEADER_TOKEN) return false;
    if(rsPkt.header.packetSize != sizeof(canStreamer_readStream_t)) return false;
    if(rsPkt.trailer.trailerToken != TRAILER_TOKEN) return false;

    *num_messages = rsPkt.maxMessages;

	return 0;
}

void TCPBridgeDevice::SerializeReadStreamMessage(uint32_t messagesToRead)
{
    canStreamer_readStream_t readMsg;
    readMsg.header.headerToken = HEADER_TOKEN;
    readMsg.header.packetSize = sizeof(canStreamer_readStream_t);
    readMsg.header.commandId = READ_STREAM_CMD;
    readMsg.maxMessages = messagesToRead;
    readMsg.trailer.trailerToken = TRAILER_TOKEN;

    memcpy(sendbuf, &readMsg, sizeof(canStreamer_readStream_t));
}

CANStatus TCPBridgeDevice::ReadStreamSession(uint32_t sessionHandle,
                                             struct HAL_CANStreamMessage* msgs,
                                             uint32_t messagesToRead,
                                             uint32_t* messagesRead,
                                             int32_t* status)
{
    int numbytes = 0;
    #if __TCP_DEBUG__
    std::cout << "Reading stream..." << std::endl;
    #endif

    // send read stream packet
    SerializeReadStreamMessage(messagesToRead);

    if(!Send(sendbuf, sizeof(canStreamer_readStream_t)))
    {
        return CANStatus::kError;
    }

    // get response
    numbytes = Recv(sizeof(canStreamer_readStream_t));

    if(ParseStreamResponse(messagesRead) != 0) {
        return CANStatus::kError;
    }
    
    #if __TCP_DEBUG__
    std::cout << "Number of messages read: " << *messagesRead << std::endl;
    #endif

    // wait to get all messages
    int bytesRemaining = 0;
    int msg_idx = 0;
    uint32_t msgNum = 0;
    while(msgNum < *messagesRead) {
        // printf("Receiving messages...\n");
        numbytes = Recv();
        
        bytesRemaining += numbytes;
        
        while(bytesRemaining > 0) {
            memcpy(msgBuf, recbuf + msg_idx, 22);
            
            // TODO: make deserialize CAN message func
            HAL_CANStreamMessage halMsg;
            memcpy(&halMsg.messageID, &msgBuf[4], 4);
            memcpy(&halMsg.timeStamp, &msgBuf[8], 4);
            halMsg.dataSize = msgBuf[12];
            memcpy(halMsg.data, &msgBuf[13], sizeof(halMsg.data));
            
            msgs[msgNum] = halMsg;
            
            msg_idx += 22;
            bytesRemaining -= 22;
            msgNum++;
        }
        msg_idx = 0;
    }

    return CANStatus::kOk;
}

CANStatus TCPBridgeDevice::GetCANStatus()
{
    // TODO: Should this do something?
    return CANStatus::kOk;
}

bool TCPBridgeDevice::IsConnected()
{
    return m_isConnected;
}


} // namespace usb
} // namespace rev
