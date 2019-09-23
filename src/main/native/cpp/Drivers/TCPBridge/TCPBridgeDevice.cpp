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
#include "rev/Drivers/TCPBridge/TCPBridgePackets.h"

#include <iostream> //TODO: Remove

#define __TCP_DEBUG__   0

#include <asio.hpp>
using asio::ip::tcp;

#define DEFAULT_PORT "8800"

namespace rev {
namespace usb {

TCPBridgeDevice::TCPBridgeDevice(const std::string ip, const unsigned short port)
    : m_ip(asio::ip::address::from_string(ip)), m_port(port), m_sock(m_ioservice)
{
    m_isConnected = Connect();
}

TCPBridgeDevice::~TCPBridgeDevice()
{
    // std::cout << "Closing device" << std::endl;
}

bool TCPBridgeDevice::Connect()
{
    std::string ipStr = m_ip.to_string();
    #if __TCP_DEBUG__
    std::cout << "Connecting to " << ipStr << ":" << m_port << std::endl;
    #endif
    try
    {
        asio::ip::tcp::endpoint endpoint(m_ip, m_port);

        asio::error_code error = asio::error::host_not_found;
        m_sock.connect(endpoint, error);
    
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
    
    m_descriptor = converter.from_bytes(ipStr + ":" + std::to_string(m_port));
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

size_t TCPBridgeDevice::Recv()
{
    asio::error_code error;
    size_t len = 0;
    
    try
    {
        len = m_sock.read_some(asio::buffer(recbuf), error);
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

    // for(size_t i = 0; i < len; i++)
    //     printf("\t%3zd: 0x%02X\n", i, recbuf[i]);

    if (error == asio::error::eof) {
        #if __TCP_DEBUG__
        std::cout << "Client disconnected" << std::endl;
        #endif
        m_isConnected = false;
    }

    return len;
}

void TCPBridgeDevice::SerializeSendMsgPacket(const CANMessage& msg, int periodMs) {
	sendbuf[0] = HEADER_TOKEN;
	sendbuf[1] = 22;
	sendbuf[2] = SEND_MSG;
	sendbuf[3] = 0;
	
    uint32_t messageID = msg.GetMessageId();
    memcpy(&sendbuf[4], &messageID, 4);
    memcpy(&sendbuf[8], &periodMs, 4);
	sendbuf[12] = msg.GetSize();
	memcpy(sendbuf + 13, msg.GetData(), msg.GetSize());
	
	sendbuf[21] = TRAILER_TOKEN;
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

    return Send(sendbuf, 22) ? CANStatus::kOk : CANStatus::kError;
}

void TCPBridgeDevice::SerializeRecieveCANMessage(uint32_t messageID, uint32_t messageMask)
{
    sendbuf[0] = HEADER_TOKEN;
	sendbuf[1] = 14;
	sendbuf[2] = RCV_MSG;
	sendbuf[3] = 0;
	
    memcpy(&sendbuf[4], &messageID, 4);
    memcpy(&sendbuf[8], &messageMask, 4);
	sendbuf[12] = 8;
	
	sendbuf[13] = TRAILER_TOKEN;
}

CANStatus TCPBridgeDevice::RecieveCANMessage(CANMessage& msg, uint32_t messageID, uint32_t messageMask)
{
    #if __TCP_DEBUG__
    std::cout << "Receiving message..." << std::endl;
    #endif

    int numbytes = 0;
    uint32_t messagesRead = 0;
    SerializeRecieveCANMessage(messageID, messageMask);

    if(!Send(sendbuf, 14))
    {
        return CANStatus::kError;
    }

    // get response
    numbytes = Recv();

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

    int bytesRemaining = 0;
    int msg_idx = 0;
    uint32_t msgNum = 0;
    
    // TODO: make deserialize CAN message func
    uint8_t tmp[8];
    uint32_t timestamp;
    uint8_t datasize;
    memcpy(&messageID, &recbuf[0], 4);
    memcpy(&timestamp, &recbuf[4], 4);
    datasize = recbuf[8];
    memcpy(tmp, &recbuf[9], 8);
    msg = CANMessage(messageID, tmp, datasize, timestamp);

    #if __TCP_DEBUG__
    std::cout << "\tMessage ID: " << messageID << std::endl;
    std::cout << "\tTimestamp: " << timestamp << std::endl;
    // std::cout << "\tDatasize: " << datasize << std::endl;
    printf("\tDatasize: 0x%02X\n", datasize);
    for(uint8_t i = 0; i < datasize; i++) {
        printf("\t0x%02X\n", recbuf[9 + i]);
    }
    #endif

    return CANStatus::kOk;
}

void TCPBridgeDevice::SerializeOpenStreamMessage(CANBridge_CANFilter filter, uint32_t maxSize)
{
    sendbuf[0] = HEADER_TOKEN;
	sendbuf[1] = 17;
	sendbuf[2] = OPEN_STREAM;
	sendbuf[3] = 0x00;

    memcpy(&sendbuf[4], &filter.messageId, 4);
    memcpy(&sendbuf[8], &filter.messageMask, 4);
    memcpy(&sendbuf[12], &maxSize, 4);
	
	sendbuf[16] = TRAILER_TOKEN;
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

    if(!Send(sendbuf, 17))
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
	int packet_size = 0;
	if(CheckPacket(&packet_size) != 0) {
		return -1;
	}
	
	uint32_t tmp = 0;
	tmp = recbuf[4];
	tmp |= (recbuf[5] << 8);
	tmp |= (recbuf[6] << 16);
	tmp |= (recbuf[7] << 24);
		
	*num_messages = tmp;
	return 0;
}

void TCPBridgeDevice::SerializeReadStreamMessage(uint32_t messagesToRead)
{
    sendbuf[0] = HEADER_TOKEN;
	sendbuf[1] = 9; // size
	sendbuf[2] = READ_STREAM;
	sendbuf[3] = 0x00;
	
	memcpy(&sendbuf[4], &messagesToRead, 4);
	sendbuf[8] = TRAILER_TOKEN;
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

    if(!Send(sendbuf, 9))
    {
        return CANStatus::kError;
    }

    // // get response
    numbytes = Recv();

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
            memcpy(msgBuf, recbuf + msg_idx, 17);
            
            // TODO: make deserialize CAN message func
            HAL_CANStreamMessage halMsg;
            memcpy(&halMsg.messageID, &msgBuf[0], 4);
            memcpy(&halMsg.timeStamp, &msgBuf[4], 4);
            halMsg.dataSize = msgBuf[8];
            memcpy(halMsg.data, &msgBuf[9], sizeof(halMsg.data));
            
            msgs[msgNum] = halMsg;
            
            msg_idx += 17;
            bytesRemaining -= 17;
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
