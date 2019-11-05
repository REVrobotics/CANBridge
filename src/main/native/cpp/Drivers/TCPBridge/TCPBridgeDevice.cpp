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

#include <iostream>

#define __TCP_DEBUG__   0 // change to 1 to print debug messages
#define __TCP_VERBOSE__ 0

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
    std::lock_guard<std::mutex> lock(sock_mutex);
    m_isConnected = false;
}

bool TCPBridgeDevice::Connect()
{
    try
    {
        // if(m_verbosity > 0)
        // {
        //     std::cout << "Resolving " << m_host << ":" << m_port << std::endl;
        // }

        asio::ip::tcp::resolver resolver(m_ioservice);
        asio::ip::tcp::resolver::query query(m_host, m_port);
        asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        asio::ip::tcp::resolver::iterator end;

        asio::error_code error = asio::error::netdb_errors::host_not_found;
        while(error || endpoint_iterator != end) {
            m_sock.close();
            m_sock.connect(*endpoint_iterator++, error);
        }
    
        if(error)
        {
            if(m_verbosity > 0)
            {
                std::cerr << "Can't connect!" << std::endl;
            }
            return false;
        }

        asio::ip::tcp::no_delay option_d(true);
        m_sock.set_option(option_d);
        asio::socket_base::send_buffer_size option_s(256); // limit this for the roboRIO, can try making larger
        m_sock.set_option(option_s);

    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "Exception occured while trying to connect to CAN server." << std::endl;
        return false;
    }

    // convert the ip address we connected to into a string
    std::string ipStr = m_sock.remote_endpoint().address().to_string();
    m_descriptor = ipStr + ":" + m_port;
    m_name = ipStr;
    m_ip = asio::ip::address::from_string(ipStr);

    if(m_verbosity > 0)
    {
        std::cout << "Connected to " << ipStr << ":" << m_port << std::endl;
    }

    std::this_thread::sleep_for (std::chrono::seconds(1));
    
    return true;
}

bool TCPBridgeDevice::IsConnected()
{
    return m_isConnected;
}

void TCPBridgeDevice::Disconnect()
{
    if(!m_isConnected) return;
    // asio::error_code ec;
    // m_sock.close(ec);
    // if(ec)
    // {
    //     std::cerr << "Error closing socket: " << ec.category().name() << " - " 
    //         << ec.message() << std::endl;
    // }
}

bool TCPBridgeDevice::SendMsg()
{
    size_t sentSize = 0;
    asio::error_code ec;

    sentSize = m_sock.send(asio::buffer(&m_msg, TCPBridgePacketSizes[m_msg.header.commandId]), 0, ec);
    if(ec)
    {
        std::cerr << "An exception occured while trying to send data - " << ec.category().name() << " - " 
            << ec.message() << std::endl;
        return false;
    }

    // printf("Sending %d bytes\n", TCPBridgePacketSizes[m_msg.header.commandId]);
    // for(uint8_t i = 0; i < TCPBridgePacketSizes[m_msg.header.commandId]; i++) {
    //     printf("\t%d: 0x%02X\n", i, *((uint8_t*)&m_msg + i));
    // }
    // printf("\n\n");

    if(sentSize != TCPBridgePacketSizes[m_msg.header.commandId])
    {
        std::cerr << "Sent size does not match expected!" << std::endl;
        return false; 
    }

    return true;
}

bool TCPBridgeDevice::RecvMsg(TCPBridgeCommands cmd)
{
    size_t readLen = 0, pktLen = 0;
    asio::error_code ec;

    // resync
    while((readLen = Read(1)))
    {
        if(m_msg.header.headerToken == HEADER_TOKEN)
        {
            break;
        }
    }

    if(readLen == 0) 
    {
        std::cerr << "Got nothing for header token..." << std::endl;
        return false;
    }

    // read rest of header
    if(!Read(sizeof(m_msg.header) - 1, 1))
    {
        std::cerr << "Couldn't read rest of header..." << std::endl;
        return false;
    }

    // check header
    if(m_msg.header.commandId >= INVALID_CMD)
    {
        std::cerr << "Invalid command rec'd..." << std::endl;
        return false;
    }

    if(m_msg.header.packetSize != TCPBridgePacketSizes[m_msg.header.commandId])
    {
        std::cerr << "Packet size does not match expected..." << std::endl;
        return false;
    }

    // read rest of packet
    if(!Read(m_msg.header.packetSize - sizeof(m_msg.header), 4))
    {
        std::cerr << "Couldn't read rest of packet" << std::endl;
        return false;
    }

    // printf("Rec'd %d bytes\n", m_msg.header.packetSize);
    // for(uint8_t i = 0; i < m_msg.header.packetSize; i++) {
    //     printf("\t%d: 0x%02X\n", i, *((uint8_t*)&m_msg + i));
    // }
    // printf("\n\n");

    if(m_msg.header.commandId != cmd)
    {
        std::cerr << "Command does not match expected" << std::endl;
        printf("Actual: 0x%02X, Expected: 0x%02X\n", m_msg.header.commandId, cmd);
        return false;
    }

    if(*((uint8_t*)&m_msg + m_msg.header.packetSize - 1) != TRAILER_TOKEN) 
    {
        std::cerr << "Trailer does not match expected" << std::endl;
        return false;
    }

    return true;
}

size_t TCPBridgeDevice::Read(size_t bytesToRead, size_t bufOffset)
{
    size_t readBytes = 0;
    asio::error_code ec, ignored_ec;
    std::chrono::milliseconds span(1000);

    
    while(bytesToRead > 0)
    {
        std::future<size_t> fut = std::async([this, bytesToRead, bufOffset, &ec]
        { 
            return this->m_sock.read_some(
                asio::buffer((char*)&this->m_msg + bufOffset, bytesToRead), ec);
        });

    
        if(fut.wait_for(span) == std::future_status::timeout)
        {
            std::cerr << "\nRead from socket timed out" << std::endl;
            m_sock.close(ignored_ec);
            m_isConnected = false;
            return 0;
        }

        size_t tmp;
        tmp = fut.get();
        readBytes += tmp;
        bytesToRead -= tmp;
        bufOffset += tmp;

        if(ec)
        {
            std::cerr << "Error raised during socket read: " << 
                ec.category().name() << " : " << ec.message() << std::endl;
            return 0;
        }
    }

    return readBytes;
}

std::string TCPBridgeDevice::GetName() const
{
    return m_name;
}

std::string TCPBridgeDevice::GetDescriptor() const
{
    return m_descriptor;
}

int TCPBridgeDevice::GetId() const
{
    return 0;
}

CANStatus TCPBridgeDevice::SendCANMessage(const CANMessage& msg, int periodMs)
{
    if(!m_isConnected) {
        std::cerr << "Aborting send - no longer connected!\n";
        return CANStatus::kError;
    }
    std::lock_guard<std::mutex> lock(sock_mutex);
    int32_t status = 0;
    #if __TCP_DEBUG__
    printf("Send message\n");
    printf("\tMessage Mask: 0x%08X\n", msg.GetMessageId());
    printf("\tsize: %u\n", msg.GetSize());
    printf("\tperiodMs: %d\n", periodMs);
    #if __TCP_VERBOSE__
    const uint8_t *data = msg.GetData();
    for(uint8_t i = 0; i < msg.GetSize(); i++)
        printf("\t%d: 0x%02X\n", i, data[i]);
    #endif
    #endif
    SendMessage(msg.GetMessageId(), msg.GetData(), msg.GetSize(), periodMs, &status);
    #if __TCP_DEBUG__
    printf("\tstatus: %d\n", status);
    #endif
    return status == 0 ? CANStatus::kOk : CANStatus::kError;
}

CANStatus TCPBridgeDevice::RecieveCANMessage(CANMessage& msg, uint32_t messageID, uint32_t messageMask)
{
    if(!m_isConnected) {
        std::cerr << "Aborting receive - no longer connected!\n";
        return CANStatus::kError;
    }
    std::lock_guard<std::mutex> lock(sock_mutex);
    int32_t status = 0;
    uint8_t dataSize = 0;
    uint32_t timeStamp = 0;
    uint8_t data[8];
    #if __TCP_DEBUG__
    printf("Receiving CAN Message\n");
    printf("\tMessage Mask: 0x%08X\n", messageMask);
    #endif
    ReceiveMessage(&messageID, messageMask, data, &dataSize, &timeStamp, &status);
    if(status == 0)
    {
        CANMessage newMsg{messageID, data, dataSize, timeStamp};
        msg = newMsg;
        #if __TCP_DEBUG__
        printf("\tID: 0x%08X\n", msg.GetMessageId());
        printf("\tstatus: %d\n", status);
        #endif
        return CANStatus::kOk;
    }
    else
    {
        #if __TCP_DEBUG__
        printf("\tstatus: %d\n", status);
        #endif
        return CANStatus::kTimeout;
    }
}

CANStatus TCPBridgeDevice::OpenStreamSession(uint32_t* sessionHandle, CANBridge_CANFilter filter, uint32_t maxSize)
{
    if(!m_isConnected) {
        std::cerr << "Aborting open - no longer connected!\n";
        return CANStatus::kError;
    }
    std::lock_guard<std::mutex> lock(sock_mutex);
    int32_t status = 0;
    #if __TCP_DEBUG__
    printf("Opening stream session\n");
    printf("\tID: 0x%08X\n", filter.messageId);
    printf("\tMask: 0x%08X\n", filter.messageMask);
    printf("\tMax: %u\n", maxSize);
    #endif
    OpenStreamSession(sessionHandle, filter.messageId, filter.messageMask, maxSize, &status);
    #if __TCP_DEBUG__
    printf("\tStatus: %d\n", status);
    printf("\tHandle: %u\n", *sessionHandle);
    #endif
    return status == 0 ? CANStatus::kOk : CANStatus::kError;
}

CANStatus TCPBridgeDevice::CloseStreamSession(uint32_t handle) {
    if(!m_isConnected) {
        std::cerr << "Aborting close - no longer connected!\n";
        return CANStatus::kError;
    }
    std::lock_guard<std::mutex> lock(sock_mutex);
    #if __TCP_DEBUG__
    printf("Closing stream session\n");
    printf("\tHandle: %u\n", handle);
    #endif
    serializeCloseStreamPacket(&m_msg, handle);
    
    if(!SendMsg())
        return CANStatus::kError;

    if(!RecvMsg(CLOSE_STREAM_CMD))
        return CANStatus::kError;

    return CANStatus::kOk;
}

CANStatus TCPBridgeDevice::GetCANStatus(float *percentBusUtilization,
                                   uint32_t *busOffCount,
                                   uint32_t *txFullCount,
                                   uint32_t *receiveErrorCount,
                                   uint32_t *transmitErrorCount,
                                   int32_t *status) {
    if(!m_isConnected) {
        std::cerr << "Aborting get status - no longer connected!\n";
        return CANStatus::kError;
    }
    std::lock_guard<std::mutex> lock(sock_mutex);
    serializeBusUtilPacket(&m_msg, *percentBusUtilization, *busOffCount,  *txFullCount,  *receiveErrorCount,  *transmitErrorCount, *status);
    if(!SendMsg())
        return CANStatus::kError;

    if(!RecvMsg(BUS_UTIL_CMD))
        return CANStatus::kError;

    *percentBusUtilization = m_msg.busUtil.utilization;
    *busOffCount = m_msg.busUtil.busOff;
    *txFullCount = m_msg.busUtil.txFull;
    *receiveErrorCount = m_msg.busUtil.receiveErr;
    *transmitErrorCount = m_msg.busUtil.transmitErr;
    *status = m_msg.busUtil.status;

    return CANStatus::kOk;
}

// Open a stream with the given parameters
void TCPBridgeDevice::OpenStreamSession(uint32_t *handle, 
                                          uint32_t messageId, 
                                          uint32_t messageMask, 
                                          uint32_t maxMessages, 
                                          int32_t *status) {
    serializeOpenStreamPacket(&m_msg, *handle, messageId, messageMask, maxMessages, *status);
    if(!SendMsg())
    {
        *status = -1;
        return;
    }
    
    if(!RecvMsg(OPEN_STREAM_CMD))
    {
        *status = -1;
        return;
    }

    *handle = m_msg.openStream.handle;
    *status = m_msg.openStream.status;
}

CANStatus TCPBridgeDevice::ReadStreamSession(uint32_t sessionHandle,
                                             struct HAL_CANStreamMessage *messages,
                                             uint32_t messagesToRead,
                                             uint32_t *messagesRead,
                                             int32_t *status) {
    if(!m_isConnected) {
        std::cerr << "Aborting read - no longer connected!\n";
        return CANStatus::kError;
    }
    std::lock_guard<std::mutex> lock(sock_mutex);
    #if __TCP_DEBUG__
    printf("Reading stream session\n");
    printf("\tHandle: %u\n", sessionHandle);
    printf("\tMessages to read: %u\n", messagesToRead);
    #endif
    serializeReadStreamPacket(&m_msg, sessionHandle, messagesToRead, *messagesRead, *status);

    if(!SendMsg())
    {
        *status = -1;
        return CANStatus::kError;
    }

    if(!RecvMsg(READ_STREAM_CMD))
    {
        *status = -1;
        return CANStatus::kError;
    }

    #if __TCP_DEBUG__
    printf("\tMessages read: %u\n", m_msg.readStream.messagesRead);
    printf("\tStatus: %d\n\n", m_msg.readStream.status);
    #endif

    *messagesRead = m_msg.readStream.messagesRead;
    *status = m_msg.readStream.status;
    for(uint32_t i = 0; i < *messagesRead; i++)
    {
        if(RecvMsg(CAN_MSG_CMD))
        {
            messages[i].messageID = m_msg.canMessage.messageId;
            messages[i].timeStamp = m_msg.canMessage.timestamp;
            messages[i].dataSize = m_msg.canMessage.datasize;
            memcpy(messages[i].data, m_msg.canMessage.data, 8);

            #if __TCP_DEBUG__
            printf("Message %u of %u\n=================\n", i, *messagesRead);
            printf("\tMessageId: 0x%08X\n", messages[i].messageID);
            printf("\tTimestamp: %u\n", messages[i].timeStamp);
            printf("\tDatasize: %u\n\n", messages[i].dataSize);
            #if __TCP_VERBOSE__
            for(uint8_t j = 0; j < messages[i].dataSize; j++)
                printf("\t%d: 0x%02X\n", j, messages[i].data[j]);
            #endif
            printf("\n\n");
            #endif
        }
        else
        {
            return CANStatus::kError;
        }
    }

    return CANStatus::kOk;
}


void TCPBridgeDevice::ReceiveMessage(uint32_t *messageId,
                                     uint32_t messageMask,
                                     uint8_t *data,
                                     uint8_t *dataSize,
                                     uint32_t *timeStamp,
                                     int32_t *status) {
    serializeReceiveMessagePacket(&m_msg, *messageId, messageMask, data, *dataSize, *timeStamp, *status);
    
    if(!SendMsg())
    {
        *status = -1;
        return;
    }

    if(!RecvMsg(READ_MSG_CMD))
    {
        *status = -1;
        return;
    }

    *messageId = m_msg.readMessage.messageId;
    *dataSize = m_msg.readMessage.dataSize;
    *timeStamp = m_msg.readMessage.timestamp;
    *status = m_msg.readMessage.status;
    memcpy(data, m_msg.readMessage.data, *dataSize > 8 ? 8 : *dataSize);
}

void TCPBridgeDevice::SendMessage(uint32_t messageId, 
                                    const uint8_t *data, 
                                    uint8_t dataSize, 
                                    int32_t periodMs, 
                                    int32_t *status) {
    serializeSendMessagePacket(&m_msg, messageId, periodMs, dataSize, data, *status);
    SendMsg();
    RecvMsg(SEND_MSG_CMD);

    *status = m_msg.sendMessage.status;
}

} // namespace usb
} // namespace rev