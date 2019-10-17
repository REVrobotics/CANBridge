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
#include "rev/Drivers/TCPBridge/TCPBridgeMessage.h"

#include <iostream>

#define __TCP_DEBUG__   0 // change to 1 to print debug messages

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
        if(m_verbosity > 0)
        {
            std::cout << "Resolving " << m_host << ":" << m_port << std::endl;
        }

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
    m_descriptor = converter.from_bytes(ipStr + ":" + m_port);
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

bool TCPBridgeDevice::WriteMessage(std::unique_ptr<TCPBridgeMessage> &msg)
{
    size_t sentSize = 0;
    asio::error_code ec;

    msg->Serialize(m_sendBuf);
    sentSize = m_sock.send(asio::buffer(m_sendBuf, msg->GetSize()), 0, ec);

    if(ec)
    {
        std::cerr << "An exception occured while trying to send data: " << ec << std::endl;
        return false;
    }

    return true;
}

std::unique_ptr<TCPBridgeMessage> TCPBridgeDevice::ReadMessage(CANStatus &stat)
{
    size_t pktLen = 0;
    TCPBridgeMessage::TCPBridgeCommands cmdType = TCPBridgeMessage::INVALID_CMD;

    // read header
    if(Read(TCPBridgeMessage::GetHeaderSize(), 0, stat))
    {
        if(TCPBridgeMessage::ParseHeader(m_readBuf, &cmdType, &pktLen))
        {
            // read the rest of the message
            if(Read(pktLen - TCPBridgeMessage::GetHeaderSize(), TCPBridgeMessage::GetHeaderSize(), stat))
            {
                switch(cmdType)
                {
                    case TCPBridgeMessage::TCPBridgeCommands::READ_STREAM_CMD:
                        // std::cout << "READ_STREAM_CMD" << std::endl;
                        return ReadStreamPacket::create(m_readBuf);
                    case TCPBridgeMessage::TCPBridgeCommands::OPEN_STREAM_CMD:
                        // std::cout << "OPEN_STREAM_CMD" << std::endl;
                        return OpenStreamPacket::create(m_readBuf);
                    case TCPBridgeMessage::TCPBridgeCommands::READ_MSG_CMD:
                        // std::cout << "READ_MSG_CMD" << std::endl;
                        return ReadMessagePacket::create(m_readBuf);
                    case TCPBridgeMessage::TCPBridgeCommands::SEND_MSG_CMD:
                        // std::cout << "SEND_MSG_CMD" << std::endl;
                        return SendMessagePacket::create(m_readBuf);
                    case TCPBridgeMessage::TCPBridgeCommands::CAN_MSG_CMD:
                        // std::cout << "CAN_MSG_CMD" << std::endl;
                        return CANMessagePacket::create(m_readBuf);
                    default:
                        #if __TCP_DEBUG__
                        std::cerr << "Unknown command type found!" << std::endl;
                        #endif
                        break;
                }
            }
        }
    }

    return InvalidPacket::create();
}

size_t TCPBridgeDevice::Read(size_t bytesToRead, size_t bufOffset, CANStatus &stat)
{
    size_t readBytes = 0;
    asio::error_code ec, ignored_ec;
    constexpr std::chrono::milliseconds span(1000);

    while(bytesToRead > 0)
    {
        std::future<size_t> fut = std::async([this, bytesToRead, bufOffset, &ec]
        { 
            return this->m_sock.read_some(asio::buffer(this->m_readBuf + bufOffset, bytesToRead), ec);
        });

    
        if(fut.wait_for(span) == std::future_status::timeout)
        {
            stat = CANStatus::kTimeout;
            m_sock.cancel(ec);
        }

        size_t tmp;
        tmp = fut.get();
        readBytes += tmp;
        bytesToRead -= tmp;
        bufOffset += tmp;

        if(ec == asio::error::basic_errors::interrupted)
        {
            stat = CANStatus::kTimeout;
            break;
        }
        else if(ec)
        {
            stat = CANStatus::kError;
            std::cout << "An error occured during call to socket read: " << ec << std::endl;
            break;
        }
    }

    return readBytes;
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

CANStatus TCPBridgeDevice::SendCANMessage(const CANMessage& msg, const int periodMs)
{
    #if __TCP_DEBUG__
    printf("sending message\n");
    printf("\tID: 0x%08X\n", msg.GetMessageId());
    printf("\tPeriod: 0x%08X\n", periodMs);
    printf("\tSize: 0x%08X\n\n", msg.GetSize());
    #endif
    std::unique_ptr<TCPBridgeMessage> sendMsg = std::unique_ptr<SendMessagePacket>(new SendMessagePacket(msg.GetMessageId(), periodMs, msg.GetSize(), msg.GetData()));
    if(!WriteMessage(sendMsg))
    {
        return CANStatus::kError;
    }
    return CANStatus::kOk;
}

CANStatus TCPBridgeDevice::RecieveCANMessage(CANMessage& msg, const uint32_t messageID, const uint32_t messageMask)
{
    // request message from server
    #if __TCP_DEBUG__
    printf("Receive message\n");
    printf("\tID: 0x%08X\n", messageID);
    printf("\tMask: 0x%08X\n", messageMask);
    #endif
    std::unique_ptr<TCPBridgeMessage> sendMsg = std::unique_ptr<ReadMessagePacket>(new ReadMessagePacket(messageID, messageMask));
    if(!WriteMessage(sendMsg))
    {
        return CANStatus::kError;
    }

    // read response
    // TCPBridgeMessage::TCPBridgeCommands cmd;
    CANStatus stat = CANStatus::kOk;
    std::unique_ptr<TCPBridgeMessage> readMsg;
    readMsg = ReadMessage(stat);

    if(readMsg->GetCommand() == TCPBridgeMessage::TCPBridgeCommands::READ_STREAM_CMD)
    {
        auto rsMsg = dynamic_cast<ReadStreamPacket&>(*readMsg);

        readMsg = ReadMessage(stat);

        if(readMsg->GetCommand() == TCPBridgeMessage::TCPBridgeCommands::CAN_MSG_CMD)
        {
            auto canMsg = dynamic_cast<CANMessagePacket&>(*readMsg);
            msg = CANMessage(canMsg.GetMessageID(), canMsg.GetData(), canMsg.GetDatasize(), canMsg.GetTimestamp());
            #if __TCP_DEBUG__
            printf("Message: %d\n", 0);
            printf("\tID: 0x%08X\n", msg.GetMessageId());
            printf("\tTimestamp: %d\n", msg.GetTimestampUs());
            printf("\tDatasize: 0x%02X\n", msg.GetSize());
            #endif
            return stat;
        }
    }

    return stat;
}

CANStatus TCPBridgeDevice::OpenStreamSession(uint32_t* sessionHandle,
                                             CANBridge_CANFilter filter,
                                             uint32_t maxSize)
{
    std::unique_ptr<TCPBridgeMessage> sendMsg = std::unique_ptr<OpenStreamPacket>(new OpenStreamPacket(filter.messageId, filter.messageMask, maxSize));
    if(!WriteMessage(sendMsg))
    {
        return CANStatus::kError;
    }
    return CANStatus::kOk;
}

CANStatus TCPBridgeDevice::CloseStreamSession(uint32_t sessionHandle)
{
    // TODO
    std::cerr << "Close stream not implemented" << std::endl;

    return CANStatus::kError;
}

CANStatus TCPBridgeDevice::ReadStreamSession(uint32_t sessionHandle,
                                             struct HAL_CANStreamMessage* msgs,
                                             uint32_t messagesToRead,
                                             uint32_t* messagesRead,
                                             int32_t* status)
{
    *messagesRead = 0;
    *status = 0; // TODO: something with this?
    std::unique_ptr<TCPBridgeMessage> rsMsg = std::unique_ptr<ReadStreamPacket>(new ReadStreamPacket(messagesToRead));

    // TCPBridgeMessage::TCPBridgeCommands cmd;
    CANStatus stat;
    std::unique_ptr<TCPBridgeMessage> msg;

    if(!WriteMessage(rsMsg))
    {
        return CANStatus::kError;
    }

    msg = ReadMessage(stat);

    if(msg->GetCommand() == TCPBridgeMessage::TCPBridgeCommands::READ_STREAM_CMD)
    {
        auto rsmsg = dynamic_cast<ReadStreamPacket&>(*msg);
        int missedMessage = 0;
        while(*messagesRead < rsmsg.GetMaxMessages())
        {
            msg = ReadMessage(stat);

            if(msg->GetCommand() == TCPBridgeMessage::TCPBridgeCommands::CAN_MSG_CMD)
            {
                auto canmsg = dynamic_cast<CANMessagePacket&>(*msg);
                HAL_CANStreamMessage halMsg;
                halMsg.messageID = canmsg.GetMessageID();
                halMsg.timeStamp = canmsg.GetTimestamp();
                halMsg.dataSize = canmsg.GetDatasize();
                memcpy(&halMsg.data, canmsg.GetData(), canmsg.GetDatasize());
                msgs[*messagesRead] = halMsg;

                #if __TCP_DEBUG__                
                printf("Message: %d\n", *messagesRead);
                printf("\tID: 0x%08X\n", halMsg.messageID);
                printf("\tTimestamp: %d\n", halMsg.timeStamp);
                printf("\tDatasize: 0x%02X\n", halMsg.dataSize);
                #endif
                (*messagesRead)++;
            }
            else 
            {
                missedMessage++;

                if(missedMessage > 10) { // TODO: this number is arbitrary
                    // std::cerr << "Too many messages missed!" << std::endl;
                    return CANStatus::kTimeout;
                }
                continue;
            }
        }
    }
    else {
        // std::cerr << "Unexpected message rec'd!" << std::endl;
        return stat;
    }

    return CANStatus::kOk;
}

CANStatus TCPBridgeDevice::GetCANStatus()
{
    // TODO: Should this do something?
    return CANStatus::kOk;
}

} // namespace usb
} // namespace rev