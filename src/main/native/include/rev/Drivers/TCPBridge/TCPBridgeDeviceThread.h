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

#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <map>
#include <vector>
#include <chrono>

// TODO: remove me
#include <clocale>
#include <iostream>
#include <iterator>
#include <iomanip>
#include <asio.hpp>

#include "rev/CANMessage.h"
#include "rev/CANBridgeUtils.h"
#include "rev/CANStatus.h"
#include "rev/Drivers/DriverDeviceThread.h"

#include "TCPBridgeMessages.h"

#include <hal/CAN.h>

namespace rev {
namespace usb {


class TCPBridgeDeviceThread : public DriverDeviceThread { 
public:
    TCPBridgeDeviceThread() =delete;
    TCPBridgeDeviceThread(const std::string host, const std::string port, long long threadIntervalMs = 1) 
        : DriverDeviceThread(0xb36c4685, threadIntervalMs), m_sock(m_ioservice)
    
    {
        if(Connect(host, port)) {
            m_run = true;
            // TODO: what if already connected?
        }
    }
    ~TCPBridgeDeviceThread()
    {
        if (m_run) {
            // disconnect socket
        }
    }

    bool Connect(const std::string host, const std::string port) {
        try
        {
            std::cout << "Resolving " << host << ":" << port << std::endl;
            asio::ip::tcp::resolver resolver(m_ioservice);
            asio::ip::tcp::resolver::query query(host, port);
            asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
            asio::ip::tcp::resolver::iterator end;

            asio::error_code error = asio::error::netdb_errors::host_not_found;
            while(error || endpoint_iterator != end) {
                m_sock.close();
                m_sock.connect(*endpoint_iterator++, error);
            }
        
            if(error)
            {
                std::cerr << "Can't connect!" << std::endl;
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

        // std::this_thread::sleep_for (std::chrono::seconds(1));
        
        m_connected = true;
        return true;
    }

    void Start() override {
        m_thread = std::thread(&TCPBridgeDeviceThread::run, this);
    }

    void OpenStream(uint32_t* handle, CANBridge_CANFilter filter, uint32_t maxSize, CANStatus *status) override {
        m_streamMutex.lock();

        *status = CANStatus::kError;
        if (m_run && m_connected) {
            // if this is the first time opening a stream
            if(!m_streamOpen) {
                serializeOpenStreamPacket(m_buf, 0, 0, 0, 100, 0);
                if(!SendMsg()) {
                    return;
                }

                if(!RecvMsg(TCPBridgeCommands::OPEN_STREAM_CMD)) {
                    return;
                }

                // there was an error opening stream on the RIO side
                if(m_msg.openStream.status != 0) {
                    printf("Error opening stream: %d\n", m_msg.openStream.status);
                    return;
                }

                // all good
                *status = CANStatus::kOk;
                *handle = m_counter++;
                m_streamOpen = true;
            }

            // Add to the map
            m_readStream[*handle] = std::unique_ptr<CANStreamHandle>(new CANStreamHandle{filter.messageId, filter.messageMask, maxSize, utils::CircularBuffer<CANMessage>{maxSize}});


        } else {
            *status = CANStatus::kError;
        }
        m_streamMutex.unlock();
    }


private:
    asio::ip::address m_ip;
    asio::io_service m_ioservice;
    asio::ip::tcp::socket m_sock;

    bool m_connected = false;
    bool m_streamOpen = false;

    TCPBridgeMessages_t m_msg;

    char m_buf[1024]; // TODO: fix

    void ReadMessages(bool &reading)  {
        // do this
        if(!m_streamOpen) {
            reading = false;
            return;
        }

        int counts = 0;
        while(RecvMsg(TCPBridgeCommands::INVALID_CMD)) {
            if(m_msg.header.commandId == CAN_MSG_CMD) {
                // m_streamMutex.lock();
                CANMessage msg(m_msg.canMessage.messageId, m_msg.canMessage.data, m_msg.canMessage.datasize, m_msg.canMessage.timestamp);
                // m_readMutex.unlock();
                m_readMutex.lock();
                if (msg.GetSize() != 0) {
                    m_readStore[m_msg.canMessage.messageId] = msg;
                }
                m_readMutex.unlock();
                
                // printf("Message %d\n=================\n", counts);
                // printf("\tMessageId: 0x%08X\n", m_msg.canMessage.messageId);
                // printf("\tTimestamp: %u\n", m_msg.canMessage.timestamp);
                // printf("\tDatasize: %u\n\n", m_msg.canMessage.datasize);

                m_streamMutex.lock();
                for (auto& stream : m_readStream) {
                    // Compare current size of the buffer to the max size of the buffer
                    if (!stream.second->messages.IsFull()
                        && rev::usb::CANBridge_ProcessMask({stream.second->messageId, stream.second->messageMask},
                        msg.GetMessageId())) {
                        stream.second->messages.Add(msg);
                    }
                }
                m_streamMutex.unlock();
                counts++;
            }
            else {
                printf("Message with command id 0x%02X found\n", m_msg.header.commandId);
            }

            if(counts > 100)
                break;
        }

        reading = false;
    }

    void WriteMessages(detail::CANThreadSendQueueElement el, std::chrono::steady_clock::time_point now) {
        // uint32_t sentMsgId = el.m_msg.GetMessageId();
        // uint16_t apiId = el.m_msg.GetApiId();
        if(!m_streamOpen)
            std::cerr << "No stream open!\n";

        std::cerr << "WriteMessages not implemented yet!" << std::endl;
    }

    bool SocketWrite(size_t sendSize)  {
        size_t sentSize = 0;
        asio::error_code ec;

        sentSize = m_sock.send(asio::buffer(m_buf, sendSize), 0, ec);
        if(ec) {
            std::cerr << "An exception occured while trying to send data - " << ec.category().name() << " - " 
                << ec.message() << std::endl;
            // need to disconnect
            return false;
        }

        if(sentSize != sendSize) {
            std::cerr << "Sent size does not match expected!" << std::endl;
            return false; 
        }

        return true;
    }

    size_t SocketRead(size_t bytesToRead, size_t bufOffset=0) {
        size_t readBytes = 0;
        asio::error_code ec, ignored_ec;
        std::chrono::milliseconds span(1000); // timeout

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
                m_connected = false;
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

    bool SendMsg() {
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

    bool RecvMsg(TCPBridgeCommands cmd) {
        size_t readLen = 0, pktLen = 0;
        asio::error_code ec;

        // resync
        while((readLen = SocketRead(1)))
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
        if(!SocketRead(sizeof(m_msg.header) - 1, 1))
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
        if(!SocketRead(m_msg.header.packetSize - sizeof(m_msg.header), 4))
        {
            std::cerr << "Couldn't read rest of packet" << std::endl;
            return false;
        }

        // printf("Rec'd %d bytes\n", m_msg.header.packetSize);
        // for(uint8_t i = 0; i < m_msg.header.packetSize; i++) {
        //     printf("\t%d: 0x%02X\n", i, *((uint8_t*)&m_msg + i));
        // }
        // printf("\n\n");

        // accept any command if INVALID_CMD is chosen
        if(cmd != INVALID_CMD) {
            if(m_msg.header.commandId != cmd)
            {
                std::cerr << "Command does not match expected" << std::endl;
                printf("Actual: 0x%02X, Expected: 0x%02X\n", m_msg.header.commandId, cmd);
                return false;
            }
        }

        if(*((uint8_t*)&m_msg + m_msg.header.packetSize - 1) != TRAILER_TOKEN) 
        {
            std::cerr << "Trailer does not match expected" << std::endl;
            return false;
        }

        return true;
    }
}; 

} // namespace usb
} // namespace rev
