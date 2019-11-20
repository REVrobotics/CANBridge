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

#include "rev/CANMessage.h"
#include "rev/CANBridgeUtils.h"
#include "rev/CANStatus.h"
#include "rev/Drivers/DriverDeviceThread.h"

#include "SerialMessage.h"

#include "serial/serial.h"

#include <mockdata/CanData.h>
#include <hal/CAN.h>

namespace rev {
namespace usb {


class SerialDeviceThread : public DriverDeviceThread { 
public:
    SerialDeviceThread() =delete;
    SerialDeviceThread(std::string port, long long threadIntervalMs = 1) : DriverDeviceThread(0xa45b5597, threadIntervalMs)
    {
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1);
        m_device.setPort(port);
        m_device.setBaudrate(115200);
        m_device.setTimeout(timeout);
        m_device.setBytesize(serial::bytesize_t::eightbits);
        m_device.setParity(serial::parity_t::parity_even);
        m_device.setStopbits(serial::stopbits_t::stopbits_one);
        m_device.setFlowcontrol(serial::flowcontrol_t::flowcontrol_none);

        try {
            if (!m_device.isOpen()) {
                m_device.open();
                m_run = true;
            } else {
                std::cout << port << " already open" << std::endl;
            }
        } catch(const std::exception& e) {
            e.what();
            m_run = false;
        }
   
    }
    ~SerialDeviceThread()
    {
        if (m_run) {
            m_device.close();
        }
    }

    void Start() override {
        m_thread = std::thread(&SerialDeviceThread::run, this);
    }

    void OpenStream(uint32_t* handle, CANBridge_CANFilter filter, uint32_t maxSize, CANStatus *status) override {
        m_streamMutex.lock();

        if (m_run && m_device.isOpen()) {
            // Create the handle
            *handle = m_counter++;
            
            // uint8_t buffer[bufferSize] = {0x02, 0x05, 0x1a, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
            // std::cout << "stream write >> ";
            // for (int i = 0; i < bufferSize; i++ ) {
            //     std::cout << std::hex << (int)buffer[i];
            // }
            // std::cout << "\n";
            // m_device.write(buffer, bufferSize);

            uint32_t msgId = 0x2051A80;
            uint8_t dataBuffer[8] = {0};

            auto now = std::chrono::steady_clock::now();

            WriteMessages(rev::usb::detail::CANThreadSendQueueElement(rev::usb::CANMessage(msgId, dataBuffer, 0, 0), 0), now);
            
            // Add to the map
            m_readStream[*handle] = std::unique_ptr<CANStreamHandle>(new CANStreamHandle{filter.messageId, filter.messageMask, maxSize, utils::CircularBuffer<std::shared_ptr<CANMessage>>{maxSize}});
        } else {
            *status = CANStatus::kError;
        }
        m_streamMutex.unlock();
    }


private:
    serial::Serial m_device;

    static const size_t bufferSize = 12;

    void ReadMessages(bool &reading)  {
        uint8_t data[bufferSize] = {0}; // 12 bytes of data
        try {
            size_t bytesRead = m_device.read(data, bufferSize);

            if (bytesRead == bufferSize) {

                std::stringstream msgStream;
                std::stringstream devStream;
                uint32_t msgId;
                uint32_t devId;

                // Gets the full message ID
                msgStream << "0x" << std::setfill('0') << std::setw(2) << std::hex << (int)data[3] << std::setfill('0') << std::setw(2) 
                            << std::hex << (int)data[2] << std::setfill('0') << std::setw(2) << std::hex << (int)data[1]
                            << std::setfill('0') << std::setw(2) << std::hex << (int)data[0];        
                msgStream >> msgId;

                // Gets just the manufacturer and device type IDs
                devStream << "0x" << std::setfill('0') << std::setw(2) << std::hex << (int)data[3] << std::setfill('0') << std::setw(2) 
                            << std::hex << (int)data[2] << "0000";
                devStream >> devId;

                // Make sure message ID isn't empty
                if (msgId != 0x0 ) {
                    // std::cout << "read >> " << std::hex << msgId << "  " << devId << std::endl;
                    uint8_t msgData[8];
                    // Check if parameter access
                    if (IsLegacyGetParam(msgId)) {
                        memcpy(msgData, data+6, 6*sizeof(uint8_t));
                        msgId = devId | ((CMD_API_PARAM_ACCESS | data[4]) << 6) | (msgId & 0x3F);
                    } else {
                        memcpy(msgData, data+4, 8*sizeof(uint8_t));
                    }

                    auto msg = std::make_shared<CANMessage>(msgId, msgData, 8);
                    
                    m_readMutex.lock();
                    if (msg->GetSize() != 0) {
                        m_readStore[msgId] = msg;
                    }
                    m_readMutex.unlock();

                    m_streamMutex.lock();
                    for (auto& stream : m_readStream) {
                        // Compare current size of the buffer to the max size of the buffer
                        if (!stream.second->messages.IsFull()
                            && rev::usb::CANBridge_ProcessMask({stream.second->messageId, stream.second->messageMask},
                            msg->GetMessageId())) {
                            stream.second->messages.Add(msg);
                        }
                    }
                    m_streamMutex.unlock();

                    reading = false;
                }
            } else {
                reading = false;
            }
        } catch(const std::exception& e) {
            std::cout << e.what() << std::endl;
        }
    }

    void WriteMessages(detail::CANThreadSendQueueElement el, std::chrono::steady_clock::time_point now) {
        uint32_t sentMsgId = el.m_msg.GetMessageId();
        uint16_t apiId = el.m_msg.GetApiId();
        // std::cout << "serial write >> " << std::hex << sentMsgId << std::endl;

        if ((el.m_intervalMs == 0 || now - el.m_prevTimestamp >= std::chrono::milliseconds(el.m_intervalMs)) && (IsValidSerialMessageId(apiId) || IsConfigParameter(apiId))) {
            // Little endian
            uint8_t idBuffer[4];
            uint8_t dataBuffer[8];

            idBuffer[0] = (sentMsgId & 0x000000ff);
            idBuffer[2] = (sentMsgId & 0x00ff0000) >> 16;
            idBuffer[3] = (sentMsgId & 0xff000000) >> 24;

            // Check to see if it's parameter access
            if (IsConfigParameter(apiId)) {
                uint32_t paramConfig;
                // If the message has empty data, then it's getting parameter value. Otherwise setting parameter value
                if (el.m_msg.IsEmpty()) {
                    paramConfig = GET_CONFIG_PARAM;
                } else {
                    paramConfig = SET_CONFIG_PARAM;
                }

                // Need to change the id to have the right parameter access value
                idBuffer[0] = (paramConfig & 0x000000ff);
                idBuffer[1] = (paramConfig & 0x0000ff00) >> 8;

                // Only 5-6 bytes of data for parameter access, leave first 2 spots free for id and buffer 0
                memcpy(dataBuffer + 2, el.m_msg.GetData(), sizeof(uint8_t)*6);

                // First data byte needs to be the parameter id, second one always needs to be a zero
                dataBuffer[0] = CMD_API_PARAM_ACCESS | apiId; // needs to be the paramter id
                dataBuffer[1] = 0;
            } else { 
                // If not parameter access, leave api ID as is 
                idBuffer[1] = (sentMsgId & 0x0000ff00) >> 8;
                memcpy(dataBuffer, el.m_msg.GetData(), sizeof(uint8_t)*8);   
            }

            uint8_t buffer[bufferSize];
            std::copy(dataBuffer, dataBuffer + 8, std::copy(idBuffer, idBuffer + 4, buffer));
            // std::cout << "write >> ";
            // for (int i = 0; i < bufferSize; i++ ) {
            //     std::cout << std::hex << (int)buffer[i];
            // }
            // std::cout << "\n";

            size_t bytesWritten = m_device.write(buffer, bufferSize);

            // std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if (bytesWritten != bufferSize) {
                std::cout << "Failed to send message, wrote " << bytesWritten << " bytes of data." << std::endl;
            } else {
                // std::cout << "Message sent" << std::endl;
            }
        }
    }
}; 

} // namespace usb
} // namespace rev
