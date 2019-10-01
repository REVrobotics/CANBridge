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

#include "SerialMessage.h"

#include "serial/serial.h"

#include <mockdata/CanData.h>
#include <hal/CAN.h>

namespace rev {
namespace usb {


class SerialDeviceThread { 
public:
    SerialDeviceThread() =delete;
    SerialDeviceThread(std::string port, long long threadIntervalMs = 1) : 
        m_threadComplete(false),
        m_threadIntervalMs(threadIntervalMs)
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
            } else {
                std::cout << "COM port " << port << " already open" << std::endl;
            }
        } catch(const std::exception& e) {
            std::cout << e.what() << std::endl;
            throw "Failed to open device!";
        }
   
    }
    ~SerialDeviceThread()
    {
        m_device.close();
    }

    void Start() {
        //std::cout << "Starting Thread..." << std::endl;
        m_thread = std::thread (&SerialDeviceThread::run, this);
    }

    void Stop() {
        //std::cout << "Stopping Thread..." << std::endl;
        m_threadComplete = true;
        if (m_thread.joinable()) {
            m_thread.join();
        }
        //std::cout << "Thread Stopped." << std::endl;
    }

    bool EnqueueMessage(const CANMessage& msg, int32_t timeIntervalMs) {
        m_sendMutex.lock();
        m_sendQueue.push(detail::CANThreadSendQueueElement(msg, timeIntervalMs));
        m_sendMutex.unlock();

        // TODO: Limit the max queue size
        return true;
    }

    bool RecieveMessage(std::map<uint32_t, CANMessage>& recvMap) {
        // This needs to return all messages with the id, the it will handle which ones pass the mask
        m_recvMutex.lock();
        recvMap = m_recvStore;
        m_recvMutex.unlock();

        return true;
    }

    void OpenStream(uint32_t* handle, CANBridge_CANFilter filter, uint32_t maxSize, CANStatus *status) {
        m_streamMutex.lock();

        if (m_device.isOpen()) {
            // Create the handle
            *handle = counter++;
            
            uint8_t buffer[bufferSize] = {0x00, 0xc0, 0x05, 0x02, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

            m_device.write(buffer, bufferSize);

            // std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // Add to the map
            m_recvStream[*handle] = std::unique_ptr<CANStreamHandle>(new CANStreamHandle{filter.messageId, filter.messageMask, maxSize, utils::CircularBuffer<CANMessage>{maxSize}});
        } else {
            *status = CANStatus::kError;
        }
        
        m_streamMutex.unlock();
    }

    void CloseStream(uint32_t handle) {
        m_streamMutex.lock();
        m_recvStream.erase(handle);
        m_streamMutex.unlock();
    }

    // Return a vector of the messages, because pointer can't point to empty vector
    // Use a circular buffer instead of just the vector
    void ReadStream(uint32_t handle, struct HAL_CANStreamMessage* messages, uint32_t messagesToRead, uint32_t* messagesRead) {
        m_streamMutex.lock();
        *messagesRead = m_recvStream[handle]->messages.GetCount(); // first before remove
        for (uint32_t i = 0; i < *messagesRead; i++) {
            CANMessage m;
            if (m_recvStream[handle]->messages.Remove(m)) {
                messages[i] = ConvertCANRecieveToHALMessage(m);
            }
        }
        m_streamMutex.unlock();
        
    }

    static HAL_CANStreamMessage ConvertCANRecieveToHALMessage(rev::usb::CANMessage msg)
    {
        HAL_CANStreamMessage halMsg;
        halMsg.timeStamp = msg.GetTimestampUs();
        halMsg.messageID = msg.GetMessageId();
        halMsg.dataSize = msg.GetSize();
        memcpy(halMsg.data, msg.GetData(), sizeof(halMsg.data));
        return halMsg;
    }



private:
    serial::Serial m_device;
    
    std::atomic_bool m_threadComplete;
    std::thread m_thread;
    std::mutex m_sendMutex;
    std::mutex m_recvMutex;
    std::mutex m_streamMutex;

    uint32_t counter = 0xe45b5597;

    std::queue<detail::CANThreadSendQueueElement> m_sendQueue;
    std::map<uint32_t, CANMessage> m_recvStore;
    std::map<uint32_t, std::unique_ptr<CANStreamHandle>> m_recvStream; // (id, mask), max size, message buffer

    long long m_threadIntervalMs;

    static const size_t bufferSize = 12;


    void run() {

        while (m_threadComplete == false) {
            auto sleepTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(m_threadIntervalMs);
            
            // 1) Handle all recieved CAN traffic
            bool reading = true;
            m_device.flush();
            while (reading) {
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

                            uint8_t msgData[8];
                            // Check if parameter access
                            if (IsLegacyGetParam(msgId)) {
                                memcpy(msgData, data+6, 6*sizeof(uint8_t));
                                msgId = devId | ((CMD_API_PARAM_ACCESS | data[4]) << 6) | (msgId & 0x3F);
                            } else {
                                memcpy(msgData, data+4, 8*sizeof(uint8_t));
                            }

                            // std::cout << std::hex << ">> r msg ids: " <<  msgId <<  std::dec << std::endl;

                            CANMessage msg(msgId, msgData, 8);
                            
                            m_recvMutex.lock();
                            if (msg.GetSize() != 0) {
                                m_recvStore[msgId] = msg;
                            }
                            m_recvMutex.unlock();

                            m_streamMutex.lock();
                            for (auto& stream : m_recvStream) {
                                // Compare current size of the buffer to the max size of the buffer
                                if (!stream.second->messages.IsFull()
                                    && rev::usb::CANBridge_ProcessMask({stream.second->messageId, stream.second->messageMask},
                                    msg.GetMessageId())) {
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

            // 2) Schedule CANMessage queue
            m_sendMutex.lock();
            size_t queueSize = m_sendQueue.size();

            for (size_t i=0;i<queueSize;i++) {
                detail::CANThreadSendQueueElement el = m_sendQueue.front();
                m_sendQueue.pop();
                if (el.m_intervalMs == -1) {
                    continue;
                }

                auto now = std::chrono::steady_clock::now();
                uint32_t sentMsgId = el.m_msg.GetMessageId();
                uint16_t apiId = el.m_msg.GetApiId();

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

                    size_t bytesWritten = m_device.write(buffer, bufferSize);

                    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    if (bytesWritten != bufferSize) {
                        std::cout << "Failed to send message, wrote " << bytesWritten << " bytes of data." << std::endl;
                    } else {
                        // std::cout << "Message sent" << std::endl;
                    }
                }

                // Return to queue if repeated
                if (el.m_intervalMs > 0 ) {
                    el.m_prevTimestamp = now;
                    m_sendQueue.push(el);
                }
            }

            m_sendMutex.unlock();

            // 3) Stall thread
            std::this_thread::sleep_until(sleepTime);
        }
        //std::cout << "Thread Killed!" << std::endl;
    }
}; 

} // namespace usb
} // namespace rev
