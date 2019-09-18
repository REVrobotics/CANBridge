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

#include "rev/CANMessage.h"
#include "rev/CANBridgeUtils.h"
#include "utils/CircularBuffer.h"

#include "candlelib/candle.h"

#include <mockdata/CanData.h>
#include <hal/CAN.h>

namespace rev {
namespace usb {

struct CANStreamHandle {
    uint32_t messageId;
    uint32_t messageMask;
    uint32_t maxSize;
    utils::CircularBuffer<CANMessage> messages; 

};

namespace detail {

class CANThreadSendQueueElement {
public:
    CANThreadSendQueueElement() =delete;
    CANThreadSendQueueElement(CANMessage msg, int32_t intervalMs) : 
        m_msg(msg), m_intervalMs(intervalMs), m_prevTimestamp(std::chrono::steady_clock::now()) {

    }
    CANMessage m_msg;
    int32_t m_intervalMs;
    std::chrono::time_point<std::chrono::steady_clock> m_prevTimestamp;
};

} // namespace detail

class TCPBridgeDeviceThread { 
public:
    TCPBridgeDeviceThread() =delete;
    TCPBridgeDeviceThread(std::string ip, long long threadIntervalMs = 1) : 
        // m_device(dev),
        m_threadComplete(false),
        m_threadIntervalMs(threadIntervalMs)
    {
        //m_thread = std::thread (&TCPBridgeDeviceThread::run, this);
    }
    ~TCPBridgeDeviceThread()
    {
    }

    void Start() {
        std::cout << "Starting starting Thread..." << std::endl;
        m_thread = std::thread (&TCPBridgeDeviceThread::run, this);
    }

    void Stop() {
        std::cout << "Stopping Thread..." << std::endl;
        m_threadComplete = true;
        if (m_thread.joinable()) {
            m_thread.join();
        }
        std::cout << "Thread Stopped." << std::endl;
    }

    bool EnqueueMessage(const CANMessage& msg, int32_t timeIntervalMs) {
        // m_sendMutex.lock();
        // m_sendQueue.push(detail::CANThreadSendQueueElement(msg, timeIntervalMs));
        // m_sendMutex.unlock();

        // TODO: Limit the max queue size
        return true;
    }

    bool RecieveMessage(std::map<uint32_t, CANMessage>& recvMap) {
        // This needs to return all messages with the id, the it will handle which ones pass the mask
        // m_recvMutex.lock();
        // recvMap = m_recvStore;
        // m_recvMutex.unlock();

        return true;
    }

    void OpenStream(uint32_t* handle, CANBridge_CANFilter filter, uint32_t maxSize) {
        // m_streamMutex.lock();

        // // Create the handle
        // *handle = counter++;

        // // Add to the map
        // m_recvStream[*handle] = std::unique_ptr<CANStreamHandle>(new CANStreamHandle{filter.messageId, filter.messageMask, maxSize, utils::CircularBuffer<CANMessage>{maxSize}});

        // m_streamMutex.unlock();
    }

    void CloseStream(uint32_t handle) {
        // m_streamMutex.lock();
        // m_recvStream.erase(handle);
        // m_streamMutex.unlock();
    }

    // Return a vector of the messages, because pointer can't point to empty vector
    // Use a circular buffer instead of just the vector
    void ReadStream(uint32_t handle, struct HAL_CANStreamMessage* messages, uint32_t messagesToRead, uint32_t* messagesRead) {
        // m_streamMutex.lock();
        // *messagesRead = m_recvStream[handle]->messages.GetCount(); // first before remove

        // for (uint32_t i = 0; i < *messagesRead; i++) {
        //     CANMessage m;
        //     if (m_recvStream[handle]->messages.Remove(m)) {
        //         messages[i] = ConvertCANRecieveToHALMessage(m);

        //     }
        // }
        // m_streamMutex.unlock();
        
    }

    static HAL_CANStreamMessage ConvertCANRecieveToHALMessage(rev::usb::CANMessage msg)
    {
        HAL_CANStreamMessage halMsg;
        // halMsg.timeStamp = msg.GetTimestampUs();
        // halMsg.messageID = msg.GetMessageId();
        // halMsg.dataSize = msg.GetSize();
        // memcpy(halMsg.data, msg.GetData(), sizeof(halMsg.data));

        return halMsg;
    }



private:
    // candle_handle m_device;
    
    std::atomic_bool m_threadComplete;
    std::thread m_thread;
    std::mutex m_sendMutex;
    std::mutex m_recvMutex;
    std::mutex m_streamMutex;

    // uint32_t counter = 0xe45b5597;

    // std::queue<detail::CANThreadSendQueueElement> m_sendQueue;
    // std::map<uint32_t, CANMessage> m_recvStore;
    // std::map<uint32_t, std::unique_ptr<CANStreamHandle>> m_recvStream; // (id, mask), max size, message buffer

    long long m_threadIntervalMs;

    void run() {
        std::cout << "Thread starting!" << std::endl;

        while (m_threadComplete == false) {
            auto sleepTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(m_threadIntervalMs);

            // 3) Stall thread
            std::this_thread::sleep_until(sleepTime);
        }
        std::cout << "Thread Killed!" << std::endl;
    }
}; 

} // namespace usb
} // namespace rev
