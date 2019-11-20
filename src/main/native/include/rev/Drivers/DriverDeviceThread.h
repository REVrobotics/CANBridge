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
#include <chrono>
#include <stdint.h>
#include <map>
#include <queue>

#include <iostream>

#include "rev/CANMessage.h"
#include "rev/CANBridgeUtils.h"
#include "rev/CANStatus.h"

#include <mockdata/CanData.h>
#include <hal/CAN.h>

namespace rev {
namespace usb {

class DriverDeviceThread {
public:

    DriverDeviceThread(uint32_t counter, long long threadIntervalMs) :
        m_threadComplete(false),
        m_threadIntervalMs(threadIntervalMs),
        m_counter(counter) {
    }

    virtual void Start() = 0;

void Stop() {
        m_threadComplete = true;
        if (m_thread.joinable()) {
            m_thread.join();
        }
    }

    bool EnqueueMessage(const CANMessage& msg, int32_t timeIntervalMs) {
        m_writeMutex.lock();
        std::cout << "enqueue >> " << msg.GetMessageId() << std::endl;
        m_sendQueue.push(detail::CANThreadSendQueueElement(msg, timeIntervalMs));
        m_writeMutex.unlock();

        // TODO: Limit the max queue size
        return true;
    }

    bool ReceiveMessage(std::map<uint32_t, std::shared_ptr<CANMessage>>& readMap) {
        // This needs to return all messages with the id, the it will handle which ones pass the mask
        m_readMutex.lock();
        readMap = m_readStore;
        m_readMutex.unlock();

        return true;
    }

    virtual void OpenStream(uint32_t* handle, CANBridge_CANFilter filter, uint32_t maxSize, CANStatus* status) = 0;

    void CloseStream(uint32_t handle) {
        m_streamMutex.lock();
        m_readStream.erase(handle);
        m_streamMutex.unlock();
    }

    void ReadStream(uint32_t handle, struct HAL_CANStreamMessage* messages, uint32_t messagesToRead, 
                    uint32_t* messagesRead) {
        m_streamMutex.lock();
        *messagesRead = m_readStream[handle]->messages.GetCount(); // first before remove
        for (uint32_t i = 0; i < *messagesRead; i++) {
            std::shared_ptr<CANMessage> m;
            if (m_readStream[handle]->messages.Remove(m)) {
                messages[i] = ConvertCANToHALMessage(m);
            }
        }
        m_streamMutex.unlock();
    }

    HAL_CANStreamMessage ConvertCANToHALMessage(std::shared_ptr<CANMessage> msg) {
        HAL_CANStreamMessage halMsg;
        halMsg.timeStamp = msg->GetTimestampUs();
        halMsg.messageID = msg->GetMessageId();
        halMsg.dataSize = msg->GetSize();
        memcpy(halMsg.data, msg->GetData(), sizeof(halMsg.data));

        return halMsg;
    }

    bool ShouldRun() {
        return m_run;
    }

    void GetCANStatus(rev::usb::CANStatusDetails* details) {
        details = &m_statusDetails;
    }


protected:
    std::atomic_bool m_threadComplete;
    std::atomic_bool m_run = false;
    std::thread m_thread;
    std::mutex m_writeMutex;
    std::mutex m_readMutex;
    std::mutex m_streamMutex;

    uint32_t m_counter;

    CANStatusDetails m_statusDetails;

    std::queue<detail::CANThreadSendQueueElement> m_sendQueue;
    std::map<uint32_t, std::shared_ptr<CANMessage>> m_readStore;
    std::map<uint32_t, std::unique_ptr<CANStreamHandle>> m_readStream; // (id, mask), max size, message buffer

    long long m_threadIntervalMs;

    virtual void ReadMessages(bool &reading) = 0;
    virtual void WriteMessages(detail::CANThreadSendQueueElement element, std::chrono::steady_clock::time_point now) = 0;
    void ReadStreamMessages() {
        
    }

    void run() {
        while (m_threadComplete == false && m_run) {
            auto sleepTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(m_threadIntervalMs);

            // 1) Handle all received CAN traffic
            bool reading = true;
            while (reading) {
                ReadMessages(reading);
            }

            // 2) Schedule CANMessage queue
            m_writeMutex.lock();
            size_t queueSize = m_sendQueue.size();

            for (size_t i=0;i<queueSize;i++) {
                detail::CANThreadSendQueueElement el = m_sendQueue.front();
                m_sendQueue.pop();
                if (el.m_intervalMs == -1) {
                    continue;
                }

                auto now = std::chrono::steady_clock::now();

                WriteMessages(el, now);

                // Return to queue if repeated
                if (el.m_intervalMs > 0 ) {
                    el.m_prevTimestamp = now;
                    m_sendQueue.push(el);
                }
            }

            m_writeMutex.unlock();

            // 3) Stall thread
            std::this_thread::sleep_until(sleepTime);
        }

    }
};



} // namespace usb
} // namespace rev