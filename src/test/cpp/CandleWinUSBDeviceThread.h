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

// TODO: remove me
#include <clocale>
#include <iostream>

#include "CANMessage.h"

#include "candle.h"

namespace rev {
namespace usb {

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

class CandleWinUSBDeviceThread { 
public:
    CandleWinUSBDeviceThread() =delete;
    CandleWinUSBDeviceThread(candle_handle dev, long long threadIntervalMs = 1) : 
        m_device(dev),
        m_threadComplete(false),
        m_threadIntervalMs(threadIntervalMs)
    {
        //m_thread = std::thread (&CandleWinUSBDeviceThread::run, this);
    }
    ~CandleWinUSBDeviceThread()
    {
    }

    void Start() {
        std::cout << "Starting Thread..." << std::endl;
        m_thread = std::thread (&CandleWinUSBDeviceThread::run, this);
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
        m_sendMutex.lock();
        m_sendQueue.push(detail::CANThreadSendQueueElement(msg, timeIntervalMs));
        m_sendMutex.unlock();

        // TODO: Limit the max queue size
        return true;
    }

    bool RecieveMessage(CANMessage& msg);

private:
    std::atomic_bool m_threadComplete;
    std::thread m_thread;
    std::mutex m_sendMutex;
    std::mutex m_recvMutex;

    std::queue<detail::CANThreadSendQueueElement> m_sendQueue;
    std::map<uint32_t, std::queue<CANMessage>> m_recvStore;

    candle_handle m_device;

    long long m_threadIntervalMs;

    void run() {
        std::cout << "Thread starting!" << std::endl;

        while (m_threadComplete == false) {
            auto sleepTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(m_threadIntervalMs);

            // 1) Handle all recieved CAN traffic
            bool reading = true;
            while (reading) {
                candle_frame_t incomingFrame;
                reading = candle_frame_read(m_device, &incomingFrame, 0);

                // Recieved a new frame, store it
                if (reading) {
                    CANMessage msg(incomingFrame.can_id, incomingFrame.data, incomingFrame.can_dlc, incomingFrame.timestamp_us);

                    std::cout << "Recieved Message: " << std::hex << msg << std::endl;

                    // TODO: The queue is for streaming API, implement that here
                    m_recvMutex.lock();
                    if (m_recvStore[incomingFrame.can_id].size() > 1) {
                        m_recvStore[incomingFrame.can_id].pop();
                    }
                    m_recvStore[incomingFrame.can_id].push(msg);
                    m_recvMutex.unlock();
                }
            }

            // 2) Schedule CANMessage queue
            m_sendMutex.lock();
            size_t queueSize = m_sendQueue.size();

            for (int i=0;i<queueSize;i++) {
                detail::CANThreadSendQueueElement el = m_sendQueue.front();
                m_sendQueue.pop();
                if (el.m_intervalMs == -1) {
                    continue;
                }

                auto now = std::chrono::steady_clock::now();

                if (el.m_intervalMs == 0 || now - el.m_prevTimestamp >= std::chrono::milliseconds(el.m_intervalMs)) {
                    candle_frame_t frame;
                    frame.can_dlc = el.m_msg.GetSize();
                    frame.can_id = el.m_msg.GetMessageId();
                    memcpy(frame.data, el.m_msg.GetData(), frame.can_dlc);
                    frame.timestamp_us = now.time_since_epoch().count() / 1000;
                    if (candle_frame_send(m_device, 0, &frame, false, 20) == false) {
                        std::cout << "Failed to send message: " << candle_error_text(candle_dev_last_error(m_device)) << std::endl;
                        wchar_t tmp[512];
                        candle_windows_error_text(candle_dev_last_windows_error(m_device), tmp, 512);
                        std::wcout << L"Fail Code Windows: " << tmp << std::endl;
                    } else {
                        //std::cout << "Frame sent successfully" << std::endl;
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
        std::cout << "Thread Killed!" << std::endl;
    }
}; 

} // namespace usb
} // namespace rev
