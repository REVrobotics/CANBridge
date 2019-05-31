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

#include "CANMessage.h"

#include "candle.h"

namespace rev {
namespace usb {

class CandleWinUSBDeviceThread { 
public:
    CandleWinUSBDeviceThread() =delete;
    CandleWinUSBDeviceThread(candle_handle dev) : 
        m_device(dev),
        m_threadComplete(false) 
    {
        m_thread = std::thread (&CandleWinUSBDeviceThread::run, this);
    }
    ~CandleWinUSBDeviceThread()
    {
        m_threadComplete = true;
        if (m_thread.joinable()) {
            m_thread.join();
        }
    }

    bool EnqueueMessage(const CANMessage& msg) {
        m_mutex.lock();
        m_sendQueue.push(msg);
        m_mutex.unlock();

        // TODO: Limit the max queue size
        return true;
    }

private:
    std::atomic_bool m_threadComplete;
    std::thread m_thread;
    std::mutex m_mutex;

    std::queue<CANMessage> m_sendQueue;
    std::map<uint32_t, std::vector<CANMessage>> m_recvStore;

    candle_handle m_device;
    void run() {
        while (m_threadComplete == false) {
            
        }
    }
}; 

} // namespace usb
} // namespace rev
