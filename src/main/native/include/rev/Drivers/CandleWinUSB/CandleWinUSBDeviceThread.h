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
#include "rev/CANStatus.h"
#include "rev/Drivers/DriverDeviceThread.h"

#include "utils/ThreadUtils.h"

#include "candlelib/candle.h"

#include <hal/simulation/CanData.h>
#include <hal/CAN.h>

#define CANDLE_EXTENDED_ID_MASK 0x80000000
#define CANDLE_REMOTE_FRAME_MASK 0x40000000

namespace rev {
namespace usb {


class CandleWinUSBDeviceThread :public DriverDeviceThread { 
public:
    CandleWinUSBDeviceThread() =delete;
    CandleWinUSBDeviceThread(candle_handle dev, long long threadIntervalMs = 1) : DriverDeviceThread(0xe45b5597, threadIntervalMs),
        m_device(dev)
    { }
    ~CandleWinUSBDeviceThread()
    {
    }

    void Start() override {
        if (m_thread.get() != nullptr && m_thread->joinable()) {
            m_thread->join();
        }

        m_thread = std::make_unique<std::thread>(&CandleWinUSBDeviceThread::CandleRun, this);

        // Set to high priority to prevent buffer overflow on the device on high client CPU load
        utils::SetThreadPriority(m_thread.get(), utils::ThreadPriority::High);
    }

    void setPriority(utils::ThreadPriority priority) {
        if (m_thread.get() != nullptr)
            utils::SetThreadPriority(m_thread.get(), priority);
    }

    void clearQueue() {
        bool empty = m_sendQueue.empty();
    }

    void stopRepeatedMessage(uint32_t messageId) {
        for (int i = 0; i < m_sendQueue.size(); i++) {
            detail::CANThreadSendQueueElement el = m_sendQueue.front();
            m_sendQueue.pop_front();
            if (el.m_msg.GetMessageId() != messageId) m_sendQueue.push_back(el);
        }
    }

    void OpenStream(uint32_t* handle, CANBridge_CANFilter filter, uint32_t maxSize, CANStatus *status) override {
        std::lock_guard<std::mutex> lock(m_streamMutex);

        // Create the handle
        *handle = m_counter++;

        // Add to the map
        m_readStream[*handle] = std::unique_ptr<CANStreamHandle>(new CANStreamHandle{filter.messageId, filter.messageMask, maxSize, utils::CircularBuffer<std::shared_ptr<CANMessage>>{maxSize}});

        *status = CANStatus::kOk;
    }



private:
    candle_handle m_device;
    
   void ReadMessages(bool &reading) {
       candle_frame_t incomingFrame;
    
        reading = candle_frame_read(m_device, &incomingFrame, 0);
        // Received a new frame, store it
        if (reading) {
            candle_frametype_t frameType = candle_frame_type(&incomingFrame);
            if(frameType == CANDLE_FRAMETYPE_ERROR) {
                time(&lastErrorTime);
                // Parse error data
                if (incomingFrame.can_id & 0x00000040) {
                    m_statusDetails.busOffCount++;
                } 
                if (incomingFrame.data[1] & 0x02) {
                    m_statusDetails.txFullCount++;
                }
                if (incomingFrame.data[1] & 0x10 || incomingFrame.data[1] & 0x04) {
                    m_statusDetails.receiveErrCount++;
                }
                if (incomingFrame.data[1] & 0x20 || incomingFrame.data[1] & 0x08 || incomingFrame.data[2] & 0x80 || incomingFrame.data[4]) {
                    m_statusDetails.transmitErrCount++;
                }
            } else if(frameType == CANDLE_FRAMETYPE_RECEIVE) {
                
                auto msg = std::make_shared<CANMessage>(candle_frame_id(&incomingFrame),
                                                        candle_frame_data(&incomingFrame),
                                                        candle_frame_dlc(&incomingFrame),
                                                        candle_frame_timestamp_us(&incomingFrame));

                // Read functions
                {
                    std::lock_guard<std::mutex> lock(m_readMutex);
                    m_readStore[candle_frame_id(&incomingFrame)] = msg;
                }

                // Streaming functions
                {
                    std::lock_guard<std::mutex> lock(m_streamMutex);
                    for (auto& stream : m_readStream) {
                        // Compare current size of the buffer to the max size of the buffer
                        if (!stream.second->messages.IsFull()
                            && rev::usb::CANBridge_ProcessMask({stream.second->messageId, stream.second->messageMask},
                            msg->GetMessageId())) {
                            stream.second->messages.Add(msg);
                        }
                    }
                }
            }

            m_threadStatus = CANStatus::kOk;
        }
   }

   bool WriteMessages(detail::CANThreadSendQueueElement el, std::chrono::steady_clock::time_point now) {
        if (el.m_intervalMs <= 1 || (now - el.m_prevTimestamp >= std::chrono::milliseconds(el.m_intervalMs)) ) {
            candle_frame_t frame;
            frame.can_dlc = el.m_msg.GetSize();

            uint32_t messageId = el.m_msg.GetMessageId() & NON_RESERVED_ARB_ID_MASK;

            bool isExtended = true; // FRC CAN is always extended
            bool isRtr = el.m_msg.GetMessageId() & HAL_CAN_IS_FRAME_REMOTE;

            frame.can_id = messageId;
            if(isExtended) {
            	frame.can_id |= CANDLE_EXTENDED_ID_MASK;
            }
            if(isRtr) {
                frame.can_id |= CANDLE_REMOTE_FRAME_MASK;
            }

            memcpy(frame.data, el.m_msg.GetData(), frame.can_dlc);
            frame.timestamp_us = now.time_since_epoch().count() / 1000;

            // TODO: Feed back an error
            if (candle_frame_send(m_device, 0, &frame, false, 20) == false) {
                // std::cout << "Failed to send message: " << std::hex << (int)el.m_msg.GetMessageId() << std::dec << "  " << candle_error_text(candle_dev_last_error(m_device)) << std::endl;
                m_threadStatus = CANStatus::kDeviceWriteError;
                m_statusErrCount++;
                return false;
            } else {
                m_threadStatus = CANStatus::kOk;
                return true;
            }
        }
        return false;
   }

    void CandleRun() {
        while (m_threadComplete == false) {
            m_threadStatus = CANStatus::kOk; // Start each loop with the status being good. Really only a write issue.
            auto sleepTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(m_threadIntervalMs);

            // 1) Handle all received CAN traffic
            bool reading = false;
            ReadMessages(reading);

            // 2) Schedule CANMessage queue
            {
                std::lock_guard<std::mutex> lock(m_writeMutex);
                if (m_sendQueue.size() > 0) {
                    detail::CANThreadSendQueueElement el = m_sendQueue.front();
                    if (el.m_intervalMs == -1) {
                        removeElementsWithId(el.m_msg.GetMessageId());
                        continue;
                    }

                    auto now = std::chrono::steady_clock::now();

                    m_sendQueue.pop_front();
                    if (WriteMessages(el, now)) {

                        // Return to end of queue if repeated
                        if (el.m_intervalMs > 0 ) {
                            el.m_prevTimestamp = now;
                            m_sendQueue.push_back(el);
                        }
                    } else {
                        m_sendQueue.push_back(el);
                    }
                }
            }

            // 3) Stall thread
            if (!reading && m_sendQueue.empty()) {
                std::this_thread::sleep_until(sleepTime);
            }
        }

    }
};

} // namespace usb
} // namespace rev
