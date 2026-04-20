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

#ifdef _WIN32

#include "rev/Drivers/CandleWinUSB/CandleWinUSBDevice.h"
#include "rev/CANBridgeUtils.h"

#include <iostream> //TODO: Remove
#include <thread>
#include <map>

#include <hal/simulation/CanData.h>
#include <hal/CAN.h>

#define CANDLE_DEFAULT_CHANNEL 0
#define CANDLE_DEFAULT_FLAGS    (candle_device_mode_flags_t)(CANDLE_MODE_NORMAL | CANDLE_MODE_PAD_PKTS_TO_MAX_PKT_SIZE)

namespace rev {
namespace usb {

static void close_candle_dev(candle_handle hDev)
{
    candle_dev_close(hDev);
    candle_dev_free(hDev);
}

CandleWinUSBDevice::CandleWinUSBDevice(candle_handle hDev) :
    m_thread(hDev)
{
    m_handle = hDev;
    if (candle_dev_open(hDev) == false) {
        std::cout << candle_error_text(candle_dev_last_error(hDev)) << std::endl;
        close_candle_dev(hDev);
        throw "Failed to open device!";
    }

    // Set FRC capable settings, SPARK MAX is hard coded to this, but other devices (e.g. canable with
    // candlelight_fw) may need to be set to it explicitly. Note this only works for devices with 48MHz
    // clocks, and could be expanded in the future to work with any device that properly reports its
    // own capabilities.
    if (!candle_channel_set_bitrate(hDev, CANDLE_DEFAULT_CHANNEL, 1000000)) {
        std::cout << "Unable to set bitrate! " << candle_error_text(candle_dev_last_error(hDev)) << std::endl;
    }


    if (candle_channel_start(hDev, CANDLE_DEFAULT_CHANNEL, CANDLE_DEFAULT_FLAGS) == false) {
        std::cout << candle_error_text(candle_dev_last_error(hDev)) << std::endl;
        close_candle_dev(hDev);
        throw "Failed to start device channel 0!";
    }

    m_descriptor = unicode_decode(candle_dev_get_path(m_handle));
    m_name = candle_dev_get_name(m_handle);
    m_thread.Start();
}

CandleWinUSBDevice::~CandleWinUSBDevice()
{
    m_thread.Stop();

    candle_channel_stop(m_handle, CANDLE_DEFAULT_CHANNEL);
    close_candle_dev(m_handle);
}

void CandleWinUSBDevice::ClearSendQueue() {
    m_thread.clearQueue();
}

std::string CandleWinUSBDevice::GetName() const
{
    return m_name;
}


std::string CandleWinUSBDevice::GetDescriptor() const
{
    return m_descriptor;
}

int CandleWinUSBDevice::GetId() const
{
    return 0;
}

int CandleWinUSBDevice::GetNumberOfErrors()
{
    return m_thread.GetNumberOfErrors();
}

void CandleWinUSBDevice::setThreadPriority(utils::ThreadPriority priority) {
    m_thread.setPriority(priority);
}

void CandleWinUSBDevice::stopRepeatedMessage(uint32_t messageId) {
    m_thread.stopRepeatedMessage(messageId);
}

CANStatus CandleWinUSBDevice::SendCANMessage(const CANMessage& msg, int periodMs)
{
    m_thread.EnqueueMessage(msg, periodMs);
    return m_thread.GetLastThreadError();
}

CANStatus CandleWinUSBDevice::ReceiveCANMessage(std::shared_ptr<CANMessage>& msg, uint32_t messageID, uint32_t messageMask)
{
    // Assume timeout
    CANStatus status = CANStatus::kTimeout;

    // parse through the keys, find the messages that match, and return it
    // The first in the message id, then the messages
    std::map<uint32_t, std::shared_ptr<CANMessage>> messages;
    m_thread.ReceiveMessage(messages);
    std::shared_ptr<CANMessage> mostRecent;
    for (auto& m : messages) {
        if (
            CANBridge_ProcessMask({m.second->GetMessageId(), 0}, m.first)
            && CANBridge_ProcessMask({messageID, messageMask}, m.first)
            && (!mostRecent || m.second->GetTimestampUs() > mostRecent->GetTimestampUs())
        ) {
            mostRecent = m.second;
            status = CANStatus::kOk;
        }
    }

    if (status == CANStatus::kOk) {
        msg = mostRecent;
        status = m_thread.GetLastThreadError(); // check and see if something else went wrong
    }

    return status;
}

CANStatus CandleWinUSBDevice::OpenStreamSession(uint32_t* sessionHandle, CANBridge_CANFilter filter, uint32_t maxSize)
{
    CANStatus status;
    // Register the stream with the correct buffer size
    m_thread.OpenStream(sessionHandle, filter, maxSize, &status);

    return m_thread.GetLastThreadError();
}
CANStatus CandleWinUSBDevice::CloseStreamSession(uint32_t sessionHandle)
{
    m_thread.CloseStream(sessionHandle);
    return m_thread.GetLastThreadError();;
}
CANStatus CandleWinUSBDevice::ReadStreamSession(uint32_t sessionHandle, struct HAL_CANStreamMessage* msgs, uint32_t messagesToRead, uint32_t* messagesRead)
{
    m_thread.ReadStream(sessionHandle, msgs, messagesToRead, messagesRead);
    return m_thread.GetLastThreadError();
}

CANStatus CandleWinUSBDevice::GetCANDetailStatus(float* percentBusUtilization, uint32_t* busOff, uint32_t* txFull, uint32_t* receiveErr, uint32_t* transmitErr) {
    return GetCANDetailStatus(percentBusUtilization, busOff, txFull,receiveErr, transmitErr, nullptr);
}

CANStatus CandleWinUSBDevice::GetCANDetailStatus(float* percentBusUtilization, uint32_t* busOff, uint32_t* txFull, uint32_t* receiveErr, uint32_t* transmitErr, uint32_t* lastErrorTime)
{
    m_thread.GetCANStatusDetails(busOff, txFull, receiveErr, transmitErr, lastErrorTime);
    *percentBusUtilization = 0.0; // todo how to get this

    return m_thread.GetLastThreadError();
}

bool CandleWinUSBDevice::IsConnected()
{
    return true;
}

bool CandleWinUSBDevice::CopyReceivedMessagesMap(std::map<uint32_t, std::shared_ptr<CANMessage>>& receivedMessagesMap)
{
    return m_thread.ReceiveMessage(receivedMessagesMap);
}


} // namespace usb
} // namespace rev

#else
typedef int __ISOWarning__CLEAR_;
#endif // _WIN32
