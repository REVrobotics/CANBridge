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

#include "rev/Drivers/SerialPort/SerialDevice.h"

#include <iostream> //TODO: Remove
#include <thread>

#include <mockdata/CanData.h>
#include <hal/CAN.h>

#include "serial/serial.h"

namespace rev {
namespace usb {

SerialDevice::SerialDevice(std::string port) :
    m_thread(port)
{  
    m_descriptor = std::wstring();
    convert_string_to_wstring(port, m_descriptor);
    m_name = "SPARK MAX";
    m_thread.Start();

}

SerialDevice::~SerialDevice()
{
    m_thread.Stop();
}

std::string SerialDevice::GetName() const
{
    return m_name;
}


std::wstring SerialDevice::GetDescriptor() const
{
    return m_descriptor;
}

int SerialDevice::GetId() const
{
    return 0;
}

CANStatus SerialDevice::SendCANMessage(const CANMessage& msg, int periodMs)
{
   
    m_thread.EnqueueMessage(msg, periodMs);
    return CANStatus::kOk;
}

CANStatus SerialDevice::RecieveCANMessage(CANMessage& msg, uint32_t messageID, uint32_t messageMask)
{
    CANStatus status = CANStatus::kTimeout;
   
    // parse through the keys, find the messges the match, and return it
    // The first in the message id, then the messages
    std::map<uint32_t, CANMessage> messages;
    m_thread.RecieveMessage(messages);
    CANMessage mostRecent;
    for (auto& m : messages) {
        if (CANBridge_ProcessMask({m.second.GetMessageId(), 0}, m.first) && CANBridge_ProcessMask({messageID, messageMask}, m.first)) {
            mostRecent = m.second;
            status = CANStatus::kOk;    
        }
    }
    
    if (status == CANStatus::kOk) {
        msg = mostRecent;
    }

    return status;
}

CANStatus SerialDevice::OpenStreamSession(uint32_t* sessionHandle, CANBridge_CANFilter filter, uint32_t maxSize)
{
    // Register the stream with the correct buffer size
    CANStatus stat = CANStatus::kOk;
    m_thread.OpenStream(sessionHandle, filter, maxSize, &stat);
    
    return stat;
}
CANStatus SerialDevice::CloseStreamSession(uint32_t sessionHandle)
{
    m_thread.CloseStream(sessionHandle);
    return CANStatus::kOk;
}
CANStatus SerialDevice::ReadStreamSession(uint32_t sessionHandle, struct HAL_CANStreamMessage* msgs, uint32_t messagesToRead, uint32_t* messagesRead, int32_t* status)
{
    m_thread.ReadStream(sessionHandle, msgs, messagesToRead, messagesRead);
    status = static_cast<int32_t>(CANStatus::kOk);    
    return CANStatus::kOk;
}

CANStatus SerialDevice::GetCANStatus(float* percentBusUtilization, uint32_t* busOff, uint32_t* txFull, uint32_t* receiveErr, uint32_t* transmitErr, int32_t* status)
{
    return CANStatus::kOk;
}

bool SerialDevice::IsConnected()
{
    return true;
}


} // namespace usb
} // namespace rev

#else
typedef int __ISOWarning__CLEAR_;
#endif // _WIN32
