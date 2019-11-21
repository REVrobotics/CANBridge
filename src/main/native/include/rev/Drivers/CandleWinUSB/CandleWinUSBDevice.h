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

#include <map>
#include <string>

#include "candlelib/candle.h"

#include "rev/Drivers/CandleWinUSB/CandleWinUSBDeviceThread.h"
#include "rev/CANDevice.h"
#include "rev/CANMessage.h"
#include "rev/CANStatus.h"

namespace rev {
namespace usb {

class CandleWinUSBDevice : public CANDevice {
public:
    CandleWinUSBDevice() =delete;
    CandleWinUSBDevice(candle_handle hDev);
    virtual ~CandleWinUSBDevice();

    virtual std::string GetName() const;
    virtual std::string GetDescriptor() const;
    virtual int GetNumberOfErrors();

    virtual int GetId() const;

    virtual CANStatus SendCANMessage(const CANMessage& msg, int periodMs) override;
    virtual CANStatus ReceiveCANMessage(std::shared_ptr<CANMessage>& msg, uint32_t messageID, uint32_t messageMask) override;
    virtual CANStatus OpenStreamSession(uint32_t* sessionHandle, CANBridge_CANFilter filter, uint32_t maxSize) override;
    virtual CANStatus CloseStreamSession(uint32_t sessionHandle);
    virtual CANStatus ReadStreamSession(uint32_t sessionHandle, HAL_CANStreamMessage* msgs, uint32_t messagesToRead, uint32_t* messagesRead);

    virtual CANStatus GetCANDetailStatus(float* percentBusUtilization, uint32_t* busOff, uint32_t* txFull, uint32_t* receiveErr, uint32_t* transmitErr);

    virtual bool IsConnected();
private:
    candle_handle m_handle;
    CandleWinUSBDeviceThread m_thread;
    std::string m_descriptor;
    std::string m_name;
};

} // namespace usb
} // namespace rev
