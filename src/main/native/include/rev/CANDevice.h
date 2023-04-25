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

#include <string>

#include "rev/CANMessage.h"
#include "rev/CANStatus.h"
#include "rev/CANBridgeUtils.h"

#include <hal/simulation/CanData.h>
#include <hal/CAN.h>

namespace rev {
namespace usb {

class CANDevice {
public:
    CANDevice() {}
    virtual ~CANDevice() {}

    virtual std::string GetName() const = 0;
    virtual std::string GetDescriptor() const {return std::string();}
    virtual int GetNumberOfErrors() = 0;

    virtual int GetId() const = 0;

    virtual CANStatus SendCANMessage(const CANMessage& msg, int periodMs) = 0;
    virtual CANStatus ReceiveCANMessage(std::shared_ptr<CANMessage>& msg, uint32_t messageID, uint32_t messageMask) = 0;
    virtual CANStatus OpenStreamSession(uint32_t* sessionHandle, CANBridge_CANFilter filter, uint32_t maxSize) = 0;
    virtual CANStatus CloseStreamSession(uint32_t sessionHandle) = 0;
    virtual CANStatus ReadStreamSession(uint32_t sessionHandle, HAL_CANStreamMessage* msgs, uint32_t messagesToRead, uint32_t* messagesRead) = 0;

    virtual CANStatus GetCANDetailStatus(float* percentBusUtilization, uint32_t* busOff, uint32_t* txFull, uint32_t* receiveErr, uint32_t* transmitErr) = 0;

    virtual bool IsConnected() = 0;
};

} // namespace usb
} // namespace rev
