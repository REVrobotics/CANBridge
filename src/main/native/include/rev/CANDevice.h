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
#include <map>

#include "rev/CANMessage.h"
#include "rev/CANStatus.h"
#include "rev/CANBridgeUtils.h"
#include "utils/ThreadUtils.h"

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

    /**
     * Start, stop, or update a periodic signal
     *
     * @param msg the message to send. See notes in periodMs.
     * @param periodMs the period of the signal, 0, or -1. If this value is -1,
     *        the value of msg is irrelevant, and we stop any signals with the
     *        given id. If the value is 0, we queue a new message given by msg.
     *        This message will always be scheduled to be sent exactly once.
     *        If a repeating value is scheduled, passing 0 will cancel it.
     *        If this value is any positive value, schedule msg to be sent
     *        every periodMs milliseconds. If a message with this id was already
     *        scheduled to repeat, update the period and message contents. Note
     *        that this will not replace a message with periodMs set to 0.
     *
     * @return a CANStatus
     */
    virtual CANStatus SendCANMessage(const CANMessage& msg, int periodMs) = 0;
    virtual CANStatus ReceiveCANMessage(std::shared_ptr<CANMessage>& msg, uint32_t messageID, uint32_t messageMask) = 0;
    virtual CANStatus OpenStreamSession(uint32_t* sessionHandle, CANBridge_CANFilter filter, uint32_t maxSize) = 0;
    virtual CANStatus CloseStreamSession(uint32_t sessionHandle) = 0;
    virtual CANStatus ReadStreamSession(uint32_t sessionHandle, HAL_CANStreamMessage* msgs, uint32_t messagesToRead, uint32_t* messagesRead) = 0;

    virtual CANStatus GetCANDetailStatus(float* percentBusUtilization, uint32_t* busOff, uint32_t* txFull, uint32_t* receiveErr, uint32_t* transmitErr) = 0;
    virtual CANStatus GetCANDetailStatus(float* percentBusUtilization, uint32_t* busOff, uint32_t* txFull, uint32_t* receiveErr, uint32_t* transmitErr, uint32_t* lastErrorTime) = 0;

    virtual bool IsConnected() = 0;
    virtual void setThreadPriority(utils::ThreadPriority priority) {};
    virtual void stopRepeatedMessage(uint32_t messageId) {};
    virtual void ClearSendQueue() {};

    virtual bool CopyReceivedMessagesMap(std::map<uint32_t, std::shared_ptr<CANMessage>>& receivedMessagesMap) = 0;
};

} // namespace usb
} // namespace rev
