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

#ifndef _CANBRIDGEUTILS_H_
#define _CANBRIDGEUTILS_H_

#include <string>

#include "rev/CANBridge.h"
#include "rev/CANMessage.h"
#include "utils/CircularBuffer.h"

namespace rev {
namespace usb {

struct CANStreamHandle {
    uint32_t messageId;
    uint32_t messageMask;
    uint32_t maxSize;
    utils::CircularBuffer<std::shared_ptr<CANMessage>> messages; 

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

class CANBridge_CANFilter {
    public:
    uint32_t messageId;
    uint32_t messageMask;

    friend bool operator<(const CANBridge_CANFilter& lhs, const CANBridge_CANFilter& rhs) {
        return lhs.messageId < rhs.messageId && lhs.messageMask < rhs.messageMask;
    }
};


void convert_wstring_to_string(const std::wstring& in, std::string& out); 
void convert_string_to_wstring(const std::string& in, std::wstring& out);

#if defined(_WIN32)
std::string unicode_decode(const std::wstring &wstr);
std::wstring unicode_encode(const std::string &wstr);
#endif


bool CANBridge_ProcessMask(const CANBridge_CANFilter& filter, uint32_t id); 

bool CANMessageCompare(CANMessage& a, CANMessage& b);

int parse_serial_com_port(const std::string& in);

} // namespace rev
} // namespace usb

#endif // _CANBRDIGEUTILS_H_
