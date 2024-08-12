/*
 * Copyright (c) 2019 - 2020 REV Robotics
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

#ifdef __linux__

#include "rev/Drivers/SocketCAN/SocketCANDriver.h"
#include "rev/Drivers/SocketCAN/SocketCANDevice.h"

#include <map>
#include <iostream>
#include <memory>

#include <net/if.h>

namespace rev {
namespace usb {

std::vector<CANDeviceDetail> SocketCANDriver::GetDevices()
{
    std::vector<CANDeviceDetail> retval;

    // TODO: Better way of doing this?
    // find canx or vcanx interface names
    struct if_nameindex *if_nidxs, *intf;

    if_nidxs = if_nameindex();
    if ( if_nidxs != NULL )
    {
        for (intf = if_nidxs; intf->if_index != 0 || intf->if_name != NULL; intf++)
        {
            char* buf = intf->if_name;

            // Not possible can name, protect later compares
            if (strnlen(buf, 4) < 4) {
                continue;
            }

            // possibly vcanx
            if (buf[0] == 'v') {
                buf++;
            }

            if (strncmp(buf, "can", 3) == 0) {
                std::string ifnameStr = std::string(intf->if_name);
                retval.push_back( {ifnameStr, ifnameStr, this->GetName()} );
            }
        }

        if_freenameindex(if_nidxs);
    }

    return retval;
}

std::unique_ptr<CANDevice> SocketCANDriver::CreateDeviceFromDescriptor(const char* descriptor)
{
    try {
        return std::make_unique<SocketCANDevice>(descriptor);
    } catch(...) {
        // do nothing if it failed
    }
    return std::unique_ptr<CANDevice>(nullptr);
}

SocketCANDevice::SocketCANDevice(const char* port) {
    // Constructor implementation
}

SocketCANDevice::~SocketCANDevice() {
    // Destructor implementation
}

std::string SocketCANDevice::GetName() const {
    // Implementation
    return "Name";
}

std::string SocketCANDevice::GetDescriptor() const {
    // Implementation
    return "Descriptor";
}

int SocketCANDevice::GetNumberOfErrors() {
    // Implementation
    return 0;
}

int SocketCANDevice::GetId() const {
    // Implementation
    return 0;
}

CANStatus SocketCANDevice::SendCANMessage(const CANMessage& msg, int periodMs) {
    // Implementation
    return CANStatus::kOk;
}

CANStatus SocketCANDevice::ReceiveCANMessage(std::shared_ptr<CANMessage>& msg, uint32_t messageID, uint32_t messageMask) {
    // Implementation
    return CANStatus::kOk;
}

CANStatus SocketCANDevice::OpenStreamSession(uint32_t* sessionHandle, CANBridge_CANFilter filter, uint32_t maxSize) {
    // Implementation
    return CANStatus::kOk;
}

CANStatus SocketCANDevice::CloseStreamSession(uint32_t sessionHandle) {
    // Implementation
    return CANStatus::kOk;
}

CANStatus SocketCANDevice::ReadStreamSession(uint32_t sessionHandle, HAL_CANStreamMessage* msgs, uint32_t messagesToRead, uint32_t* messagesRead) {
    // Implementation
    return CANStatus::kOk;
}

CANStatus SocketCANDevice::GetCANDetailStatus(float* percentBusUtilization, uint32_t* busOff, uint32_t* txFull, uint32_t* receiveErr, uint32_t* transmitErr) {
    // Implementation
    *percentBusUtilization = 0.0f;
    *busOff = 0;
    *txFull = 0;
    *receiveErr = 0;
    *transmitErr = 0;
    return CANStatus::kOk;
}

CANStatus SocketCANDevice::GetCANDetailStatus(float* percentBusUtilization, uint32_t* busOff, uint32_t* txFull, uint32_t* receiveErr, uint32_t* transmitErr, uint32_t* lastErrorTime) {
    // Implementation
    *percentBusUtilization = 0.0f;
    *busOff = 0;
    *txFull = 0;
    *receiveErr = 0;
    *transmitErr = 0;
    *lastErrorTime = 0;
    return CANStatus::kOk;
}

bool SocketCANDevice::IsConnected() {
    // Implementation
    return true;
}
} // namespace usb
} // namespace rev

#else
typedef int __ISOWarning__CLEAR_;
#endif // _WIN32
