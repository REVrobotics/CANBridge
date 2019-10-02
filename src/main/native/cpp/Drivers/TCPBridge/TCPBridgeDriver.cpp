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

#include "rev/Drivers/TCPBridge/TCPBridgeDriver.h"
#include "rev/Drivers/TCPBridge/TCPBridgeDevice.h"

#include <map>
#include <iostream>
#include <memory>

#include "candlelib/candle.h"

namespace rev {
namespace usb {

std::vector<CANDeviceDetail> TCPBridgeDriver::GetDevices()
{
    std::vector<CANDeviceDetail> retval;
    // TODO: make this not hardcoded...
    TCPBridgeDevice m_dev{"roboRIO-2714-FRC.local", "8800"};
    if(m_dev.IsConnected()) {
        retval.push_back({m_dev.GetDescriptor(), "roboRIO", this->GetName()});
    }
    else
        std::cout << "Could not connect" << std::endl;

    return retval;
}

std::unique_ptr<CANDevice> TCPBridgeDriver::CreateDeviceFromDescriptor(const wchar_t* descriptor)
{
    std::string ipAddr;
    convert_wstring_to_string(descriptor, ipAddr);

    // TODO: make this not hardcoded...
    return std::make_unique<TCPBridgeDevice>("roboRIO-2714-FRC.local", "8800");
}

} // namespace usb
} // namespace rev
