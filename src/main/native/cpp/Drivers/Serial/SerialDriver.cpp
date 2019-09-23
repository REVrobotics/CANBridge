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

#include "rev/Drivers/Serial/SerialDriver.h"
#include "rev/Drivers/Serial/SerialDevice.h"

#include "rev/Drivers/Serial/serial.h"

#include <map>
#include <iostream>
#include <memory>

namespace rev {
namespace usb {

std::vector<CANDeviceDetail> SerialDriver::GetDevices()
{
    // Search driver layer for devices
    std::vector<CANDeviceDetail> retval;

    std::vector<serial::PortInfo> found  = serial::list_ports();
    for (auto& dev : found) {
            std::wstring desc(dev.hardware_id.c_str());
            std::string name(dev.port.c_str());
            retval.push_back({desc, name, this->GetName()});
    }

    return retval;
}

std::unique_ptr<CANDevice> SerialDriver::CreateDeviceFromDescriptor(const wchar_t* descriptor)
{
    // Search driver layer for devices
   
    std::vector<serial::PortInfo> found  = serial::list_ports();
    for (auto& dev : found) {
            std::wstring path(dev.hardware_id.c_str());
            if (path == std::wstring(descriptor)) {
                return std::make_unique<SerialDevice>(dev);
            }
    }

    return std::unique_ptr<CANDevice>();
}

} // namespace usb
} // namespace rev

#else
typedef int __ISOWarning__CLEAR_;
#endif // _WIN32
