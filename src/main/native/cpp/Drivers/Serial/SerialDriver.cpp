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

#include "rev/Drivers/SerialPort/SerialDriver.h"
#include "rev/Drivers/SerialPort/SerialDevice.h"
#include "rev/CANBridgeUtils.h"

#include "serial/serial.h"

#include <map>
#include <iostream>
#include <memory>
#include <sstream>

namespace rev {
namespace usb {

static const std::string SparkMax_HardwareId = "USB\\VID_0483&PID_5740&REV_0200";

std::vector<CANDeviceDetail> SerialDriver::GetDevices()
{
    // Search driver layer for devices
    std::vector<CANDeviceDetail> retval;

    std::vector<serial::PortInfo> found = serial::list_ports();
    for (auto& dev : found) {
        if (parse_serial_com_port(dev.port) != -1 && dev.hardware_id.compare(SparkMax_HardwareId) == 0) {
            std::string name("SPARK MAX"); 
            retval.push_back({dev.port, name, this->GetName()}); 
        }
    }

    return retval;
}

std::unique_ptr<CANDevice> SerialDriver::CreateDeviceFromDescriptor(const char* descriptor)
{
    // Search driver layer for devices
    std::vector<serial::PortInfo> found = serial::list_ports();
    for (auto& dev : found) {
        if (dev.port == std::string(descriptor)) {
            try {
                return std::make_unique<SerialDevice>(dev.port);
            } catch(...) {
                // do nothing if it failed
            }
        }
    }

    return std::unique_ptr<CANDevice>();
}



} // namespace usb
} // namespace rev

#else
typedef int __ISOWarning__CLEAR_;
#endif // _WIN32
