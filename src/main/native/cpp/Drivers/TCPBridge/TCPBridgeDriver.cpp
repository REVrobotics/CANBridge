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

const std::string defaultPort("8800");

std::vector<CANDeviceDetail> TCPBridgeDriver::GetDevices()
{
    std::vector<CANDeviceDetail> retval;
    std::vector<std::string> devs = GetHostList();
    for(std::vector<std::string>::const_iterator it = devs.begin(); it != devs.end(); ++it)
    {
        TCPBridgeDevice m_dev{*it, GetPort()};
        if(m_dev.IsConnected()) {
            retval.push_back({m_dev.GetDescriptor(), "roboRIO", this->GetName()});
        }
        else
            std::cout << "Could not connect" << std::endl;
    }

    return retval;
}

std::unique_ptr<CANDevice> TCPBridgeDriver::CreateDeviceFromDescriptor(const char* descriptor)
{
    std::string s_descriptor(descriptor);

    std::string delimiter = ":";
    std::string host_name = s_descriptor.substr(0, s_descriptor.find(delimiter));
    std::string port = s_descriptor.substr(s_descriptor.find(delimiter) + delimiter.length(), s_descriptor.length());

    if(!host_name.empty() && !port.empty())
    {
        return std::make_unique<TCPBridgeDevice>(host_name, port);
    }
    else
    {
        std::cerr << "Invalid descriptor received: " << s_descriptor << std::endl;
        return nullptr;
    }
}

void TCPBridgeDriver::SetTeamNumber(std::string teamNumber)
{
    m_teamNumber = teamNumber;
}

std::string TCPBridgeDriver::GetTeamNumber() const
{
    return m_teamNumber;
}

void TCPBridgeDriver::SetPort(std::string port)
{
    m_port = port;
}

std::string TCPBridgeDriver::GetPort() const
{
    return m_port;
}

std::vector<std::string> TCPBridgeDriver::GetHostList()
{
    std::vector<std::string> hostList;

    hostList.push_back("172.22.11.2"); // default ip when connected over usb

    // if team number has been set
    if(!m_teamNumber.empty())
    {
        std::string upperDigits = m_teamNumber.substr(0, 2);
        std::string lowerDigits = m_teamNumber.substr(2, 2);
        hostList.push_back("roborio-" + m_teamNumber + "-FRC");
        hostList.push_back("10." + upperDigits + "." + lowerDigits + ".2"); // "10.xx.yy.2"
        hostList.push_back("roborio-" + m_teamNumber + "-FRC.lan");
        hostList.push_back("roborio-" + m_teamNumber + "-FRC.frc-field.local");
    }

    return hostList;
}

} // namespace usb
} // namespace rev
