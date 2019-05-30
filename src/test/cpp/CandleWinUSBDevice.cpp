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

#include "CandleWinUSBDevice.h"

namespace rev {
namespace usb {

CandleWinUSBDevice::CandleWinUSBDevice(candle_handle hDev)
{
    m_handle = hDev;
    candle_dev_open(hDev);
}

CandleWinUSBDevice::~CandleWinUSBDevice()
{
    candle_dev_close(m_handle);
    candle_dev_free(m_handle);
}

std::string CandleWinUSBDevice::GetName() const
{
    return "SPARK MAX";
}


std::wstring CandleWinUSBDevice::GetDescriptor() const
{
    return candle_dev_get_path(m_handle);
}

int CandleWinUSBDevice::GetId() const
{
    return 0;
}

CANStatus CandleWinUSBDevice::SendMessage(CANMessage msg, int periodMs)
{

}
CANStatus CandleWinUSBDevice::RecieveMessage(CANMessage& msg)
{
    
}
CANStatus CandleWinUSBDevice::OpenStreamSession()
{
    
}
CANStatus CandleWinUSBDevice::CloseStreamSession()
{
    
}
CANStatus CandleWinUSBDevice::ReadStreamSession()
{
    
}

CANStatus CandleWinUSBDevice::GetCANStatus()
{
    
}

bool CandleWinUSBDevice::IsConnected()
{
    return true;
}


} // namespace usb
} // namespace rev
