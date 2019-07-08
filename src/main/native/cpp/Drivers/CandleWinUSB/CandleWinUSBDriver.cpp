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

#include "rev/Drivers/CandleWinUSB/CandleWinUSBDriver.h"
#include "rev/Drivers/CandleWinUSB/CandleWinUSBDevice.h"

#include <map>
#include <iostream>
#include <memory>

#include "candlelib/candle.h"

namespace rev {
namespace usb {

std::vector<std::wstring> CandleWinUSBDriver::GetDevices()
{
    // Search driver layer for devices
    candle_list_handle clist;
    uint8_t num_interfaces;
    candle_handle dev;
    std::vector<std::wstring> retval;

    if (candle_list_scan(&clist)) {
        if (candle_list_length(clist, &num_interfaces)) {
            for (uint8_t i=0; i<num_interfaces; i++) {
                if (candle_dev_get(clist, i, &dev)) {
                    std::wstring path(candle_dev_get_path(dev));
                    retval.push_back(path);
                }
            }
        }
        candle_list_free(clist);
    }

    return retval;
}

std::unique_ptr<CANDevice> CandleWinUSBDriver::CreateDeviceFromDescriptor(const wchar_t* descriptor)
{
    // Search driver layer for devices
    candle_list_handle clist;
    uint8_t num_interfaces;
    candle_handle dev;

    if (candle_list_scan(&clist)) {
        if (candle_list_length(clist, &num_interfaces)) {
            for (uint8_t i=0; i<num_interfaces; i++) {
                if (candle_dev_get(clist, i, &dev)) {
                    std::wstring path(candle_dev_get_path(dev));

                    if (path == std::wstring(descriptor)) {
                        return std::make_unique<CandleWinUSBDevice>(dev);
                    }
                }
            }
        }
        candle_list_free(clist);
    }
    return std::unique_ptr<CANDevice>();
}

} // namespace usb
} // namespace rev