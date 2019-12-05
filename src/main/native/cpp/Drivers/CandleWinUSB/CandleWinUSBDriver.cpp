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

#include "rev/Drivers/CandleWinUSB/CandleWinUSBDriver.h"
#include "rev/Drivers/CandleWinUSB/CandleWinUSBDevice.h"

#include <map>
#include <iostream>
#include <memory>

#include "candlelib/candle.h"

namespace rev {
namespace usb {

std::vector<CANDeviceDetail> CandleWinUSBDriver::GetDevices()
{
    // Search driver layer for devices
    candle_list_handle clist;
    uint8_t num_interfaces;
    candle_handle dev;
    std::vector<CANDeviceDetail> retval;

    if (candle_list_scan(&clist)) {
        if (candle_list_length(clist, &num_interfaces)) {
            for (uint8_t i=0; i<num_interfaces; i++) {
                if (candle_dev_get(clist, i, &dev)) {
                    std::string path = unicode_decode(candle_dev_get_path(dev));
                    std::string name(candle_dev_get_name(dev));
                    retval.push_back({path, name, this->GetName()});
                }
            }
        }
        candle_list_free(clist);
    }

    return retval;
}

std::unique_ptr<CANDevice> CandleWinUSBDriver::CreateDeviceFromDescriptor(const char* descriptor)
{
    // Search driver layer for devices
    candle_list_handle clist;
    uint8_t num_interfaces;
    candle_handle dev;

    if (candle_list_scan(&clist)) {
        if (candle_list_length(clist, &num_interfaces)) {
            for (uint8_t i=0; i<num_interfaces; i++) {
                if (candle_dev_get(clist, i, &dev)) {
                    std::string path = unicode_decode(candle_dev_get_path(dev));
                    if (path == std::string(descriptor)) {
                        try {
                            return std::make_unique<CandleWinUSBDevice>(dev);
                        } catch(...) {
                            // do nothing if it failed
                            std::cout << "Failed to create CandleWinUSB device: " << std::endl;
                        }
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

#else
typedef int __ISOWarning__CLEAR_;
#endif // _WIN32
