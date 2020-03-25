/*
 * Copyright (c) 2018-2019 REV Robotics
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

#include "gtest/gtest.h"
#include <iostream>

#include "rev/CANBridge.h"
// #include "rev/Drivers/CandleWinUSB/CandleWinUSBDevice.h"
// #include "rev/Drivers/CandleWinUSB/CandleWinUSBDriver.h"
// #include "rev/Drivers/SerialPort/SerialDevice.h"
// #include "rev/Drivers/SerialPort/SerialDriver.h"
#include "rev/CANMessage.h"

/**
 * Returns the number of devices seen on the bus, will only register the first device
 */
TEST(CANMessageTest, SendAndReceive) {
    auto handle = CANBridge_Scan();
    int32_t status = 0;

    int numDevices = CANBridge_NumDevices(handle);

    std::cout << "Found " << numDevices << " Devices: " << std::endl;

    for (int i=0;i<numDevices;i++) {
        std::cout << "\t" << CANBridge_GetDeviceName(handle, i) << " - " << CANBridge_GetDeviceDescriptor(handle, i) << std::endl;
    }

    if (numDevices > 0) {
        std::cout << "Registering single device to HAL" << std::endl;
        CANBridge_RegisterDeviceToHAL(CANBridge_GetDeviceDescriptor(handle, 0), 0, 0, &status);
        
        // auto device = handle->devices[0];

        // rev::usb::CANMessage msg;
        // uint32_t messageID = 33891849;
        // uint32_t messageMask = 0;
        // device->ReceiveCANMessage(msg, messageID, messageMask);
    }

    EXPECT_EQ(numDevices, 1);

    CANBridge_UnregisterDeviceFromHAL(CANBridge_GetDeviceDescriptor(handle, 0));

    CANBridge_FreeScan(handle);
}
