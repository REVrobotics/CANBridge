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

#include "rev/RevUSB.h"

#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <algorithm>

#include "rev/CANDriver.h"
#include "rev/CANDevice.h"

#ifdef _WIN32
#include "rev/Drivers/CandleWinUSB/CandleWinUSBDriver.h"
#endif

#include <mockdata/CanData.h>

#ifdef __FRC_ROBORIO__
#error Not designed for FRC RoboRIO Projects! Requires HALSIM. For roboRIO projects use CAN.
#endif

struct RevUSB_Scan {
    std::vector<rev::usb::CANDeviceDetail> devices;
};

class RevUSB_CANFilter {
public:
    uint32_t messageId;
    uint32_t messageMask;
};

static const std::vector<rev::usb::CANDriver*> CANDriverList = {
#ifdef _WIN32
    new rev::usb::CandleWinUSBDriver()
#endif
};

static std::vector<std::pair<std::unique_ptr<rev::usb::CANDevice>, RevUSB_CANFilter>> CANDeviceList = {};

static int32_t RevUSB_StatusToHALError(rev::usb::CANStatus status) {
    // TODO: Map these to the actual HAL error codes
    return static_cast<int32_t>(status);
}

c_RevUSB_ScanHandle RevUSB_Scan()
{
    c_RevUSB_ScanHandle handle = new struct RevUSB_Scan;

    for (auto& driver : CANDriverList) {
        for (auto d : driver->GetDevices()) {
            handle->devices.push_back(d);
        }
    }

    return handle;
}

int RevUSB_NumDevices(c_RevUSB_ScanHandle handle)
{
    return handle->devices.size();
}

const wchar_t* RevUSB_GetDeviceDescriptor(c_RevUSB_ScanHandle handle, size_t index)
{
    if (index >= handle->devices.size()) {
        return NULL;
    }

    return handle->devices[index].descriptor.c_str();
}

const char* RevUSB_GetDeviceName(c_RevUSB_ScanHandle handle, size_t index)
{
    if (index >= handle->devices.size()) {
        return NULL;
    }

    return handle->devices[index].name.c_str();

}

const char* RevUSB_GetDriverName(c_RevUSB_ScanHandle handle, size_t index)
{
    if (index >= handle->devices.size()) {
        return NULL;
    }

    return handle->devices[index].driverName.c_str();
}

void RevUSB_FreeScan(c_RevUSB_ScanHandle handle)
{
    delete handle;
}

static bool RevUSB_ProcessMask(const RevUSB_CANFilter& filter, uint32_t id, uint32_t mask = 0) 
{
    return true;
}

void RevUSB_SendMessageCallback(const char* name, void* param,
                                            uint32_t messageID,
                                            const uint8_t* data,
                                            uint8_t dataSize, int32_t periodMs,
                                            int32_t* status)
{
    for (auto& dev : CANDeviceList) {
        if (RevUSB_ProcessMask(dev.second, messageID)) {
            auto stat = dev.first->SendCANMessage(rev::usb::CANMessage(messageID, data, dataSize), periodMs);
            *status = RevUSB_StatusToHALError(stat);
        }
    }
}

struct RevUSB_CANRecieve {
    rev::usb::CANMessage m_message;
    uint32_t timestamp;
    int32_t status;
};

static bool CANRecieveCompare(struct RevUSB_CANRecieve a, struct RevUSB_CANRecieve b)
{   
    return a.timestamp < b.timestamp;
} 

void RevUSB_ReceiveMessageCallback(
    const char* name, void* param, uint32_t* messageID, uint32_t messageIDMask,
    uint8_t* data, uint8_t* dataSize, uint32_t* timeStamp, int32_t* status)
{
    std::vector<struct RevUSB_CANRecieve> recieves;

    // 1) Recieve on all registered channels
    for (auto& dev : CANDeviceList) {
            struct RevUSB_CANRecieve msg;
            auto stat = dev.first->RecieveCANMessage(msg.m_message, messageIDMask, msg.timestamp);

        if (RevUSB_ProcessMask(dev.second, msg.m_message.GetMessageId(), messageIDMask)) {
            msg.status = RevUSB_StatusToHALError(stat);
            recieves.push_back(msg);
        }
    }

    if (recieves.size() == 0) {
        // TODO: what is the correct error return here
        *status = RevUSB_StatusToHALError(rev::usb::CANStatus::kError);
        return;
    }

    // 2) Return the newest message that does not have an error
    std::sort(recieves.begin(), recieves.end(), CANRecieveCompare);

    for (auto& recv : recieves) {
        if (recv.status == 0) {
            *timeStamp = recv.timestamp;
            *status = recv.status;
            *messageID = recv.m_message.GetMessageId();
            *dataSize = recv.m_message.GetSize();

            if(*dataSize > 8) {
                *dataSize = 8;
            }

            memcpy(data, recv.m_message.GetData(), *dataSize);
            return;
        }
    }

    // 3) Return an error if necessary
    *status = recieves[0].status;
}

void RevUSB_OpenStreamSessionCallback(
    const char* name, void* param, uint32_t* sessionHandle, uint32_t messageID,
    uint32_t messageIDMask, uint32_t maxMessages, int32_t* status)
{
    
}

void RevUSB_CloseStreamSessionCallback(const char* name,
                                                   void* param,
                                                   uint32_t sessionHandle)
{
    
}

void RevUSB_ReadStreamSessionCallback(
    const char* name, void* param, uint32_t sessionHandle,
    struct HAL_CANStreamMessage* messages, uint32_t messagesToRead,
    uint32_t* messagesRead, int32_t* status)
{
    
}

void RevUSB_GetCANStatusCallback(
    const char* name, void* param, float* percentBusUtilization,
    uint32_t* busOffCount, uint32_t* txFullCount, uint32_t* receiveErrorCount,
    uint32_t* transmitErrorCount, int32_t* status)
{
    
}

static std::vector<int32_t> LocalCallbackStore;

static void RevUSB_RegisterHAL()
{
    if (LocalCallbackStore.size() == 0) {
        LocalCallbackStore.push_back(HALSIM_RegisterCanSendMessageCallback(RevUSB_SendMessageCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanReceiveMessageCallback(RevUSB_ReceiveMessageCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanOpenStreamCallback(RevUSB_OpenStreamSessionCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanCloseStreamCallback(RevUSB_CloseStreamSessionCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanReadStreamCallback(RevUSB_ReadStreamSessionCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanGetCANStatusCallback(RevUSB_GetCANStatusCallback, NULL));
    }
}

void RevUSB_RegisterDeviceToHAL(const wchar_t* descriptor, uint32_t messageId, uint32_t messageMask)
{
    RevUSB_RegisterHAL();

    for (auto& driver : CANDriverList) {
        for (auto d : driver->GetDevices()) {
            if (d.descriptor.compare(descriptor) == 0) {
                RevUSB_CANFilter dev;
                dev.messageId = messageId;
                dev.messageMask = messageMask;
                CANDeviceList.push_back(std::make_pair(driver->CreateDeviceFromDescriptor(descriptor), dev));
                return;
            }
        }
    }
}
