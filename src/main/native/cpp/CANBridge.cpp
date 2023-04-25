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

#include "rev/CANBridge.h"

#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <algorithm>

#include "rev/CANDriver.h"
#include "rev/CANDevice.h"
#include "rev/CANMessage.h"
#include "rev/CANBridgeUtils.h"

#ifdef _WIN32
#include "rev/Drivers/CandleWinUSB/CandleWinUSBDriver.h"
#endif

#include "rev/Drivers/SerialPort/SerialDriver.h"

#include <hal/simulation/CanData.h>
#include <hal/CAN.h>

#ifdef __FRC_ROBORIO__
#error Not designed for FRC RoboRIO Projects! Requires HALSIM. For roboRIO projects use CAN.
#endif

struct CANBridge_Scan {
    std::vector<rev::usb::CANDeviceDetail> devices;
};


static const std::vector<rev::usb::CANDriver*> CANDriverList = {
#ifdef _WIN32
    new rev::usb::CandleWinUSBDriver(),
#endif
    new rev::usb::SerialDriver()
};

static std::vector<std::pair<std::unique_ptr<rev::usb::CANDevice>, rev::usb::CANBridge_CANFilter>> CANDeviceList = {};

static int32_t CANBridge_StatusToHALError(rev::usb::CANStatus status) {
    // TODO: Map these to the actual HAL error codes
    return static_cast<int32_t>(status);
}

c_CANBridge_ScanHandle CANBridge_Scan()
{
    c_CANBridge_ScanHandle handle = new struct CANBridge_Scan;

    for (auto& driver : CANDriverList) {
        for (auto d : driver->GetDevices()) {
            handle->devices.push_back(d);
        }
    }

    return handle;
}

int CANBridge_NumDevices(c_CANBridge_ScanHandle handle)
{
    return handle->devices.size();
}

const char* CANBridge_GetDeviceDescriptor(c_CANBridge_ScanHandle handle, size_t index)
{
    if (index >= handle->devices.size()) {
        return NULL;
    }

    return handle->devices[index].descriptor.c_str();
}

const char* CANBridge_GetDeviceName(c_CANBridge_ScanHandle handle, size_t index)
{
    if (index >= handle->devices.size()) {
        return NULL;
    }

    return handle->devices[index].name.c_str();

}

const char* CANBridge_GetDriverName(c_CANBridge_ScanHandle handle, size_t index)
{
    if (index >= handle->devices.size()) {
        return NULL;
    }

    return handle->devices[index].driverName.c_str();
}

int CANBridge_GetNumberOfErrors() 
{
    int err = 0;
    for (auto& dev : CANDeviceList) {
        err += dev.first->GetNumberOfErrors();
    }

    return err;
}

void CANBridge_FreeScan(c_CANBridge_ScanHandle handle)
{
    if (handle != NULL) {
        delete handle;
    }
}


void CANBridge_SendMessageCallback(const char* name, void* param,
                                            uint32_t messageID,
                                            const uint8_t* data,
                                            uint8_t dataSize, int32_t periodMs,
                                            int32_t* status)
{
    for (auto& dev : CANDeviceList) {
        if (rev::usb::CANBridge_ProcessMask(dev.second, messageID)) {
            auto stat = dev.first->SendCANMessage(rev::usb::CANMessage(messageID, data, dataSize), periodMs);
            *status = CANBridge_StatusToHALError(stat);
        }
    }
}

struct CANBridge_CANReceive {
    std::shared_ptr<rev::usb::CANMessage> m_message;
    int32_t status;
};

static bool CANReceiveCompare(struct CANBridge_CANReceive a, struct CANBridge_CANReceive b)
{   
    return rev::usb::CANMessageCompare(*a.m_message, *b.m_message);
} 

void CANBridge_ReceiveMessageCallback(
    const char* name, void* param, uint32_t* messageID, uint32_t messageIDMask,
    uint8_t* data, uint8_t* dataSize, uint32_t* timeStamp, int32_t* status)
{
    std::vector<struct CANBridge_CANReceive> receives;

    // 1) Receive on all registered channels
    for (auto& dev : CANDeviceList) {
        struct CANBridge_CANReceive msg;
        auto stat = dev.first->ReceiveCANMessage(msg.m_message, *messageID, messageIDMask);

        if (stat == rev::usb::CANStatus::kOk && rev::usb::CANBridge_ProcessMask(dev.second, msg.m_message->GetMessageId())) {
            msg.status = CANBridge_StatusToHALError(stat);
            receives.push_back(msg);
        }
    }

    if (receives.size() == 0) {
        // TODO: what is the correct error return here
        *status = HAL_ERR_CANSessionMux_MessageNotFound;
        return;
    }



    // 2) Return the newest message that does not have an error
    std::sort(receives.begin(), receives.end(), CANReceiveCompare);

    for (auto& recv : receives) {
        if (recv.status == 0 && recv.m_message->isNew()) {
            *timeStamp = recv.m_message->GetTimestampUs();
            *status = recv.status;
            *messageID = recv.m_message->GetMessageId();
            *dataSize = recv.m_message->GetSize();

            if(*dataSize > 8) {
                *dataSize = 8;
            }            

            memcpy(data, recv.m_message->GetData(), *dataSize);

            recv.m_message->setIsNew(false);

            return;
        }
    }

    // 3) Return an error if necessary
    *status = HAL_ERR_CANSessionMux_MessageNotFound;
}

/**
 * This needs to call the CandleWinUSBDevice.OpenStreamSession and register
 * a stream with the correct size buffer.
 */
void CANBridge_OpenStreamSessionCallback(
    const char* name, void* param, uint32_t* sessionHandle, uint32_t messageID,
    uint32_t messageIDMask, uint32_t maxMessages, int32_t* status)
{
    rev::usb::CANStatus stat = rev::usb::CANStatus::kError;
    for (auto& dev : CANDeviceList) {
        if (dev.first != nullptr) {
            stat = dev.first->OpenStreamSession(sessionHandle, {messageID, messageIDMask}, maxMessages);
        }
    }
    *status = (int32_t)stat;
}

void CANBridge_CloseStreamSessionCallback(const char* name, void* param, uint32_t sessionHandle)
{
    for (auto& dev : CANDeviceList) {        
        dev.first->CloseStreamSession(sessionHandle);
    } 
}


void CANBridge_ReadStreamSessionCallback(
    const char* name, void* param, uint32_t sessionHandle,
    struct HAL_CANStreamMessage* messages, uint32_t messagesToRead,
    uint32_t* messagesRead, int32_t* status)
{
    /** Need to classify behavior for status messages as well as handling
     * multiple devices opening multiple streams.
     */
    for (auto& dev : CANDeviceList) {
        auto stat = dev.first->ReadStreamSession(sessionHandle, messages, messagesToRead, messagesRead);
        *status = (int32_t)stat;
    }
}

void CANBridge_GetCANStatusCallback(
    const char* name, void* param, float* percentBusUtilization,
    uint32_t* busOffCount, uint32_t* txFullCount, uint32_t* receiveErrorCount,
    uint32_t* transmitErrorCount, int32_t* status)
{
    for (auto& dev : CANDeviceList) {
        auto stat = dev.first->GetCANDetailStatus(percentBusUtilization, busOffCount, txFullCount, receiveErrorCount, transmitErrorCount);
        *status = (int32_t)stat;
    }
}

static std::vector<int32_t> LocalCallbackStore;

static void CANBridge_RegisterHAL()
{
    if (LocalCallbackStore.size() == 0) {
        LocalCallbackStore.push_back(HALSIM_RegisterCanSendMessageCallback(CANBridge_SendMessageCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanReceiveMessageCallback(CANBridge_ReceiveMessageCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanOpenStreamCallback(CANBridge_OpenStreamSessionCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanCloseStreamCallback(CANBridge_CloseStreamSessionCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanReadStreamCallback(CANBridge_ReadStreamSessionCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanGetCANStatusCallback(CANBridge_GetCANStatusCallback, NULL));
    }
}

void CANBridge_RegisterDeviceToHAL(const char* descriptor, uint32_t messageId, uint32_t messageMask, int32_t* status)
{
    CANBridge_RegisterHAL();

    for (auto& driver : CANDriverList) {
        for (auto d : driver->GetDevices()) {
            if (d.descriptor.compare(descriptor) == 0) {
                rev::usb::CANBridge_CANFilter filter;
                filter.messageId = messageId;
                filter.messageMask = messageMask;
                auto device = driver->CreateDeviceFromDescriptor(descriptor);
                if (device != nullptr) {
                    CANDeviceList.push_back(std::make_pair(std::move(device), filter));
                    *status = (int32_t)rev::usb::CANStatus::kOk;
                    return;
                } else {
                    *status = (int32_t)rev::usb::CANStatus::kError;
                    return;
                }
            }
        }
    }
}

void CANBridge_UnregisterDeviceFromHAL(const char* descriptor) 
{
    std::vector<std::pair<std::unique_ptr<rev::usb::CANDevice>, rev::usb::CANBridge_CANFilter>>::const_iterator device = CANDeviceList.begin();
    for ( ; device != CANDeviceList.end(); ) {
        if (device->first.get() != nullptr && device->first->GetDescriptor().compare(descriptor) == 0) {
            device = CANDeviceList.erase(device);
            return;
        } else {
            ++device;
        }
    }
}


