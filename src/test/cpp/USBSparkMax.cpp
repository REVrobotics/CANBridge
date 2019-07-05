#include "USBSparkMax.h"

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

#include "CANDriver.h"
#include "CANDevice.h"
#include "CandleWinUSBDriver.h"

#include <mockdata/CanData.h>

struct USBSparkMax_Scan {
    std::vector<std::wstring> devices;
};

class USBSparkMax_CANFilter {
public:
    uint32_t messageId;
    uint32_t messageMask;
};

static const std::vector<rev::usb::CANDriver*> CANDriverList = {
    new rev::usb::CandleWinUSBDriver()
};

static std::vector<std::pair<std::unique_ptr<rev::usb::CANDevice>, USBSparkMax_CANFilter>> CANDeviceList = {};

static int32_t USBSparkMax_StatusToHALError(rev::usb::CANStatus status) {
    // TODO: Map these to the actual HAL error codes
    return static_cast<int32_t>(status);
}

c_USBSparkMax_ScanHandle USBSparkMax_Scan()
{
    c_USBSparkMax_ScanHandle handle = new struct USBSparkMax_Scan;
    int i=0;

    for (auto& driver : CANDriverList) {
        for (auto d : driver->GetDevices()) {
            handle->devices.push_back(d);
        }
    }

    return handle;
}

int USBSparkMax_NumDevices(c_USBSparkMax_ScanHandle handle)
{
    return handle->devices.size();
}

const wchar_t* USBSparkMax_GetDeviceName(c_USBSparkMax_ScanHandle handle, int index)
{
    if (index >= handle->devices.size()) {
        return NULL;
    }

    return handle->devices[index].c_str();
}

void USBSparkMax_FreeScan(c_USBSparkMax_ScanHandle handle)
{
    delete handle;
}

static bool USBSparkMax_ProcessMask(const USBSparkMax_CANFilter& filter, uint32_t id, uint32_t mask = 0) 
{
    return true;
}

void USBSparkMax_SendMessageCallback(const char* name, void* param,
                                            uint32_t messageID,
                                            const uint8_t* data,
                                            uint8_t dataSize, int32_t periodMs,
                                            int32_t* status)
{
    for (auto& dev : CANDeviceList) {
        if (USBSparkMax_ProcessMask(dev.second, messageID)) {
            auto stat = dev.first->SendCANMessage(rev::usb::CANMessage(messageID, data, dataSize), periodMs);
            *status = USBSparkMax_StatusToHALError(stat);
        }
    }
}

struct USBSparkMax_CANRecieve {
    rev::usb::CANMessage m_message;
    uint32_t timestamp;
    int32_t status;
};

static bool CANRecieveCompare(struct USBSparkMax_CANRecieve a, struct USBSparkMax_CANRecieve b)
{   
    return a.timestamp < b.timestamp;
} 

void USBSparkMax_ReceiveMessageCallback(
    const char* name, void* param, uint32_t* messageID, uint32_t messageIDMask,
    uint8_t* data, uint8_t* dataSize, uint32_t* timeStamp, int32_t* status)
{
    std::vector<struct USBSparkMax_CANRecieve> recieves;

    // 1) Recieve on all registered channels
    for (auto& dev : CANDeviceList) {
            struct USBSparkMax_CANRecieve msg;
            auto stat = dev.first->RecieveCANMessage(msg.m_message, messageIDMask, msg.timestamp);

        if (USBSparkMax_ProcessMask(dev.second, msg.m_message.GetMessageId(), messageIDMask)) {
            msg.status = USBSparkMax_StatusToHALError(stat);
            recieves.push_back(msg);
        }
    }

    if (recieves.size() == 0) {
        // TODO: what is the correct error return here
        *status = USBSparkMax_StatusToHALError(rev::usb::CANStatus::kError);
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

void USBSparkMax_OpenStreamSessionCallback(
    const char* name, void* param, uint32_t* sessionHandle, uint32_t messageID,
    uint32_t messageIDMask, uint32_t maxMessages, int32_t* status)
{
    
}

void USBSparkMax_CloseStreamSessionCallback(const char* name,
                                                   void* param,
                                                   uint32_t sessionHandle)
{
    
}

void USBSparkMax_ReadStreamSessionCallback(
    const char* name, void* param, uint32_t sessionHandle,
    struct HAL_CANStreamMessage* messages, uint32_t messagesToRead,
    uint32_t* messagesRead, int32_t* status)
{
    
}

void USBSparkMax_GetCANStatusCallback(
    const char* name, void* param, float* percentBusUtilization,
    uint32_t* busOffCount, uint32_t* txFullCount, uint32_t* receiveErrorCount,
    uint32_t* transmitErrorCount, int32_t* status)
{
    
}

static std::vector<int32_t> LocalCallbackStore;

static void USBSparkMax_RegisterHAL()
{
    if (LocalCallbackStore.size() == 0) {
        LocalCallbackStore.push_back(HALSIM_RegisterCanSendMessageCallback(USBSparkMax_SendMessageCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanReceiveMessageCallback(USBSparkMax_ReceiveMessageCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanOpenStreamCallback(USBSparkMax_OpenStreamSessionCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanCloseStreamCallback(USBSparkMax_CloseStreamSessionCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanReadStreamCallback(USBSparkMax_ReadStreamSessionCallback, NULL));
        LocalCallbackStore.push_back(HALSIM_RegisterCanGetCANStatusCallback(USBSparkMax_GetCANStatusCallback, NULL));
    }
}

void USBSparkMax_RegisterDeviceToHAL(const wchar_t* descriptor, uint32_t messageId, uint32_t messageMask)
{
    c_USBSparkMax_ScanHandle handle = new struct USBSparkMax_Scan;
    int i=0;
    USBSparkMax_RegisterHAL();

    for (auto& driver : CANDriverList) {
        for (auto d : driver->GetDevices()) {
            if (d.compare(descriptor) == 0) {
                USBSparkMax_CANFilter dev;
                dev.messageId = messageId;
                dev.messageMask = messageMask;
                CANDeviceList.push_back(std::make_pair(driver->CreateDeviceFromDescriptor(descriptor), dev));
                return;
            }
        }
    }
}
