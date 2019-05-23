#include "USBSparkMax.h"

#include <string>
#include <vector>
#include <iostream>

#include "CANDriver.h"
#include "CandleWinUSBDriver.h"

#include <mockdata/CanData.h>

struct USBSparkMax_Scan {
    std::vector<std::wstring*> devices;
};

static const std::vector<rev::usb::CANDriver*> CANDriverList = {
    new rev::usb::CandleWinUSBDriver()
};

static std::vector<rev::usb::CANDevice> CANDeviceList = {};

c_USBSparkMax_ScanHandle USBSparkMax_Scan()
{
    c_USBSparkMax_ScanHandle handle = new struct USBSparkMax_Scan;
    int i=0;

    for (auto& driver : CANDriverList) {
        for (const auto& d : driver->GetDevices()) {
            auto tmp = new std::wstring(d->GetDescriptor());
            handle->devices.push_back(tmp);
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

    return handle->devices[index]->c_str();
}

void USBSparkMax_Close(c_USBSparkMax_ScanHandle handle)
{
    for (auto d : handle->devices) {
        delete d;
    }
    delete handle;
}

void USBSparkMax_RegisterDeviceToHAL(const wchar_t* name)
{

}

void USBSparkMax_SendMessageCallback(const char* name, void* param,
                                            uint32_t messageID,
                                            const uint8_t* data,
                                            uint8_t dataSize, int32_t periodMs,
                                            int32_t* status)
{
    
}

void USBSparkMax_ReceiveMessageCallback(
    const char* name, void* param, uint32_t* messageID, uint32_t messageIDMask,
    uint8_t* data, uint8_t* dataSize, uint32_t* timeStamp, int32_t* status)
{
    
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

void USBSparkMax_RegisterDeviceToHAL(const char* name)
{
    HALSIM_RegisterCanSendMessageCallback(USBSparkMax_SendMessageCallback, NULL);
    HALSIM_RegisterCanReceiveMessageCallback(USBSparkMax_ReceiveMessageCallback, NULL);
    HALSIM_RegisterCanOpenStreamCallback(USBSparkMax_OpenStreamSessionCallback, NULL);
    HALSIM_RegisterCanCloseStreamCallback(USBSparkMax_CloseStreamSessionCallback, NULL);
    HALSIM_RegisterCanReadStreamCallback(USBSparkMax_ReadStreamSessionCallback, NULL);
    HALSIM_RegisterCanGetCANStatusCallback(USBSparkMax_GetCANStatusCallback, NULL);
}
