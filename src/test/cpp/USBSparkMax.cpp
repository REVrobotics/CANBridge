#include "USBSparkMax.h"

#include <string>
#include <vector>
#include <iostream>

#include "CANDriver.h"
#include "CANDevice.h"
#include "CandleWinUSBDriver.h"

#include <mockdata/CanData.h>

struct USBSparkMax_Scan {
    std::vector<std::wstring> devices;
};

static const std::vector<rev::usb::CANDriver*> CANDriverList = {
    new rev::usb::CandleWinUSBDriver()
};

static std::vector<std::unique_ptr<rev::usb::CANDevice>> CANDeviceList = {};

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

void USBSparkMax_Close(c_USBSparkMax_ScanHandle handle)
{
    delete handle;
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

void USBSparkMax_RegisterDeviceToHAL(const wchar_t* descriptor)
{
    c_USBSparkMax_ScanHandle handle = new struct USBSparkMax_Scan;
    int i=0;
    USBSparkMax_RegisterHAL();

    for (auto& driver : CANDriverList) {
        for (auto d : driver->GetDevices()) {
            if (d.compare(descriptor) == 0) {
                CANDeviceList.push_back(driver->CreateDeviceFromDescriptor(descriptor));
                return;
            }
        }
    }
}
