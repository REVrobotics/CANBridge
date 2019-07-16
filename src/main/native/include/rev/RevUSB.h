#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct RevUSB_Scan* c_RevUSB_ScanHandle;

//List available SPARK MAX devices
c_RevUSB_ScanHandle RevUSB_Scan();
int RevUSB_NumDevices(c_RevUSB_ScanHandle handle);
const wchar_t* RevUSB_GetDeviceName(c_RevUSB_ScanHandle handle, size_t index);
void RevUSB_FreeScan(c_RevUSB_ScanHandle);

void RevUSB_RegisterDeviceToHAL(const wchar_t* descriptor, uint32_t messageId, uint32_t messageMask);

#ifdef __cplusplus
} // extern "C"
#endif
