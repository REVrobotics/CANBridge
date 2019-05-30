#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct USBSparkMax_Scan* c_USBSparkMax_ScanHandle;

//List available SPARK MAX devices
c_USBSparkMax_ScanHandle USBSparkMax_Scan();
int USBSparkMax_NumDevices(c_USBSparkMax_ScanHandle handle);
const wchar_t* USBSparkMax_GetDeviceName(c_USBSparkMax_ScanHandle handle, int index);
void USBSparkMax_FreeScan(c_USBSparkMax_ScanHandle);

void USBSparkMax_RegisterDeviceToHAL(const wchar_t* descriptor, uint32_t messageId, uint32_t messageMask);

#ifdef __cplusplus
} // extern "C"
#endif
