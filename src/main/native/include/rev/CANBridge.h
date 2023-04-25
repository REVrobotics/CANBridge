/*
 * Copyright (c) 2019 - 2020 REV Robotics
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

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CANBridge_Scan* c_CANBridge_ScanHandle;

/**
 * Scan for any available REV USB compatible devices. 
 * The scan results include only directly connected devices
 * and not detect devices on the rest of the CAN network. 
 * 
 * @return Handle to CANBridge Scan Data 
 */
c_CANBridge_ScanHandle CANBridge_Scan();

/**
 * Returns the number of devices discovered by scan. Includes
 * the device used as a bridge in the count of devices detected
 * 
 * @param handle previously opened c_CANBridge_ScanHandle
 * 
 * @return number of devices detected, including the bridge device
 */
int CANBridge_NumDevices(c_CANBridge_ScanHandle handle);

/**
 * Get the device descriptor used to register the device
 * to the WPILib HAL
 * 
 * @param index of the device in the scan
 * 
 * @return wchar_t pointer to  descriptor value. This becomes invalid after
 * a call to CANBridge_FreeScan()
 */
const char* CANBridge_GetDeviceDescriptor(c_CANBridge_ScanHandle handle, size_t index);

/**
 * Get the device name
 * 
 * @param index of the device in the scan
 * 
 * @return pointer to device name. This becomes invalid after
 * a call to CANBridge_FreeScan()
 */
const char* CANBridge_GetDeviceName(c_CANBridge_ScanHandle handle, size_t index);

/**
 * Get the driver name
 * 
 * @param index of the device in the scan
 * 
 * @return pointer to driver name. This becomes invalid after
 * a call to CANBridge_FreeScan()
 */
const char* CANBridge_GetDriverName(c_CANBridge_ScanHandle handle, size_t index);

int CANBridge_GetNumberOfErrors();

/**
 * Free the memory allocated by CANBridge_Scan()
 * 
 * @param handle of allocated scan
 * 
 */
void CANBridge_FreeScan(c_CANBridge_ScanHandle handle);

/**
 * Register the device with the WPILib HAL for any incoming or
 * outgoing CAN traffic that passes the filter created from the
 * message ID and mask.
 * 
 * Call this before calling CANBridge_FreeScan()
 * 
 * @param descriptor The device descriptor returned by 
 * CANBridge_GetDeviceDescriptor().
 * 
 * @param messageId the ID to match for this device
 * 
 * @param messageMask the bits of the messageId that must match
 * to be handled by this device. Pass 0 to match all messages.
 * 
 */
void CANBridge_RegisterDeviceToHAL(const char* descriptor, uint32_t messageId, uint32_t messageMask, int32_t* status);

/**
 * Unregister the device from the WPILib HAL.
 * 
 * Call this after scanning the device for other devices connected to
 * the CAN bus.
 * 
 * @param descriptor The device descriptor returned by 
 * CANBridge_GetDeviceDescriptor().
 * 
 * @param messageId the ID to match for this device
 * 
 */
void CANBridge_UnregisterDeviceFromHAL(const char* descriptor);

#ifdef __cplusplus
} // extern "C"
#endif
