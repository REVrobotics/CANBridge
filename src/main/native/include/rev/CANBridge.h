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
 * Returns the number of devices discovered by scan
 * 
 * @param previously opened c_CANBridge_ScanHandle
 * 
 * @return number of devices detected
 */
int CANBridge_NumDevices(c_CANBridge_ScanHandle handle);

/**
 * Get the device descriptor used to register the device
 * to the WPILib HAL
 * 
 * @param inverted The phase of the encoder
 * 
 * @return CANError.kOK if successful
 */
const wchar_t* CANBridge_GetDeviceDescriptor(c_CANBridge_ScanHandle handle, size_t index);
const char* CANBridge_GetDeviceName(c_CANBridge_ScanHandle handle, size_t index);
const char* CANBridge_GetDriverName(c_CANBridge_ScanHandle handle, size_t index);
void CANBridge_FreeScan(c_CANBridge_ScanHandle);

void CANBridge_RegisterDeviceToHAL(const wchar_t* descriptor, uint32_t messageId, uint32_t messageMask);

#ifdef __cplusplus
} // extern "C"
#endif
