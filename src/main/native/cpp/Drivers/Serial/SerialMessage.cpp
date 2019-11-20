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

#include <stdint.h>
#include <iostream>

#include "rev/Drivers/SerialPort/SerialMessage.h"

// Constant defines of valid message IDS for the serial communication
const int CMD_API_CLEAR_FAULTS = 0x06E;
const int CMD_API_DRV_STAT = 0x06A;
const int CMD_API_BURN_FLASH = 0x072;
const int CMD_API_SET_FOLLOWER = 0x073;
const int CMD_API_FACTORY_DEFAULT = 0x074;
const int CMD_API_FACTORY_RESET = 0x075;

const int CMD_API_FIRMWARE = 0x098;

const int CMD_API_SWDL_BOOTLOADER = 0x1FF;
const int CMD_API_SWDL_DATA = 0x09C;
const int CMD_API_SWDL_CHKSUM = 0x09D;
const int CMD_API_SWDL_RETRANSMIT = 0x09E;

const int ID_SIZE = 13;



const int ValidIds[ID_SIZE] = {
    CMD_API_CLEAR_FAULTS,
    CMD_API_DRV_STAT,
    CMD_API_BURN_FLASH,
    CMD_API_SET_FOLLOWER,
    CMD_API_FACTORY_DEFAULT,
    CMD_API_FACTORY_RESET,
    
    CMD_API_FIRMWARE,

    CMD_API_SWDL_BOOTLOADER,
    CMD_API_SWDL_DATA,
    CMD_API_SWDL_CHKSUM,
    CMD_API_SWDL_RETRANSMIT,

    // Reset into DFU Mode command
    0x010,

    CMD_API_PARAM_ACCESS
};

bool IsValidSerialMessageId(uint16_t apiId) {

    for (int i = 0; i < ID_SIZE; i++) {
        if (ValidIds[i] == apiId) {
            return true;
        }
    }
    
    return false;
}

bool IsConfigParameter(uint16_t apiId) {
    if (apiId >= CMD_API_PARAM_ACCESS) {
        return true;
    }

    return false;
}

bool IsLegacyGetParam(uint32_t msgId) {
    return ((msgId & GET_CONFIG_PARAM) == GET_CONFIG_PARAM) || ((msgId & SET_CONFIG_PARAM) == SET_CONFIG_PARAM);
}

