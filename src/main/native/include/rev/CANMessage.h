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

#include <string>
#include <cstring>

#include <cstdint>
#include <cstddef>
#include <iostream>
#include <chrono>

namespace rev {
namespace usb {

enum class CANDeviceType {
    deviceBroadcast=0,
    robotController,
    motorController,
    relayController,
    gyroSensor,
    accelerometerSensor,
    ultrasonicSensor,
    gearToothSensor,
    powerDistribution,
    pneumaticsController,
    miscCANDevice,
    IOBreakout,
    dev_rsvd12,dev_rsvd13,dev_rsvd14,dev_rsvd15,
    dev_rsvd16,dev_rsvd17,dev_rsvd18,dev_rsvd19,
    dev_rsvd20,dev_rsvd21,dev_rsvd22,dev_rsvd23,
    dev_rsvd24,dev_rsvd25,dev_rsvd26,dev_rsvd27,
    dev_rsvd28,dev_rsvd29,dev_rsvd30,
    firmwareUpdate=31
};

enum class CANManufacturer {
    manufacturerBroadcast=0,
    NI=1,
    LM=2,
    DEKA=3,
    CTRE=4,
    REV=5,
    Grapple=6,
    MindSensors=7,
    TeamUse=8
};

class CANMessage {
public:
    CANMessage() : m_data{0}, m_size(0), m_messageId(0) { }
    CANMessage(uint32_t messageId, const uint8_t* data, uint8_t dataSize, uint32_t timestampUs = 0) :
        m_size(dataSize), m_messageId(messageId) {

            m_timestamp = (timestampUs != 0) ? timestampUs : std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
            std::memset(m_data, 0, 8);
            std::memcpy(m_data, data, dataSize > 8 ? 8 : dataSize);
    }

    ~CANMessage() = default;

    size_t Serialize(uint8_t *buffer, size_t bufSize) {
        int totalLen = bufSize > m_size ? m_size : bufSize;

        if (totalLen < 4) {
            return m_size;
        }

        std::memcpy(buffer, (uint8_t*)&m_messageId, sizeof(m_messageId));

        totalLen -= sizeof(m_messageId);
        buffer += sizeof(m_messageId);

        if (totalLen < 0) {
            return m_size;
        }

        std::memcpy(buffer, m_data, totalLen < 8 ? totalLen : 8);
        return m_size;
    }

    static uint32_t CreateCANId(uint16_t deviceId, uint16_t apiId, CANDeviceType type, CANManufacturer manu) {
        uint32_t retval = 0;
        retval |= deviceId;
        retval |= (apiId & 0xCFF) << 6;
        retval |= (static_cast<uint32_t>(manu) & 0xFF) << 16;
        return retval | (static_cast<uint32_t>(type) & 0x1F) << 24;
    }

    uint32_t GetMessageId() const {
        return m_messageId;
    }

    uint16_t GetDeviceId() const {
        return m_messageId & 0x3F;
    }

    uint16_t GetApiId() const {
        return static_cast<uint16_t>((m_messageId & 0xFFC0) >> 6);
    }

    CANDeviceType GetDeviceType() const {
        return static_cast<CANDeviceType>(((m_messageId & 0x1F000000) >> 24));
    }
    CANManufacturer GetManufacturer() const {
        return static_cast<CANManufacturer>(((m_messageId & 0x00FF0000) >> 16));
    }

    const uint8_t* GetData() const {
        return m_data;
    }

    uint8_t GetSize() const {
        return m_size;
    }

    bool IsEmpty() const {
        for (int i = 0; i < 8; i++) {
            if (m_data[i] != 0) {
                return false;
            }
        }
        return true;
    }

    uint32_t GetTimestampUs() const {
        return m_timestamp;
    }

    friend std::ostream& operator<<(std::ostream &os, const CANMessage& m) { 
        os << "Message Id: " << (int)m.GetMessageId() << " Size: " << (int)m.GetSize() << " Data: ";
        auto data = m.GetData();
        for (int i = 0; i < 8; i++) {
             os << std::hex << (int)(data[i]) << "_";
        }

        return os;
    }

   

private:
    uint8_t m_data[8];
    uint8_t m_size;
    uint32_t m_messageId;
    uint32_t m_timestamp;
};

} // namespace usb
} // namespace rev

//std::ostream& operator<<(std::ostream &os, const rev::usb::CANMessage& m) { 
//    return os << "Message Id: " << m.GetMessageId() << " Size: " << m.GetSize() << " Data: " << std::hex << m.GetData();
//}
