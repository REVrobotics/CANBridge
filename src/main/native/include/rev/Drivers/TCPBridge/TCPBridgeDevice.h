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

#include <map>
#include <string>
#include <locale>
#include <codecvt>

#include <winsock2.h>
#include <ws2tcpip.h>

#include "rev/Drivers/TCPBridge/TCPBridgeDeviceThread.h"
#include "rev/CANDevice.h"
#include "rev/CANMessage.h"
#include "rev/CANStatus.h"

namespace rev {
namespace usb {

class TCPBridgeDevice : public rev::usb::CANDevice {
public:
    TCPBridgeDevice() =delete;
    TCPBridgeDevice(std::string ip);
    virtual ~TCPBridgeDevice();

    virtual std::string GetName() const;
    virtual std::wstring GetDescriptor() const;

    virtual int GetId() const;

    virtual CANStatus SendCANMessage(const CANMessage& msg, int periodMs) override;
    virtual CANStatus RecieveCANMessage(CANMessage& msg, uint32_t messageID, uint32_t messageMask) override;
    virtual CANStatus OpenStreamSession(uint32_t* sessionHandle, CANBridge_CANFilter filter, uint32_t maxSize) override;
    virtual CANStatus CloseStreamSession(uint32_t sessionHandle);
    virtual CANStatus ReadStreamSession(uint32_t sessionHandle, HAL_CANStreamMessage* msgs, uint32_t messagesToRead, uint32_t* messagesRead, int32_t* status);

    virtual CANStatus GetCANStatus();

    virtual bool IsConnected();

    virtual bool Connect();
private:
    // candle_handle m_handle;
    struct sockaddr_in m_handle;
    SOCKET sockfd;
    SOCKET ConnectSocket;
    
    // TCPBridgeDeviceThread m_thread;
    std::string m_ip;
    std::string m_port;
    std::wstring m_descriptor;
    std::string m_name;
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    bool m_isConnected{false};
    uint8_t sendbuf[30];
    char recbuf[1024];
    uint8_t msgBuf[30];

    int parse_stream_res(uint8_t *buf, uint32_t* num_messages);
    int check_packet(uint8_t *buf, int *packet_size);
    void SerializeOpenStreamMessage(CANBridge_CANFilter filter, uint32_t maxSize);
    void SerializeReadStreamMessage(uint32_t messagesToRead);
    void SerializeRecieveCANMessage(uint32_t messageID, uint32_t messageMask);
    void serialize_send_msg_packet(const CANMessage& msg, int periodMs);
};

} // namespace usb
} // namespace rev
