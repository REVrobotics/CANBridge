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
#include <mutex>

#include "rev/CANDevice.h"
#include "rev/CANMessage.h"
#include "rev/CANStatus.h"
#include "rev/Drivers/TCPBridge/TCPBridgeMessages.h"

#include <asio.hpp>

namespace rev {
namespace usb {

class TCPBridgeDevice : public rev::usb::CANDevice {
public:
    TCPBridgeDevice() =delete;
    TCPBridgeDevice(const std::string host, const std::string port);
    virtual ~TCPBridgeDevice();

    virtual std::string GetName() const;
    virtual std::string GetDescriptor() const;

    virtual int GetId() const;

    virtual CANStatus SendCANMessage(const CANMessage& msg, int periodMs) override;
    virtual CANStatus RecieveCANMessage(CANMessage& msg, uint32_t messageID, uint32_t messageMask) override;
    virtual CANStatus OpenStreamSession(uint32_t* sessionHandle, CANBridge_CANFilter filter, uint32_t maxSize) override;
    virtual CANStatus CloseStreamSession(uint32_t sessionHandle) override;
    virtual CANStatus ReadStreamSession(uint32_t sessionHandle, HAL_CANStreamMessage* msgs, uint32_t messagesToRead, uint32_t* messagesRead, int32_t* status) override;
    virtual CANStatus GetCANStatus(float* percentBusUtilization, uint32_t* busOff, uint32_t* txFull, uint32_t* receiveErr, uint32_t* transmitErr, int32_t* status) override;

    virtual bool IsConnected();
    void Disconnect();
    virtual bool Connect();

private:
    std::string m_host;
    std::string m_port;
    asio::ip::address m_ip;

    asio::io_service m_ioservice;
    asio::ip::tcp::socket m_sock;

    std::string m_descriptor;
    std::string m_name;

    int m_verbosity = 5;
    bool m_isConnected{false};

    std::mutex sock_mutex;

    TCPBridgeMessages_t m_msg;

    bool SendMsg();
    bool Recv(void *buf, uint8_t bytesToRead);
    bool RecvMsg(TCPBridgeCommands cmd);
    size_t Read(size_t bytesToRead, size_t bufOffset = 0);

    void OpenStreamSession(uint32_t *handle, uint32_t messageId, uint32_t messageMask, uint32_t maxMessages, int32_t *status);
    void ReceiveMessage(uint32_t* messageId, uint32_t messageMask, uint8_t* data, uint8_t* dataSize, uint32_t* timeStamp, int32_t* status);
    void SendMessage(uint32_t messageID, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status);
    void SendReadStream(uint32_t maxMessages, canStreamer_readStream_t *response);
};

} // namespace usb
} // namespace rev
