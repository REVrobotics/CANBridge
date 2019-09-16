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

#include "rev/Drivers/TCPBridge/TCPBridgeDevice.h"

#include <iostream> //TODO: Remove
#include <thread>

#include <mockdata/CanData.h>
#include <hal/CAN.h>

#include<stdio.h>
#include<winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>

#pragma comment(lib,"ws2_32.lib") //Winsock Library
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

#define DEFAULT_PORT "8800"

namespace rev {
namespace usb {

// static void close_candle_dev(candle_handle hDev)
// {
//     // candle_dev_close(hDev);
//     // candle_dev_free(hDev);
// }

// TCPBridgeDevice::TCPBridgeDevice(const char *ip) :
//     m_thread(ip)
TCPBridgeDevice::TCPBridgeDevice(const char *ip)
{
    WSADATA wsaData;
    SOCKET ConnectSocket = INVALID_SOCKET;
    struct addrinfo *result = NULL, *ptr = NULL, hints;
    int iResult;
    char s[INET6_ADDRSTRLEN];

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return;
    }

    printf("Winsock initialized.\n");

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    iResult = getaddrinfo(ip, DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return;
    }

    // Attempt to connect to an address until one succeeds
    for(ptr=result; ptr != NULL ;ptr=ptr->ai_next) {

        // Create a SOCKET for connecting to server
        ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, 
            ptr->ai_protocol);
        if (ConnectSocket == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            return;
        }

        // Connect to server.
        iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            closesocket(ConnectSocket);
            ConnectSocket = INVALID_SOCKET;
            continue;
        }

        break;
    }

    freeaddrinfo(result);

    void *addr;
    std::string ipver;

    // Get the pointer to the address itself, different fields in IPv4 and IPv6
    if (ptr->ai_family == AF_INET)
    {
        // IPv4
        struct sockaddr_in *ipv4 = (struct sockaddr_in *)ptr->ai_addr;
        addr = &(ipv4->sin_addr);
        ipver = "IPv4";
    }
    else
    {
        // IPv6
        struct sockaddr_in6 *ipv6 = (struct sockaddr_in6 *)ptr->ai_addr;
        addr = &(ipv6->sin6_addr);
        ipver = "IPv6";
    }
    
    if (ConnectSocket == INVALID_SOCKET) {
        printf("Unable to connect to server!\n");
        WSACleanup();
        return;
    }

    // inet_ntop(ptr->ai_family, ptr->ai_addr, s, sizeof s);
    inet_ntop(ptr->ai_family, addr, (PSTR)s, sizeof(s));
    std::cout << "Socket connected to " << ipver << ": " << s << std::endl;
    // printf("Socket Connected!");
    
    // m_descriptor = s;
    m_name = s;
    // m_thread.Start();
}

TCPBridgeDevice::~TCPBridgeDevice()
{
    // m_thread.Stop();
    
    // candle_channel_stop(m_handle, CANDLE_DEFAULT_CHANNEL);
    // close_candle_dev(m_handle);
}

std::string TCPBridgeDevice::GetName() const
{
    return m_name;
}


std::wstring TCPBridgeDevice::GetDescriptor() const
{
    return m_descriptor;
}

int TCPBridgeDevice::GetId() const
{
    return 0;
}

CANStatus TCPBridgeDevice::SendCANMessage(const CANMessage& msg, int periodMs)
{
    // auto data = msg.GetData();
    // m_thread.EnqueueMessage(msg, periodMs);
    return CANStatus::kOk;
}

CANStatus TCPBridgeDevice::RecieveCANMessage(CANMessage& msg, uint32_t messageID, uint32_t messageMask)
{
    // // parse through the keys, find the messges the match, and return it
    // std::map<uint32_t, CANMessage> messages;
    // m_thread.RecieveMessage(messages);

    // CANMessage mostRecent;
    // for (auto m : messages) {
    //     if (CANBridge_ProcessMask({m.second.GetMessageId(), 0}, m.first) && CANBridge_ProcessMask({messageID, messageMask}, m.first)) {
    //         std::cout << ">> mask:  " << (messageMask & messageID) << " == " << (messageMask & m.first) << std::endl;

    //         mostRecent = m.second;
            
    //     }
    // }
    // msg = mostRecent;

    // auto data = msg.GetData();
    // std::cout << "3) msg id: " << (int)msg.GetMessageId() << " data: ";
    // for (int i = 0; i < 8; i++) {
    //     std::cout << std::hex << (int)(data[i]) << "_";
    // }
    // std::cout << "\n";


    return CANStatus::kOk;
}

CANStatus TCPBridgeDevice::OpenStreamSession(uint32_t* sessionHandle, CANBridge_CANFilter filter, uint32_t maxSize)
{
    // Register the stream with the correct buffer size
    // m_thread.OpenStream(sessionHandle, filter, maxSize);
    
    return CANStatus::kOk;
}
CANStatus TCPBridgeDevice::CloseStreamSession(uint32_t sessionHandle)
{
    // m_thread.CloseStream(sessionHandle);
    return CANStatus::kOk;
}
CANStatus TCPBridgeDevice::ReadStreamSession(uint32_t sessionHandle, struct HAL_CANStreamMessage* msgs, uint32_t messagesToRead, uint32_t* messagesRead, int32_t* status)
{
    // m_thread.ReadStream(sessionHandle, msgs, messagesToRead, messagesRead);
    // status = static_cast<int32_t>(CANStatus::kOk);    
    return CANStatus::kOk;
}

CANStatus TCPBridgeDevice::GetCANStatus()
{
    return CANStatus::kOk;
}

bool TCPBridgeDevice::IsConnected()
{
    return true;
}


} // namespace usb
} // namespace rev
