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
#include "rev/Drivers/TCPBridge/TCPBridgePackets.h"

#include <iostream> //TODO: Remove
#include <thread>
#include <locale>
#include <codecvt>
#include <memory.h>

#include <mockdata/CanData.h>
#include <hal/CAN.h>

#include <stdio.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>

#pragma comment (lib,"ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

#define DEFAULT_PORT "8800"

namespace rev {
namespace usb {

TCPBridgeDevice::TCPBridgeDevice(std::string ip)
{
    m_ip = ip;
    if(Connect())
        m_isConnected = true;
}

TCPBridgeDevice::~TCPBridgeDevice()
{
    // printf("Closing socket\n");
    closesocket(ConnectSocket);
    WSACleanup();
}

bool TCPBridgeDevice::Connect()
{
    WSADATA wsaData;
    struct addrinfo *result = NULL, *ptr = NULL, hints;
    int iResult;
    char s[INET6_ADDRSTRLEN];

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return false;
    }

    // printf("Winsock initialized.\n");

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // printf("Getting address info\n");
    iResult = getaddrinfo(m_ip.c_str(), DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return false;
    }

    // Attempt to connect to an address until one succeeds
    // printf("Attempting to connect.\n");
    for(ptr=result; ptr != NULL ;ptr=ptr->ai_next) {
        // Create a SOCKET for connecting to server
        ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, 
            ptr->ai_protocol);
        if (ConnectSocket == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            return false;
        }

        // Connect to server.
        iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            closesocket(ConnectSocket);
            printf("connect failed with error: %d\n", iResult);
            ConnectSocket = INVALID_SOCKET;
            continue;
        }

        break;
    }

    freeaddrinfo(result);
    
    if (ConnectSocket == INVALID_SOCKET) {
        printf("Unable to connect to server!\n");
        WSACleanup();
        return false;
    }

    int iOptVal = 0;
    int iOptLen = sizeof (int);
    if(setsockopt(ConnectSocket, SOL_SOCKET, SO_SNDBUF, (char *) &iOptVal, iOptLen) == SOCKET_ERROR)
    {
        printf("getsockopt for SO_SNDBUF failed with error: %u\n", WSAGetLastError());
    } else
    {
        printf("SO_SNDBUF Value: %d\n", iOptVal);
    }

    bool bOptVal = true;
    int bOptLen = sizeof (bool);
    iResult = setsockopt(ConnectSocket, SOL_SOCKET, TCP_NODELAY, (char *) &bOptVal, bOptLen);
    if(iResult == SOCKET_ERROR) {
        printf("setsockopt for TCP_NODELAY failed with error: %u\n", WSAGetLastError());
    } else
    {
        printf("Set TCP_NODELAY: ON\n");
    }

    // void *addr;
    // std::string ipver;

    // // Get the pointer to the address itself, different fields in IPv4 and IPv6
    // if (ptr->ai_family == AF_INET)
    // {
    //     // IPv4
    //     struct sockaddr_in *ipv4 = (struct sockaddr_in *)ptr->ai_addr;
    //     addr = &(ipv4->sin_addr);
    //     ipver = "IPv4";
    // }
    // else
    // {
    //     // IPv6
    //     struct sockaddr_in6 *ipv6 = (struct sockaddr_in6 *)ptr->ai_addr;
    //     addr = &(ipv6->sin6_addr);
    //     ipver = "IPv6";
    // }

    // inet_ntop(ptr->ai_family, addr, (PSTR)s, sizeof(s));
    // std::cout << "Socket connected to " << ipver << ": " << s << std::endl;
    
    m_descriptor = converter.from_bytes(s);
    m_name = s;

    return true;
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

void TCPBridgeDevice::serialize_send_msg_packet(const CANMessage& msg, int periodMs) {
	sendbuf[0] = HEADER_TOKEN;
	sendbuf[1] = 22;
	sendbuf[2] = SEND_MSG;
	sendbuf[3] = 0;
	
    uint32_t messageID = msg.GetMessageId();
    memcpy(&sendbuf[4], &messageID, 4);
    memcpy(&sendbuf[8], &periodMs, 4);
	
	sendbuf[12] = msg.GetSize();
	
	memcpy(sendbuf + 13, msg.GetData(), msg.GetSize());
	
	sendbuf[21] = TRAILER_TOKEN;
}

CANStatus TCPBridgeDevice::SendCANMessage(const CANMessage& msg, int periodMs)
{
    std::cout << "Send message..." << std::endl;

    serialize_send_msg_packet(msg, periodMs);

    if(send(ConnectSocket, (char *) sendbuf, 22, 0) == SOCKET_ERROR) {
        printf("send failed: %d\n", WSAGetLastError());
    }

    return CANStatus::kOk;
}

void TCPBridgeDevice::SerializeRecieveCANMessage(uint32_t messageID, uint32_t messageMask)
{
    sendbuf[0] = HEADER_TOKEN;
	sendbuf[1] = 14;
	sendbuf[2] = RCV_MSG;
	sendbuf[3] = 0;
	
    memcpy(&sendbuf[4], &messageID, 4);
    memcpy(&sendbuf[8], &messageMask, 4);
	sendbuf[12] = 8;
	
	sendbuf[13] = TRAILER_TOKEN;
}

CANStatus TCPBridgeDevice::RecieveCANMessage(CANMessage& msg, uint32_t messageID, uint32_t messageMask)
{
    std::cout << "Receive message" << std::endl;

    int numbytes = 0;
    uint32_t messagesRead = 0;

    // send read stream packet
    std::cout << "Data size: " << msg.GetSize() << std::endl;
    SerializeRecieveCANMessage(messageID, messageMask);

    if(send(ConnectSocket, (char *) sendbuf, 14, 0) == SOCKET_ERROR) {
        printf("send failed: %d\n", WSAGetLastError());
    }

    // get response
    if((numbytes = recv(ConnectSocket, recbuf, 100, 0)) == SOCKET_ERROR) {
        printf("Receive failed: %d\n", WSAGetLastError());
    }

    if(parse_stream_res((uint8_t *)recbuf, &messagesRead) != 0) {
        printf("Error parsing messages packet\n");
    }

    if((numbytes = recv(ConnectSocket, recbuf, 1024, 0)) == SOCKET_ERROR) {
        printf("Receive failed: %d\n", WSAGetLastError());
    }

    int bytesRemaining = 0;
    int msg_idx = 0;
    uint32_t msgNum = 0;
    
    // TODO: make deserialize CAN message func
    uint8_t tmp[8];
    uint32_t timestamp;
    memcpy(&messageID, &recbuf[0], 4);
    memcpy(&timestamp, &recbuf[4], 4);
    memcpy(tmp, &recbuf[9], 8);
    // halMsg.dataSize = recbuf[8];
    // memcpy(halMsg.data, &recbuf[9], sizeof(halMsg.data));
    CANMessage tmsg = CANMessage(messageID, tmp, recbuf[8], timestamp);
    msg = tmsg;

    return CANStatus::kOk;
}

void TCPBridgeDevice::SerializeOpenStreamMessage(CANBridge_CANFilter filter, uint32_t maxSize)
{
    sendbuf[0] = HEADER_TOKEN;
	sendbuf[1] = 17;
	sendbuf[2] = OPEN_STREAM;
	sendbuf[3] = 0x00;

    memcpy(&sendbuf[4], &filter.messageId, 4);
    memcpy(&sendbuf[8], &filter.messageMask, 4);
    memcpy(&sendbuf[12], &maxSize, 4);
	
	sendbuf[16] = TRAILER_TOKEN;
}

CANStatus TCPBridgeDevice::OpenStreamSession(uint32_t* sessionHandle, CANBridge_CANFilter filter, uint32_t maxSize)
{
    std::cout << "Opening Stream:" << std::endl;
    std::cout << "\tMessage ID: " << filter.messageId << std::endl;
    std::cout << "\tMessage Mask: " << filter.messageMask << std::endl;

    SerializeOpenStreamMessage(filter, maxSize);

    if(send(ConnectSocket, (char *) sendbuf, 100, 0) == SOCKET_ERROR ) {
        printf("send failed: %d\n", WSAGetLastError());
    }

    return CANStatus::kOk;
}
CANStatus TCPBridgeDevice::CloseStreamSession(uint32_t sessionHandle)
{
    std::cout << "Close stream not implemented yet..." << std::endl;

    return CANStatus::kOk;
}

int TCPBridgeDevice::check_packet(uint8_t *buf, int *packet_size) {
    if(buf[0] != HEADER_TOKEN) {
        printf("Invalid command packet rec'd\n");
        return -1;
    }

    *packet_size = buf[1];

    if((*packet_size > MAXBUFFERSIZE) || (buf[*packet_size-1] != TRAILER_TOKEN)) {
        printf("Invalid packet size rec'd: %d\n", *packet_size-1);
        return -2;
    }

    return 0;
}

int TCPBridgeDevice::parse_stream_res(uint8_t *buf, uint32_t* num_messages) {
	int packet_size = 0;
	if(check_packet(buf, &packet_size) != 0) {
		printf("Packet failed checking\n");
		return -1;
	}
	
	uint32_t tmp = 0;
	tmp = buf[4];
	tmp |= (buf[5] << 8);
	tmp |= (buf[6] << 16);
	tmp |= (buf[7] << 24);
		
	*num_messages = tmp;
	return 0;
}

void TCPBridgeDevice::SerializeReadStreamMessage(uint32_t messagesToRead)
{
    sendbuf[0] = HEADER_TOKEN;
	sendbuf[1] = 9; // size
	sendbuf[2] = READ_STREAM;
	sendbuf[3] = 0x00;
	
	memcpy(&sendbuf[4], &messagesToRead, 4);
	sendbuf[8] = TRAILER_TOKEN;
}

CANStatus TCPBridgeDevice::ReadStreamSession(uint32_t sessionHandle,
                                             struct HAL_CANStreamMessage* msgs,
                                             uint32_t messagesToRead,
                                             uint32_t* messagesRead,
                                             int32_t* status)
{
    int numbytes = 0;
    // std::cout << "Reading stream..." << std::endl;

    // send read stream packet
    SerializeReadStreamMessage(messagesToRead);

    if(send(ConnectSocket, (char *) sendbuf, 17, 0) == SOCKET_ERROR) {
        printf("send failed: %d\n", WSAGetLastError());
    }

    // get response
    if((numbytes = recv(ConnectSocket, recbuf, 100, 0)) == SOCKET_ERROR) {
        printf("Receive failed: %d\n", WSAGetLastError());
    }

    if(parse_stream_res((uint8_t *)recbuf, messagesRead) != 0) {
        printf("Error parsing messages packet\n");
    }
    
    // printf("num_messages read: %d\n", *messagesRead);

    // wait to get all messages
    int bytesRemaining = 0;
    int msg_idx = 0;
    uint32_t msgNum = 0;
    while(msgNum < *messagesRead) {
        // printf("Receiving messages...\n");
        if((numbytes = recv(ConnectSocket, recbuf, 1024, 0)) == SOCKET_ERROR) {
            printf("Receive failed: %d\n", WSAGetLastError());
        }
        
        bytesRemaining += numbytes;
        
        while(bytesRemaining > 0) {
            // printf("Message: %d\n", msgNum);
            memcpy(msgBuf, recbuf + msg_idx, 17);
            
            // TODO: make deserialize CAN message func
            HAL_CANStreamMessage halMsg;
            memcpy(&halMsg.messageID, &msgBuf[0], 4);
            memcpy(&halMsg.timeStamp, &msgBuf[4], 4);
            halMsg.dataSize = msgBuf[8];
            memcpy(halMsg.data, &msgBuf[9], sizeof(halMsg.data));
            
            msgs[msgNum] = halMsg;
            
            msg_idx += 17;
            bytesRemaining -= 17;
            msgNum++;
        }
        msg_idx = 0;
    }

    return CANStatus::kOk;
}

CANStatus TCPBridgeDevice::GetCANStatus()
{
    std::cout << "Get status!" << std::endl;
    return CANStatus::kOk;
}

bool TCPBridgeDevice::IsConnected()
{
    return m_isConnected;
}


} // namespace usb
} // namespace rev
