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

#include <rev/CANBridgeUtils.h>

#include <locale>
#include <codecvt>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace rev {
namespace usb {

void convert_wstring_to_string(const std::wstring& in, std::string& out)
{
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    out = converter.to_bytes(in.c_str());
}

void convert_string_to_wstring(const std::string& in, std::wstring& out) {
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    out = converter.from_bytes(in.c_str());
}

void convert_candle_to_wstring(const std::wstring& in, std::wstring& out) {
    std::wstring tmp;
    
    for (uint32_t i = 0; i < in.size(); i++) {
        std::stringstream ss_in, ss_out;
        std::string s;
        
        ss_in << "0x" << std::setfill('0') << std::setw(4) << std::hex << (int)in.at(i);
        // std::cout << "in: " << ss_in.str() << std::endl;
        uint32_t raw = std::stoi(ss_in.str(), 0, 16);
        ss_out << "0x" << std::setfill('0') << std::setw(2) << std::hex << (int)(raw & 0xFF); 
        // std::cout << "out: " << ss_out.str() << std::endl;
        s = ss_out.str();

        int c = std::stoi(s, 0, 16);
        tmp += static_cast<wchar_t>(c);
        ss_out.str("");
        ss_out << "0x" << std::setfill('0') << std::setw(2) << std::hex << (int)(raw >> 8);
        // std::cout << "out: " << ss_out.str() << std::endl;
        s = ss_out.str();

        c = std::stoi(s, 0, 16);
        if (c != 0x00) {
            tmp += static_cast<wchar_t>(c);     
        }
    }
    out = std::wstring(tmp);
}

void convert_wstring_to_candle(const std::wstring& in, std::wstring& out) {
    std::wstring tmp;

    for (uint32_t i = 0; i < in.size(); i += 2) {
        std::stringstream ss;
        std::string s;
        
        if (i + 1 < in.size()) {
            ss << "0x" << std::setfill('0') << std::setw(2) << std::hex << (int)in.at(i+1);
            ss  << std::setfill('0') << std::setw(2) << std::hex << (int)in.at(i);
        } else {
            ss << "0x" << std::setfill('0') << std::setw(2) << std::hex << (int)in.at(i);
        }
        s = ss.str();

        int c = std::stoi(s, 0, 16);
        tmp += static_cast<wchar_t>(c);
    }
    out = std::wstring(tmp);
}

bool CANBridge_ProcessMask(const CANBridge_CANFilter& filter, uint32_t id) 
{
    bool result = (filter.messageMask & id) == (filter.messageMask & filter.messageId); 
    
    return result;
}

bool CANMessageCompare(CANMessage& a, CANMessage& b) 
{
    return a.GetTimestampUs() > b.GetTimestampUs();
}

int parse_serial_com_port(const std::string& in) {
    uint8_t substr_break;
#if defined(_WIN32)
    substr_break = 3; // COMxx
#elif defined(__linux__) 
    substr_break = 11; // /dev/ttyUSBxx
#elif defined(__APPLE__)
    substr_break = 9; // /dev/diskxx
#endif

    if(!in.empty() && (substr_break < in.length())) {
        std::string num = in.substr(substr_break, in.length());
        if (!num.empty()) {
            return std::stoi(num);
        }
    }

    return -1;
}

}
}
