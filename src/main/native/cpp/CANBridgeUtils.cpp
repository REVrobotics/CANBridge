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
#include <locale>
#include <vector>
#if defined(_WIN32)
#include <windows.h>
#endif


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

// Convert a wide Unicode string to ANSI string
#if defined(_WIN32)
std::string unicode_decode(const std::wstring &wstr)
{
    int size_needed = WideCharToMultiByte(CP_ACP, 0, &wstr[0], (int)wstr.size(), NULL, 0, NULL, NULL);
	std::string strTo( size_needed, 0 );
	WideCharToMultiByte(CP_ACP, 0, &wstr[0], (int)wstr.size(), &strTo[0], size_needed, NULL, NULL);
	return strTo;
    
}

// Convert a ANSI string to wide Unicode string
std::wstring unicode_encode(const std::string &str)
{
	int size_needed = MultiByteToWideChar(CP_ACP, 0, &str[0], (int)str.size(), NULL, 0);
	std::wstring wstrTo(size_needed, 0);
	MultiByteToWideChar(CP_ACP, 0, &str[0], (int)str.size(), &wstrTo[0], size_needed);
	return wstrTo;
}
#endif



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
            int returnval = -1;
            try
            {
                returnval = std::stoi(num);
            }
            catch(const std::exception& e)
            {
                std::cerr << "parse_serial_com_port: Error running stoi: " << e.what() << '\n';
            }
            
            return returnval;
        }
    }

    return -1;
}

}
}
