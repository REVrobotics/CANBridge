/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "MockDS.h"

#include <stdint.h>

#include <hal/cpp/fpga_clock.h>
#include <wpi/Logger.h>
#include <wpi/SmallString.h>
#include <wpi/SmallVector.h>
#include <wpinet/UDPClient.h>
#include <wpi/raw_ostream.h>

#include <iostream>

static void LoggerFunc(unsigned int level, const char* file, unsigned int line,
                       const char* msg) {
  wpi::SmallString<128> buf;
  wpi::raw_svector_ostream oss(buf);
  if (level == 20) {
    oss << "DS: " << msg << '\n';
    wpi::errs() << oss.str();
    return;
  }

  oss << "DS: ";

  if (level >= 50)
    oss << "CRITICAL: ";
  else if (level >= 40)
    oss << "ERROR: ";
  else if (level >= 30)
    oss << "WARNING: ";
  else
    return;
  oss << msg << " (" << file << ":" << std::to_string(line) << ")\n";
  wpi::errs() << oss.str();
}

static void generateEnabledDsPacket(wpi::SmallVectorImpl<uint8_t>& data,
                                    uint16_t sendCount) {
  data.clear();
  data.push_back(sendCount >> 8);
  data.push_back(sendCount);
  data.push_back(0x01);  // general data tag
  data.push_back(0x04);  // teleop enabled
  data.push_back(0x10);  // normal data request
  data.push_back(0x00);  // red 1 station
}

using namespace frc;

void MockDS::start() {
  if (m_active) return;
  m_active = true;
  m_thread = std::thread([&]() {
    wpi::Logger logger(LoggerFunc);
    wpi::UDPClient client(logger);
    client.start();
    auto timeout_time = hal::fpga_clock::now();
    int initCount = 0;
    uint16_t sendCount = 0;
    wpi::SmallVector<uint8_t, 8> data;
    while (m_active) {
      // Keep 20ms intervals, and increase time to next interval
      auto current = hal::fpga_clock::now();
      while (timeout_time <= current) {
        timeout_time += std::chrono::milliseconds(20);
      }
      std::this_thread::sleep_until(timeout_time);
      generateEnabledDsPacket(data, sendCount++);
      // ~10 disabled packets are required to make the robot actually enable
      // 1 is definitely not enough.
      if (initCount < 10) {
        initCount++;
        data[3] = 0;
      }

      // if m_enable is false, set to disabled packet
      if(!m_enable) {
        data[3] = 0;
      }

      client.send(data, "127.0.0.1", 1110);
    }
    client.shutdown();
  });
}

void MockDS::stop() {
  m_active = false;
  if (m_thread.joinable()) m_thread.join();
}

void MockDS::enable() {
  m_enable = true;
}

void MockDS::disable() {
  m_enable = false;
}
