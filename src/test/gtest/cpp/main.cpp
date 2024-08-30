/*
 * Copyright (c) 2019 - 2020 REV Robotics
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

#include <hal/HAL.h>

#include "MockDS.h"

#include "gtest/gtest.h"

#include <rev/Drivers/SocketCAN/SocketCANDriver.h>


int main(int argc, char** argv) {

  rev::usb::SocketCANDriver driver;

  auto output = driver.GetDevices();

  if (output.size() == 0) {
    std::cout << "No devices found" << std::endl;
    return 1;
  }

  for (auto itr = output.begin(); itr != output.end(); itr++) {
    std::cout << itr->descriptor << std::endl;
  }

  auto device = driver.CreateDeviceFromDescriptor(output[0].descriptor.c_str());

  std::cout << "Selected device: " << device->GetName() << std::endl;

  return 0;

  #if 0
  HAL_Initialize(500, 0);
  frc::MockDS ds;
  ds.start();
  HAL_ObserveUserProgramStarting();
  ds.enable();

  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
  #endif
}
