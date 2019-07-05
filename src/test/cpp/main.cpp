#include <hal/HAL.h>

#include <iostream>
#include <chrono>
#include <thread>

#include "USBSparkMax.h"

#include <rev/CANSparkMax.h>

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);

  std::cout << "Running Scan..." << std::endl;

  auto handle = USBSparkMax_Scan();

  std::cout << "Scan Complete" << std::endl;

  int numDevices = USBSparkMax_NumDevices(handle);

  std::cout << "Found " << numDevices << " Devices: " << std::endl;

  if (numDevices == 0) {
    return 0;
  }

  for (int i=0;i<numDevices;i++) {
    std::cout << "\t" << USBSparkMax_GetDeviceName(handle, i) << std::endl;
  }

  if (numDevices == 1) {
    std::cout << "Registering single device to HAL" << std::endl;
    USBSparkMax_RegisterDeviceToHAL(USBSparkMax_GetDeviceName(handle, 0), 0, 0);
  }

  USBSparkMax_FreeScan(handle);

  std::cout << "Starting Robot... Press 'a' to stop" << std::endl;

  rev::CANSparkMax m_motor(6, rev::CANSparkMax::MotorType::kBrushless);
  m_motor.Set(0.5);

  std::cin.get();

  return 0;
}
