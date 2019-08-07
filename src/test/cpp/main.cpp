#include <hal/HAL.h>

#include <iostream>
#include <chrono>
#include <thread>

#include "rev/CANBridge.h"
#include "rev/MockDS.h"

#include <rev/CANSparkMax.h>

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);

  frc::MockDS ds;
  ds.start();
  HAL_ObserveUserProgramStarting();
  ds.enable();

  std::cout << "Running Scan..." << std::endl;

  auto handle = CANBridge_Scan();

  std::cout << "Scan Complete" << std::endl;

  int numDevices = CANBridge_NumDevices(handle);

  std::cout << "Found " << numDevices << " Devices: " << std::endl;

  if (numDevices == 0) {
    return 0;
  }

  for (int i=0;i<numDevices;i++) {
    std::cout << "\t" << CANBridge_GetDeviceName(handle, i) << " - " << CANBridge_GetDeviceDescriptor(handle, i) << std::endl;
  }

  if (numDevices == 1) {
    std::cout << "Registering single device to HAL" << std::endl;
    CANBridge_RegisterDeviceToHAL(CANBridge_GetDeviceDescriptor(handle, 0), 0, 0);
  }

  CANBridge_FreeScan(handle);

  std::cout << "Starting Robot... Press 'a' to stop" << std::endl;

  rev::CANSparkMax m_motor(61, rev::CANSparkMax::MotorType::kBrushed);
  m_motor.Set(0.5);

  std::cin.get();

  return 0;
}
