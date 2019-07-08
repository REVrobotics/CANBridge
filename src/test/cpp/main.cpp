#include <hal/HAL.h>

#include <iostream>
#include <chrono>
#include <thread>

#include "rev/RevUSB.h"
#include "rev/MockDS.h"

#include <rev/CANSparkMax.h>

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);

  frc::MockDS ds;
  ds.start();
  HAL_ObserveUserProgramStarting();
  ds.enable();

  std::cout << "Running Scan..." << std::endl;

  auto handle = RevUSB_Scan();

  std::cout << "Scan Complete" << std::endl;

  int numDevices = RevUSB_NumDevices(handle);

  std::cout << "Found " << numDevices << " Devices: " << std::endl;

  if (numDevices == 0) {
    return 0;
  }

  for (int i=0;i<numDevices;i++) {
    std::cout << "\t" << RevUSB_GetDeviceName(handle, i) << std::endl;
  }

  if (numDevices == 1) {
    std::cout << "Registering single device to HAL" << std::endl;
    RevUSB_RegisterDeviceToHAL(RevUSB_GetDeviceName(handle, 0), 0, 0);
  }

  RevUSB_FreeScan(handle);

  std::cout << "Starting Robot... Press 'a' to stop" << std::endl;

  rev::CANSparkMax m_motor(61, rev::CANSparkMax::MotorType::kBrushed);
  m_motor.Set(0.5);

  std::cin.get();

  return 0;
}
