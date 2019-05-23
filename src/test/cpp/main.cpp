#include <hal/HAL.h>

#include <iostream>

#include "USBSparkMax.h"

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);

  std::cout << "Running Scan..." << std::endl;

  auto handle = USBSparkMax_Scan();

  std::cout << "Scan Complete" << std::endl;

  int numDevices = USBSparkMax_NumDevices(handle);

  std::cout << "Found " << numDevices << " Devices: " << std::endl;

  for (int i=0;i<numDevices;i++) {
    std::cout << "\t" << USBSparkMax_GetDeviceName(handle, i) << std::endl;
  }

  USBSparkMax_Close(handle);
  //USBSparkMax_RegisterHAL();

  return 0;
}
