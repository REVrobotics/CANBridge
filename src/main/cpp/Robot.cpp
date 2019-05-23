/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>
#include <frc/TimedRobot.h>

#include <iostream>

using namespace rev;
using namespace std;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * Joystick analog values range from -1 to 1 and speed controller inputs as
 * range from -1 to 1 making it easy to work together.
 */
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    m_motor.SetPeriodicFramePeriod(CANSparkMax::PeriodicFrame::kStatus0, 1);
    m_motor.SetPeriodicFramePeriod(CANSparkMax::PeriodicFrame::kStatus1, 1);
    m_motor.SetPeriodicFramePeriod(CANSparkMax::PeriodicFrame::kStatus2, 1);
  }

  void TeleopInit() override {
   m_motor.RestoreFactoryDefaults();
   m_motor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
   m_firstRun = true;
   cout << "Starting!" << endl;
  }

  void TeleopPeriodic() override {
    if (m_firstRun) {
      m_firstRun = false;
    }
    m_motor.Set(0.5);
  }

 private:
  CANSparkMax m_motor{6, CANSparkMax::MotorType::kBrushless};
  bool m_firstRun;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
