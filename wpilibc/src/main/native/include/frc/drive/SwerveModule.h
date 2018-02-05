/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/ClosedLoopMotor.h"

namespace frc {

class SwerveModule {
 public
  SwerveModule(ClosedLoopMotor& driveMotor, ClosedLoopMotor& rotationMotor);

 private:
  ClosedLoopMotor& m_driveMotor;
  ClosedLoopMotor& m_rotationMotor;
};

}  // namespace frc
