/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/drive/SwerveModule.h"

using namespace frc;

SwerveModule::SwerveModule(ClosedLoopMotor& driveMotor,
                           ClosedLoopMotor& rotationMotor)
    : m_driveMotor(driveMotor), m_rotationMotor(rotationMotor) {}
