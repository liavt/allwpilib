/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

struct ShooterGoal {
  // Angular velocity goal in radians/second. Positive is shooting out of the
  // robot.
  double angularVelocity = 0.0;
};
