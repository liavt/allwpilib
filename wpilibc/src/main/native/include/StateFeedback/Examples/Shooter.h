/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <chrono>

#include <Eigen/Core>

#include "StateFeedback/Examples/ShooterController.h"
#include "StateFeedback/Examples/ShooterGoal.h"
#include "StateFeedback/StateFeedbackLoop.h"

namespace frc {

class Shooter {
 public:
  Shooter() = default;

  Shooter(const Shooter&) = delete;
  Shooter& operator=(const Shooter&) = delete;

  /**
   * Iterates the shooter control loop one cycle.
   *
   * @param position must
   * @param goal can be nullptr if no goal exists.
   * @param output should be nullptr if disabled.
   */
  void Iterate(const ShooterGoal* goal, const double& position,
               std::chrono::steady_clock::time_point positionTime,
               double* output);

  // Sets the shooter up to reset the kalman filter next time Iterate is called.
  void Reset();

 private:
  ShooterController m_wheel;

  bool m_lastReady = false;
  double m_min = 0.0;
  std::chrono::steady_clock::time_point m_lastTime =
      std::chrono::steady_clock::time_point::min();
};

}  // namespace frc
