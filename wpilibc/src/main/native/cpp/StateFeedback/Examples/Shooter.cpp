/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "StateFeedback/Examples/Shooter.h"

#include <algorithm>
#include <chrono>

#include <HAL/cpp/Log.h>

using namespace frc;

void Shooter::Reset() { m_wheel.Reset(); }

void Shooter::Iterate(const ShooterGoal* goal, const double& position,
                      std::chrono::steady_clock::time_point positionTime,
                      double* output) {
  if (goal) {
    // Update position/goal for our wheel.
    m_wheel.SetGoal(goal->angularVelocity);
  }

  m_wheel.SetPosition(position);

  auto dt = ShooterController::kLoopPeriod;
  if (m_lastTime != std::chrono::steady_clock::time_point::min()) {
    dt = positionTime - m_lastTime;
  }
  m_lastTime = positionTime;

  m_wheel.Update(output == nullptr, dt);

  if (m_lastReady && !m_wheel.Ready()) {
    m_min = m_wheel.DtVelocity();
  } else if (!m_wheel.Ready()) {
    m_min = std::min(m_min, m_wheel.DtVelocity());
  } else if (!m_lastReady && m_wheel.Ready()) {
    FILE_LOG(logINFO) << "Shot min was [" << m_min << "]\n";
  }

  if (output) {
    *output = m_wheel.Voltage();
  }
  m_lastReady = m_wheel.Ready();
}
