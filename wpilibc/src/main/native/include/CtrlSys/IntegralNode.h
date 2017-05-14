/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <limits>

#include <support/mutex.h>

#include "Base.h"

namespace frc {

/**
 * Represents an integrator in a control system diagram.
 */
template <typename SourceNode>
class IntegralNode {
 public:
  IntegralNode(double K, SourceNode& input, double period = kDefaultPeriod);
  virtual ~IntegralNode() = default;

  double GetOutput();

  void SetGain(double K);
  double GetGain() const;

  void SetIZone(double maxInputMagnitude);

  void Reset();

 private:
  SourceNode& m_input;

  double m_gain;
  double m_period;

  double m_total = 0.0;
  double m_maxInputMagnitude = std::numeric_limits<double>::infinity();

  mutable wpi::mutex m_mutex;
};

}  // namespace frc

#include "CtrlSys/IntegralNode.inc"
