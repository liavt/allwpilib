/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <support/mutex.h>

namespace frc {

/**
 * Returns the integral of the input node's output.
 */
template <typename SourceNode>
class DerivativeNode {
 public:
  DerivativeNode(double K, SourceNode& input, double period = kDefaultPeriod);
  virtual ~DerivativeNode() = default;

  double GetOutput();

  void SetGain(double K);
  double GetGain() const;

  void Reset();

 private:
  SourceNode& m_input;

  double m_gain;
  double m_period;

  double m_prevInput = 0.0;

  mutable wpi::mutex m_mutex;
};

}  // namespace frc

#include "CtrlSys/DerivativeNode.inc"
