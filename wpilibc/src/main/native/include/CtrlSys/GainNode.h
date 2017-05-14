/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <support/mutex.h>

namespace frc {

template <typename SourceNode>
class GainNode {
 public:
  GainNode(double K, SourceNode& input);
  virtual ~GainNode() = default;

  double GetOutput();

  void SetGain(double K);
  double GetGain() const;

 private:
  SourceNode& m_input;

  double m_gain;

  mutable wpi::mutex m_mutex;
};

}  // namespace frc

#include "CtrlSys/GainNode.inc"
