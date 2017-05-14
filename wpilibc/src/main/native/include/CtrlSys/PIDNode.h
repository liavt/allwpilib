/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Base.h"
#include "DerivativeNode.h"
#include "GainNode.h"
#include "IntegralNode.h"
#include "SumNode.h"

namespace frc {

/**
 * A PID controller implementation based on the control system diagram
 * framework. This is more general than the PIDController as its output can be
 * chained with other nodes that are not actuators.
 *
 * For a simple closed-loop PID controller, see PIDController.
 */
template <typename SourceNode>
class PIDNode {
 public:
  PIDNode(double Kp, double Ki, double Kd, SourceNode& input,
          double period = kDefaultPeriod);
  PIDNode(double Kp, double Ki, double Kd, SourceNode& feedforward,
          SourceNode& input, double period = kDefaultPeriod);
  virtual ~PIDNode() = default;

  double GetOutput();

  void SetPID(double p, double i, double d);
  double GetP() const;
  double GetI() const;
  double GetD() const;

  void SetOutputRange(double minU, double maxU);
  void SetIZone(double maxInputMagnitude);

  void Reset();

 private:
  GainNode<SourceNode> m_P;
  IntegralNode<SourceNode> m_I;
  DerivativeNode<SourceNode> m_D;
  SumNode<decltype(m_P), bool, decltype(m_I), bool, decltype(m_D), bool,
          SourceNode, bool>
      m_sum;

  double m_minU = -1.0;
  double m_maxU = 1.0;
};

}  // namespace frc
