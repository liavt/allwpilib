/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "CtrlSys/Output.h"
#include "CtrlSys/PIDNode.h"
#include "CtrlSys/RefInput.h"
#include "CtrlSys/SumNode.h"
#include "PIDInterface.h"
#include "PIDOutput.h"
#include "SmartDashboard/SendableBase.h"

namespace frc {
namespace ctrl {

/**
 * Class implements a PID Control Loop.
 *
 * Creates a separate thread which reads the given PIDSource and takes
 * care of the integral calculations, as well as writing the given PIDOutput.
 *
 * This feedback controller runs in discrete time, so time deltas are not used
 * in the integral and derivative calculations. Therefore, the sample rate
 * affects the controller's behavior for a given set of PID constants.
 */
template <typename SourceNode>
class PIDController : public SendableBase, public PIDInterface {
 public:
  PIDController(double Kp, double Ki, double Kd, SourceNode& input,
                PIDOutput& output, double period = kDefaultPeriod);
  PIDController(double Kp, double Ki, double Kd, SourceNode& feedforward,
                SourceNode& input, PIDOutput& output,
                double period = kDefaultPeriod);
  virtual ~PIDController() = default;

  PIDController(const PIDController&) = delete;
  PIDController& operator=(const PIDController) = delete;

  void SetPID(double Kp, double Ki, double Kd) override;
  void SetPID(double Kp, double Ki, double Kd, double Kff) override;
  double GetP() const override;
  double GetI() const override;
  double GetD() const override;

  void SetContinuous(bool continuous = true);
  void SetInputRange(double minimumInput, double maximumInput);
  void SetOutputRange(double minimumOutput, double maximumOutput);
  void SetIZone(double maxErrorMagnitude);

  void SetSetpoint(double setpoint) override;
  double GetSetpoint() const override;

  void SetAbsoluteTolerance(double tolerance, double deltaTolerance);
  bool OnTarget() const;

  void Enable() override;
  void Disable() override;
  bool IsEnabled() const override;
  void SetEnabled(bool enable);

  void Reset() override;

  void InitSendable(SendableBuilder& builder) override;

 private:
  RefInput m_refInput{0.0};
  GainNode<RefInput> m_feedforward{0.0, m_refInput};
  SumNode<RefInput, bool, SourceNode, bool> m_sum;
  PIDNode<decltype(m_sum)> m_pid;
  Output<decltype(m_pid)> m_output;
  double m_tolerance = 0.05;
  bool m_enabled = false;
};

}  // namespace ctrl
}  // namespace frc

#include "CtrlSys/PIDController.inc"
