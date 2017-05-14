/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Base.h"
#include "Controller.h"
#include "CtrlSys/FuncNode.h"
#include "CtrlSys/Output.h"
#include "CtrlSys/OutputGroup.h"
#include "CtrlSys/PIDNode.h"
#include "CtrlSys/SumNode.h"
#include "PIDOutput.h"

namespace frc {

/**
 * A feedback controller for a differential-drive robot. A differential-drive
 * robot has left and right wheels separated by an arbitrary width.
 *
 * A forward distance controller and angle controller are run in parallel and
 * their outputs are composed to drive each wheel. Since the forward controller
 * uses the average distance of the two sides while the angle controller uses
 * the difference between them, the controllers act independently on the drive
 * base and can thus be tuned separately.
 *
 * If you don't have a gyroscope for an angle sensor, the following equation can
 * be used in a FuncNode to estimate it.
 *
 * angle = (right - left) / width * 180 / pi
 *
 * where "right" is the right encoder reading, "left" is the left encoder
 * reading, "width" is the width of the robot in the same units as the
 * encoders, and "angle" is the angle of the robot in degrees. We recommend
 * passing this angle estimation through a low-pass filter (see LinearFilter).
 *
 * Set the position and angle PID constants via GetPositionPID()->SetPID() and
 * GetAnglePID()->SetPID() before enabling this controller.
 */
template <typename SourceNode>
class DiffDriveController : public Controller {
 public:
  DiffDriveController(SourceNode& positionRef, SourceNode& angleRef,
                      SourceNode& leftEncoder, SourceNode& rightEncoder,
                      SourceNode& angleSensor, bool clockwise,
                      PIDOutput& leftMotor, PIDOutput& rightMotor,
                      double period = kDefaultPeriod);
  virtual ~DiffDriveController() = default;

  void Enable() override;
  void Disable() override;

  auto& GetPositionPID();
  auto& GetAnglePID();

  void SetPositionTolerance(double tolerance, double deltaTolerance);
  void SetAngleTolerance(double tolerance, double deltaTolerance);

  bool AtPosition() const;
  bool AtAngle() const;

 private:
  // Control system references
  SourceNode& m_positionRef;
  SourceNode& m_angleRef;

  // Encoders
  SourceNode& m_leftEncoder;
  SourceNode& m_rightEncoder;

  // Angle sensor (e.g., gyroscope)
  SourceNode& m_angleSensor;
  bool m_clockwise;

  // Motors
  PIDOutput& m_leftMotor;
  PIDOutput& m_rightMotor;

  // Position PID
  FuncNode m_positionCalc{[&] {
    return (m_leftEncoder.GetOutput() + m_rightEncoder.GetOutput()) / 2.0;
  }};
  SumNode<SourceNode, bool, FuncNode, bool> m_positionError{
      m_positionRef, true, m_positionCalc, false};
  PIDNode<decltype(m_positionError)> m_positionPID{0.0, 0.0, 0.0,
                                                   m_positionError};

  // Angle PID
  SumNode<SourceNode, bool, SourceNode, bool> m_angleError{
      m_angleRef, true, m_angleSensor, false};
  PIDNode<decltype(m_angleError)> m_anglePID{0.0, 0.0, 0.0, m_angleError};

  // Combine outputs for left motor
  SumNode<decltype(m_positionPID), bool, decltype(m_anglePID), bool>
      m_leftMotorInput;
  Output<decltype(m_leftMotorInput)> m_leftOutput;

  // Combine outputs for right motor
  SumNode<decltype(m_positionPID), bool, decltype(m_anglePID), bool>
      m_rightMotorInput;
  Output<decltype(m_rightMotorInput)> m_rightOutput;

  OutputGroup<decltype(m_leftOutput), decltype(m_rightOutput)> m_outputs;
  double m_period;
};

}  // namespace frc

#include "CtrlSys/DiffDriveController.inc"
