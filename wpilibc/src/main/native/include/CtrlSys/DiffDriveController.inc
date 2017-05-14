/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

namespace frc {

/**
 * Constructs DiffDriveController.
 *
 * @param positionRef position reference input
 * @param angleRef angle reference input
 * @param leftEncoder left encoder
 * @param rightEncoder right encoder
 * @param angleSensor angle sensor (e.g, gyroscope)
 * @param clockwise true if clockwise rotation increases angle measurement
 * @param leftMotor left motor output
 * @param rightMotor right motor output
 * @param period the loop time for doing calculations.
 */
template <typename SourceNode>
DiffDriveController<SourceNode>::DiffDriveController(
    SourceNode& positionRef, SourceNode& angleRef, SourceNode& leftEncoder,
    SourceNode& rightEncoder, SourceNode& angleSensor, bool clockwise,
    PIDOutput& leftMotor, PIDOutput& rightMotor, double period)
    : m_positionRef(positionRef),
      m_angleRef(angleRef),
      m_leftEncoder(leftEncoder),
      m_rightEncoder(rightEncoder),
      m_angleSensor(angleSensor),
      m_clockwise(clockwise),
      m_leftMotor(leftMotor),
      m_rightMotor(rightMotor),
      m_leftMotorInput(m_positionPID, true, m_anglePID, m_clockwise),
      m_leftOutput(m_leftMotorInput, m_leftMotor),
      m_rightMotorInput(m_positionPID, true, m_anglePID, !m_clockwise),
      m_rightOutput(m_rightMotorInput, m_rightMotor),
      m_outputs(m_leftOutput, m_rightOutput),
      m_period(period) {}

template <typename SourceNode>
void DiffDriveController<SourceNode>::Enable() {
  m_outputs.Enable(m_period);
}

template <typename SourceNode>
void DiffDriveController<SourceNode>::Disable() {
  m_outputs.Disable();
}

template <typename SourceNode>
auto& DiffDriveController<SourceNode>::GetPositionPID() {
  return m_positionPID;
}

template <typename SourceNode>
auto& DiffDriveController<SourceNode>::GetAnglePID() {
  return m_anglePID;
}

template <typename SourceNode>
void DiffDriveController<SourceNode>::SetPositionTolerance(
    double tolerance, double deltaTolerance) {
  m_positionError.SetTolerance(tolerance, deltaTolerance);
}

template <typename SourceNode>
void DiffDriveController<SourceNode>::SetAngleTolerance(double tolerance,
                                                        double deltaTolerance) {
  m_angleError.SetTolerance(tolerance, deltaTolerance);
}

template <typename SourceNode>
bool DiffDriveController<SourceNode>::AtPosition() const {
  return m_positionError.InTolerance();
}

template <typename SourceNode>
bool DiffDriveController<SourceNode>::AtAngle() const {
  return m_angleError.InTolerance();
}

}  // namespace frc
