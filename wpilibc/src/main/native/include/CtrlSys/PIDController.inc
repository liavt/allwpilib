/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <string>

#include <HAL/HAL.h>

#include "SmartDashboard/SendableBuilder.h"

namespace frc {
namespace ctrl {

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param input the node that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 */
template <typename SourceNode>
PIDController<SourceNode>::PIDController(double Kp, double Ki, double Kd,
                                         SourceNode& input, PIDOutput& output,
                                         double period)
    : m_sum(m_refInput, true, input, false),
      m_pid(Kp, Ki, Kd, m_sum, period),
      m_output(m_pid, output, period) {
  static int instances = 0;
  instances++;
  HAL_Report(HALUsageReporting::kResourceType_PIDController, instances);
}

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param feedforward node to use for feedforward calculations
 * @param input the node that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 */
template <typename SourceNode>
PIDController<SourceNode>::PIDController(double Kp, double Ki, double Kd,
                                         SourceNode& feedforward,
                                         SourceNode& input, PIDOutput& output,
                                         double period)
    : m_sum(m_refInput, true, input, false),
      m_pid(Kp, Ki, Kd, feedforward, m_sum, period),
      m_output(m_pid, output, period) {
  static int instances = 0;
  instances++;
  HAL_Report(HALUsageReporting::kResourceType_PIDController, instances);
}

/**
 * Set the PID Controller gain parameters.
 *
 * Set the proportional, integral, and differential coefficients.
 *
 * @param Kp Proportional coefficient
 * @param Ki Integral coefficient
 * @param Kd Differential coefficient
 */
template <typename SourceNode>
void PIDController<SourceNode>::SetPID(double Kp, double Ki, double Kd) {
  m_pid.SetPID(Kp, Ki, Kd);
}

/**
 * Set the PID Controller gain parameters.
 *
 * Set the proportional, integral, and differential coefficients.
 *
 * @param Kp Proportional coefficient
 * @param Ki Integral coefficient
 * @param Kd Differential coefficient
 * @param Kff Feedforward coefficient
 */
template <typename SourceNode>
void PIDController<SourceNode>::SetPID(double Kp, double Ki, double Kd,
                                       double Kff) {
  m_pid.SetPID(Kp, Ki, Kd);
  m_feedforward.SetGain(Kff);
}

/**
 * Get the Proportional coefficient.
 *
 * @return proportional coefficient
 */
template <typename SourceNode>
double PIDController<SourceNode>::GetP() const {
  return m_pid.GetP();
}

/**
 * Get the Integral coefficient.
 *
 * @return integral coefficient
 */
template <typename SourceNode>
double PIDController<SourceNode>::GetI() const {
  return m_pid.GetI();
}

/**
 * Get the Differential coefficient.
 *
 * @return differential coefficient
 */
template <typename SourceNode>
double PIDController<SourceNode>::GetD() const {
  return m_pid.GetD();
}

/**
 * Set the PID controller to consider the input to be continuous.
 *
 * Rather then using the max and min in as constraints, it considers them to
 * be the same point and automatically calculates the shortest route to
 * the reference (e.g., gyroscope angle).
 *
 * @param continuous true turns on continuous; false turns off continuous
 */
template <typename SourceNode>
void PIDController<SourceNode>::SetContinuous(bool continuous) {
  m_sum.SetContinuous(continuous);
}

/**
 * Sets the maximum and minimum values expected from the input.
 *
 * @param minimumInput the minimum value expected from the input
 * @param maximumInput the maximum value expected from the input
 */
template <typename SourceNode>
void PIDController<SourceNode>::SetInputRange(double minimumInput,
                                              double maximumInput) {
  m_sum.SetInputRange(minimumInput, maximumInput);
}

/**
 * Sets the minimum and maximum values to write.
 *
 * @param minimumOutput the minimum value to write to the output
 * @param maximumOutput the maximum value to write to the output
 */
template <typename SourceNode>
void PIDController<SourceNode>::SetOutputRange(double minimumOutput,
                                               double maximumOutput) {
  m_output.SetRange(minimumOutput, maximumOutput);
}

/**
 * Set maximum magnitude of error for which integration should occur. Values
 * above this will reset the current total.
 *
 * @param maxErrorMagnitude max value of error for which integration should
 *                          occur
 */
template <typename SourceNode>
void PIDController<SourceNode>::SetIZone(double maxErrorMagnitude) {
  m_pid.SetIZone(maxErrorMagnitude);
}

/**
 * Set the setpoint for the PIDController.
 *
 * @param setpoint the desired setpoint
 */
template <typename SourceNode>
void PIDController<SourceNode>::SetSetpoint(double setpoint) {
  m_refInput.Set(setpoint);
}

/**
 * Returns the current setpoint of the PIDController.
 *
 * @return the current setpoint
 */
template <typename SourceNode>
double PIDController<SourceNode>::GetSetpoint() const {
  return m_refInput.GetOutput();
}

/**
 * Set the absolute error which is considered tolerable for use with
 * OnTarget.
 *
 * @param tolerance absolute error which is tolerable
 * @param deltaTolerance change in absolute error which is tolerable
 */
template <typename SourceNode>
void PIDController<SourceNode>::SetAbsoluteTolerance(double tolerance,
                                                     double deltaTolerance) {
  m_sum.SetTolerance(tolerance, deltaTolerance);
}

/**
 * Return true if the error and change in error is within the range determined
 * by SetTolerance.
 */
template <typename SourceNode>
bool PIDController<SourceNode>::OnTarget() const {
  return m_sum.InTolerance();
}

/**
 * Begin running the PIDController.
 */
template <typename SourceNode>
void PIDController<SourceNode>::Enable() {
  m_enabled = true;
  m_output.Enable();
}

/**
 * Stop running the PIDController, this sets the output to zero before stopping.
 */
template <typename SourceNode>
void PIDController<SourceNode>::Disable() {
  m_output.Disable();
  m_enabled = false;
}

template <typename SourceNode>
bool PIDController<SourceNode>::IsEnabled() const {
  return m_enabled;
}

template <typename SourceNode>
void PIDController<SourceNode>::SetEnabled(bool enable) {
  if (enable) {
    Enable();
  } else {
    Disable();
  }
}

template <typename SourceNode>
void PIDController<SourceNode>::Reset() {
  Disable();

  m_pid.Reset();
}

template <typename SourceNode>
void PIDController<SourceNode>::InitSendable(SendableBuilder& builder) {
  builder.SetSmartDashboardType("PIDController");
  builder.SetSafeState([=]() { Reset(); });
  builder.AddDoubleProperty("p", [=]() { return GetP(); },
                            [=](double value) { m_pid.SetP(value); });
  builder.AddDoubleProperty("i", [=]() { return GetI(); },
                            [=](double value) { m_pid.SetI(value); });
  builder.AddDoubleProperty("d", [=]() { return GetD(); },
                            [=](double value) { m_pid.SetD(value); });
  builder.AddDoubleProperty("setpoint", [=]() { return GetSetpoint(); },
                            [=](double value) { SetSetpoint(value); });
  builder.AddBooleanProperty("enabled", [=]() { return IsEnabled(); },
                             [=](bool value) { SetEnabled(value); });
}

}  // namespace ctrl
}  // namespace frc
