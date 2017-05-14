/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

namespace frc {

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param input the node that is used to get values
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 */
template <typename SourceNode>
PIDNode<SourceNode>::PIDNode(double Kp, double Ki, double Kd, SourceNode& input,
                             double period)
    : m_input(input),
      m_P(Kp, *this),
      m_I(Ki, *this, period),
      m_D(Kd, *this, period),
      m_sum(m_P, true, m_I, true, m_D, true) {}

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param feedforward node to use for feedforward calculations
 * @param input the node that is used to get values
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 */
template <typename SourceNode>
PIDNode<SourceNode>::PIDNode(double Kp, double Ki, double Kd,
                             SourceNode& feedforward, SourceNode& input,
                             double period)
    : m_P(Kp, input),
      m_I(Ki, input, period),
      m_D(Kd, input, period),
      m_sum(m_P, true, m_I, true, m_D, true, feedforward, true) {}

template <typename SourceNode>
double PIDNode<SourceNode>::GetOutput() {
  double sum = m_sum.GetOutput();

  if (sum > m_maxU) {
    return m_maxU;
  } else if (sum < m_minU) {
    return m_minU;
  } else {
    return sum;
  }
}

/**
 * Set the PID Controller gain parameters. Set the proportional, integral, and
 * differential coefficients.
 *
 * @param p Proportional coefficient
 * @param i Integral coefficient
 * @param d Differential coefficient
 */
template <typename SourceNode>
void PIDNode<SourceNode>::SetPID(double p, double i, double d) {
  m_P.SetGain(p);
  m_I.SetGain(i);
  m_D.SetGain(d);
}

/**
 * Get the Proportional coefficient.
 *
 * @return proportional coefficient
 */
template <typename SourceNode>
double PIDNode<SourceNode>::GetP() const {
  return m_P.GetGain();
}

/**
 * Get the Integral coefficient.
 *
 * @return integral coefficient
 */
template <typename SourceNode>
double PIDNode<SourceNode>::GetI() const {
  return m_I.GetGain();
}

/**
 * Get the Differential coefficient.
 *
 * @return differential coefficient
 */
template <typename SourceNode>
double PIDNode<SourceNode>::GetD() const {
  return m_D.GetGain();
}

/**
 * Sets the minimum and maximum values to write.
 *
 * @param minU the minimum value to write to the output
 * @param maxU the maximum value to write to the output
 */
template <typename SourceNode>
void PIDNode<SourceNode>::SetOutputRange(double minU, double maxU) {
  m_minU = minU;
  m_maxU = maxU;
}

/**
 * Set maximum magnitude of input for which integration should occur. Values
 * above this will reset the current total.
 *
 * @param maxInputMagnitude max value of input for which integration should
 *                          occur
 */
template <typename SourceNode>
void PIDNode<SourceNode>::SetIZone(double maxInputMagnitude) {
  m_I.SetIZone(maxInputMagnitude);
}

/**
 * Clear the integral and derivative states.
 */
template <typename SourceNode>
void PIDNode<SourceNode>::Reset() {
  m_I.Reset();
  m_D.Reset();
}

}  // namespace frc
