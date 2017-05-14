/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <algorithm>
#include <cmath>

namespace frc {

template <class T>
constexpr const T& clamp(const T& value, const T& low, const T& high) {
  return std::max(low, std::min(value, high));
}

/**
 * Construct an integrator.
 *
 * @param K a gain to apply to the integrator output
 * @param input the input node
 * @param period the loop time for doing calculations.
 */
template <typename SourceNode>
IntegralNode<SourceNode>::IntegralNode(double K, SourceNode& input,
                                       double period)
    : m_input(input) {
  m_gain = K;
  m_period = period;
}

template <typename SourceNode>
double IntegralNode<SourceNode>::GetOutput() {
  double input = m_input.GetOutput();

  std::lock_guard<wpi::mutex> lock(m_mutex);

  if (std::abs(input) > m_maxInputMagnitude) {
    m_total = 0.0;
  } else {
    m_total = clamp(m_total + input * m_period, -1.0 / m_gain, 1.0 / m_gain);
  }

  return m_gain * m_total;
}

/**
 * Set gain applied to node output.
 *
 * @param K a gain to apply
 */
template <typename SourceNode>
void IntegralNode<SourceNode>::SetGain(double K) {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  m_gain = K;
}

/**
 * Return gain applied to node output.
 */
template <typename SourceNode>
double IntegralNode<SourceNode>::GetGain() const {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  return m_gain;
}

/**
 * Set maximum magnitude of input for which integration should occur. Values
 * above this will reset the current total.
 *
 * @param maxInputMagnitude max value of input for which integration should
 *                          occur
 */
template <typename SourceNode>
void IntegralNode<SourceNode>::SetIZone(double maxInputMagnitude) {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  m_maxInputMagnitude = maxInputMagnitude;
}

/**
 * Clears integral state.
 */
template <typename SourceNode>
void IntegralNode<SourceNode>::Reset() {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  m_total = 0.0;
}

}  // namespace frc
