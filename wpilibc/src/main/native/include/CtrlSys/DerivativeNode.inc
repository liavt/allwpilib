/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

namespace frc {

/**
 * Construct a differentiator.
 *
 * @param K a gain to apply to the differentiator output
 * @param input the input node
 * @param period the loop time for doing calculations.
 */
template <typename SourceNode>
DerivativeNode<SourceNode>::DerivativeNode(double K, SourceNode& input,
                                           double period)
    : m_input(input) {
  m_gain = K;
  m_period = period;
}

template <typename SourceNode>
double DerivativeNode<SourceNode>::GetOutput() {
  double input = m_input.GetOutput();

  std::lock_guard<wpi::mutex> lock(m_mutex);

  double output = m_gain * (input - m_prevInput) / m_period;

  m_prevInput = input;

  return output;
}

/**
 * Set gain applied to node output.
 *
 * @param K a gain to apply
 */
template <typename SourceNode>
void DerivativeNode<SourceNode>::SetGain(double K) {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  m_gain = K;
}

/**
 * Return gain applied to node output.
 */
template <typename SourceNode>
double DerivativeNode<SourceNode>::GetGain() const {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  return m_gain;
}

/**
 * Clears derivative state.
 */
template <typename SourceNode>
void DerivativeNode<SourceNode>::Reset() {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  m_prevInput = 0.0;
}

}  // namespace frc
