/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

namespace frc {

/**
 * Returns the input node's output multiplied by a constant gain.
 *
 * @param K the gain on the input
 * @param input the input node
 */
template <typename SourceNode>
GainNode<SourceNode>::GainNode(double K, SourceNode& input) : m_input(input) {
  m_gain = K;
}

template <typename SourceNode>
double GainNode<SourceNode>::GetOutput() {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  return m_gain * m_input.GetOutput();
}

/**
 * Set gain applied to node output.
 *
 * @param K a gain to apply
 */
template <typename SourceNode>
void GainNode<SourceNode>::SetGain(double K) {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  m_gain = K;
}

/**
 * Return gain applied to node output.
 */
template <typename SourceNode>
double GainNode<SourceNode>::GetGain() const {
  std::lock_guard<wpi::mutex> lock(m_mutex);
  return m_gain;
}

}  // namespace frc
