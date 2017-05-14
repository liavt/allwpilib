/*----------------------------------------------------------------------------*/
/* Copyright (c) 2015-2017 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cassert>
#include <cmath>
#include <vector>

namespace frc {

/**
 * Create a linear FIR or IIR filter.
 *
 * @param input   The SourceNode object that is used to get values
 * @param ffGains The "feed forward" or FIR gains
 * @param fbGains The "feed back" or IIR gains
 */
template <typename SourceNode>
LinearFilter<SourceNode>::LinearFilter(SourceNode& input,
                                       std::initializer_list<double> ffGains,
                                       std::initializer_list<double> fbGains)
    : m_input(input),
      m_inputs(ffGains.size()),
      m_outputs(fbGains.size()),
      m_inputGains(ffGains),
      m_outputGains(fbGains) {}

/**
 * Create a linear FIR or IIR filter.
 *
 * @param input   The SourceNode object that is used to get values
 * @param ffGains The "feed forward" or FIR gains
 * @param fbGains The "feed back" or IIR gains
 */
template <typename SourceNode>
LinearFilter<SourceNode>::LinearFilter(SourceNode& input,
                                       std::initializer_list<double> ffGains,
                                       const std::vector<double>& fbGains)
    : m_input(input),
      m_inputs(ffGains.size()),
      m_outputs(fbGains.size()),
      m_inputGains(ffGains),
      m_outputGains(fbGains) {}

/**
 * Create a linear FIR or IIR filter.
 *
 * @param input   The SourceNode object that is used to get values
 * @param ffGains The "feed forward" or FIR gains
 * @param fbGains The "feed back" or IIR gains
 */
template <typename SourceNode>
LinearFilter<SourceNode>::LinearFilter(SourceNode& input,
                                       const std::vector<double>& ffGains,
                                       std::initializer_list<double> fbGains)
    : m_input(input),
      m_inputs(ffGains.size()),
      m_outputs(fbGains.size()),
      m_inputGains(ffGains),
      m_outputGains(fbGains) {}

/**
 * Create a linear FIR or IIR filter.
 *
 * @param input   The SourceNode object that is used to get values
 * @param ffGains The "feed forward" or FIR gains
 * @param fbGains The "feed back" or IIR gains
 */
template <typename SourceNode>
LinearFilter<SourceNode>::LinearFilter(SourceNode& input,
                                       const std::vector<double>& ffGains,
                                       const std::vector<double>& fbGains)
    : m_input(input),
      m_inputs(ffGains.size()),
      m_outputs(fbGains.size()),
      m_inputGains(ffGains),
      m_outputGains(fbGains) {}

/**
 * Creates a one-pole IIR low-pass filter of the form:<br>
 *   y[n] = (1 - gain) * x[n] + gain * y[n-1]<br>
 * where gain = e<sup>-dt / T</sup>, T is the time constant in seconds
 *
 * This filter is stable for time constants greater than zero.
 *
 * @param input        The SourceNode object that is used to get values
 * @param timeConstant The discrete-time time constant in seconds
 * @param period       The period in seconds between samples taken by the user
 */
template <typename SourceNode>
LinearFilter<SourceNode> LinearFilter<SourceNode>::SinglePoleIIR(
    SourceNode& input, double timeConstant, double period) {
  double gain = std::exp(-period / timeConstant);
  return LinearFilter(input, {1.0 - gain}, {-gain});
}

/**
 * Creates a first-order high-pass filter of the form:<br>
 *   y[n] = gain * x[n] + (-gain) * x[n-1] + gain * y[n-1]<br>
 * where gain = e<sup>-dt / T</sup>, T is the time constant in seconds
 *
 * This filter is stable for time constants greater than zero.
 *
 * @param input        The SourceNode object that is used to get values
 * @param timeConstant The discrete-time time constant in seconds
 * @param period       The period in seconds between samples taken by the user
 */
template <typename SourceNode>
LinearFilter<SourceNode> LinearFilter<SourceNode>::HighPass(SourceNode& input,
                                                            double timeConstant,
                                                            double period) {
  double gain = std::exp(-period / timeConstant);
  return LinearFilter(input, {gain, -gain}, {-gain});
}

/**
 * Creates a K-tap FIR moving average filter of the form:<br>
 *   y[n] = 1/k * (x[k] + x[k-1] + … + x[0])
 *
 * This filter is always stable.
 *
 * @param input  The SourceNode object that is used to get values
 * @param taps   The number of samples to average over. Higher = smoother but
 *               slower
 */
template <typename SourceNode>
LinearFilter<SourceNode> LinearFilter<SourceNode>::MovingAverage(
    SourceNode& input, int taps) {
  assert(taps > 0);

  std::vector<double> gains(taps, 1.0 / taps);
  return LinearFilter(input, gains, {});
}

/**
 * Calculates the next value of the filter.
 *
 * @return The filtered value at this step
 */
template <typename SourceNode>
double LinearFilter<SourceNode>::GetOutput() {
  double retVal = 0.0;

  // Rotate the inputs
  m_inputs.push_front(m_input.GetOutput());

  // Calculate the new value
  for (size_t i = 0; i < m_inputGains.size(); i++) {
    retVal += m_inputs[i] * m_inputGains[i];
  }
  for (size_t i = 0; i < m_outputGains.size(); i++) {
    retVal -= m_outputs[i] * m_outputGains[i];
  }

  // Rotate the outputs
  m_outputs.push_front(retVal);

  return retVal;
}

/**
 * Resets filter state.
 *
 * Warning: This operation is not thread-safe. Only call this when the Output
 * instance using a graph containing this node is disabled.
 */
template <typename SourceNode>
void LinearFilter<SourceNode>::Reset() {
  m_inputs.reset();
  m_outputs.reset();
}

}  // namespace frc
