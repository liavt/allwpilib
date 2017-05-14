/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cmath>
#include <utility>

namespace frc {

/**
 * Appends input and associated sign to array.
 *
 * @param input    The SourceNode object to add to the array for round robin.
 * @param positive If true, adds input; otherwise, subtracts input.
 * @param inputs   The other SourceNode objects and bools.
 */
template <typename SourceNode, typename... SourceNodes>
SumNode<SourceNode, SourceNodes...>::SumNode(SourceNode& input, bool positive,
                                             SourceNodes&&... inputs)
    : m_inputs{input, positive, inputs...} {}

namespace detail {

template <typename SourceNode, typename... Us>
double Sum(SourceNode&& input, bool positive, Us&&... us) {
  if (positive) {
    return input.GetOutput() + Sum(std::forward<Us>(us)...);
  } else {
    return -input.GetOutput() + Sum(std::forward<Us>(us)...);
  }
}

template <typename SourceNode>
double Sum(SourceNode&& input, bool positive) {
  if (positive) {
    return input.GetOutput();
  } else {
    return -input.GetOutput();
  }
}

template <typename Tuple, size_t... Is>
double SumImpl(const Tuple& t, std::index_sequence<Is...>) {
  return Sum(std::get<Is>(t)...);
}

}  // namespace detail

template <typename SourceNode, typename... SourceNodes>
double SumNode<SourceNode, SourceNodes...>::GetOutput() {
  double sum = detail::SumImpl(
      m_inputs, std::index_sequence_for<SourceNode, bool, SourceNodes...>());

  std::lock_guard<wpi::mutex> lock(m_mutex);

  m_lastResult = m_currentResult;
  m_currentResult = sum;

  if (m_continuous && std::abs(sum) > (m_maximumInput - m_minimumInput) / 2.0) {
    if (sum > 0.0) {
      return sum - (m_maximumInput - m_minimumInput);
    } else {
      return sum + (m_maximumInput - m_minimumInput);
    }
  } else {
    return sum;
  }
}

/**
 * Set the sum node to consider the input to be continuous.
 *
 * Rather then using the max and min in as constraints, it considers them to
 * be the same point and automatically calculates the shortest route to
 * the reference (e.g., gyroscope angle).
 *
 * @param continuous true turns on continuous; false turns off continuous
 */
template <typename SourceNode, typename... SourceNodes>
void SumNode<SourceNode, SourceNodes...>::SetContinuous(bool continuous) {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  m_continuous = continuous;
}

/**
 * Sets the maximum and minimum values expected from the input.
 *
 * @param minimumInput the minimum value expected from the input
 * @param maximumInput the maximum value expected from the input
 */
template <typename SourceNode, typename... SourceNodes>
void SumNode<SourceNode, SourceNodes...>::SetInputRange(double minimumInput,
                                                        double maximumInput) {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  m_minimumInput = minimumInput;
  m_maximumInput = maximumInput;
}

/**
 * Set the absolute error which is considered tolerable for use with
 * OnTarget.
 *
 * @param tolerance absolute error which is tolerable
 * @param deltaTolerance change in absolute error which is tolerable
 */
template <typename SourceNode, typename... SourceNodes>
void SumNode<SourceNode, SourceNodes...>::SetTolerance(double tolerance,
                                                       double deltaTolerance) {
  std::lock_guard<wpi::mutex> lock(m_mutex);

  m_tolerance = tolerance;
  m_deltaTolerance = deltaTolerance;
}

/**
 * Return true if the error and change in error is within the range determined
 * by SetTolerance().
 */
template <typename SourceNode, typename... SourceNodes>
bool SumNode<SourceNode, SourceNodes...>::InTolerance() const {
  std::lock_guard<wpi::mutex> lock(m_mutex);

  return std::abs(m_currentResult) < m_tolerance &&
         std::abs(m_currentResult - m_lastResult) < m_deltaTolerance;
}

}  // namespace frc
