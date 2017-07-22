/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <chrono>
#include <vector>

#include <Eigen/Core>

#include "StateFeedback/LuenbergerObserverCoeffs.h"
#include "StateFeedback/Plant.h"

namespace frc {

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
class StateFeedbackLoop;

/**
 * Luenberger observers combine predictions from a model and measurements to
 * give an estimate of the true system state.
 *
 * Luenberger observers use an L gain matrix to determine whether to trust the
 * model or measurements more. Kalman filter theory uses statistics to compute
 * an optimal L gain (alternatively called the Kalman gain, K) which minimizes
 * the sum of squares error in the state estimate.
 *
 * Luenberger observers run the prediction and correction steps simultaneously
 * while Kalman filters run them sequentially. To implement a discrete-time
 * Kalman filter as a Luenberger observer, use the following mapping:
 * <pre>C = H, L = A * K</pre>
 * (H is the measurement matrix).
 */
template <int States, int Inputs, int Outputs>
class LuenbergerObserver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LuenbergerObserver() = default;
  LuenbergerObserver(LuenbergerObserver&& rhs);

  const Eigen::Matrix<double, States, Outputs>& L() const;
  double L(int i, int j) const;

  const Eigen::Matrix<double, States, 1>& Xhat() const;
  Eigen::Matrix<double, States, 1>& MutableXhat();

  void Reset(
      StateFeedbackLoop<States, Inputs, Outputs, Plant<States, Inputs, Outputs>,
                        LuenbergerObserver>* /*loop*/);

  void Predict(
      StateFeedbackLoop<States, Inputs, Outputs, Plant<States, Inputs, Outputs>,
                        LuenbergerObserver>* loop,
      const Eigen::Matrix<double, Inputs, 1>& newU,
      std::chrono::nanoseconds /*dt*/);

  void Correct(const StateFeedbackLoop<States, Inputs, Outputs,
                                       Plant<States, Inputs, Outputs>,
                                       LuenbergerObserver>& loop,
               const Eigen::Matrix<double, Inputs, 1>& U,
               const Eigen::Matrix<double, Outputs, 1>& Y);

  void AddCoefficients(
      const LuenbergerObserverCoeffs<States, Inputs, Outputs>& coefficients);
  const LuenbergerObserverCoeffs<States, Inputs, Outputs>& GetCoefficients(
      int index) const;
  const LuenbergerObserverCoeffs<States, Inputs, Outputs>& GetCoefficients()
      const;

  /**
   * Sets the current observer to be index, clamped to be within range. This can
   * be used for gain scheduling.
   */
  void SetIndex(int index);

  int GetIndex() const;

 private:
  // Internal state estimate.
  Eigen::Matrix<double, States, 1> m_Xhat;

  std::vector<LuenbergerObserverCoeffs<States, Inputs, Outputs>> m_coefficients;
  int m_index = 0;
};

}  // namespace frc

#include "StateFeedback/LuenbergerObserver.inc"
