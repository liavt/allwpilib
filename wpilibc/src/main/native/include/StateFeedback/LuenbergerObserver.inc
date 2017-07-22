/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <utility>

#include "StateFeedback/StateFeedbackLoop.h"

namespace frc {

template <int States, int Inputs, int Outputs>
LuenbergerObserver<States, Inputs, Outputs>::LuenbergerObserver(
    LuenbergerObserver&& rhs)
    : m_Xhat(rhs.m_Xhat), m_index(rhs.m_index) {
  std::swap(m_coefficients, rhs.m_coefficients);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, Outputs>&
LuenbergerObserver<States, Inputs, Outputs>::L() const {
  return GetCoefficients().L;
}

template <int States, int Inputs, int Outputs>
double LuenbergerObserver<States, Inputs, Outputs>::L(int i, int j) const {
  return L()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, 1>&
LuenbergerObserver<States, Inputs, Outputs>::Xhat() const {
  return m_Xhat;
}

template <int States, int Inputs, int Outputs>
Eigen::Matrix<double, States, 1>&
LuenbergerObserver<States, Inputs, Outputs>::MutableXhat() {
  return m_Xhat;
}

template <int States, int Inputs, int Outputs>
void LuenbergerObserver<States, Inputs, Outputs>::Reset(
    StateFeedbackLoop<States, Inputs, Outputs, Plant<States, Inputs, Outputs>,
                      LuenbergerObserver>*
    /*loop*/) {
  m_Xhat.setZero();
}

template <int States, int Inputs, int Outputs>
void LuenbergerObserver<States, Inputs, Outputs>::Predict(
    StateFeedbackLoop<States, Inputs, Outputs, Plant<States, Inputs, Outputs>,
                      LuenbergerObserver>* loop,
    const Eigen::Matrix<double, Inputs, 1>& newU,
    std::chrono::nanoseconds /*dt*/) {
  m_Xhat = loop->GetPlant().UpdateX(Xhat(), newU);
}

template <int States, int Inputs, int Outputs>
void LuenbergerObserver<States, Inputs, Outputs>::Correct(
    const StateFeedbackLoop<States, Inputs, Outputs,
                            Plant<States, Inputs, Outputs>, LuenbergerObserver>&
        loop,
    const Eigen::Matrix<double, Inputs, 1>& U,
    const Eigen::Matrix<double, Outputs, 1>& Y) {
  /*
   * A^-1 is applied here to step the L matrix back an iteration to match the
   * prediction step. Note that this cancels the A in "A * K" for a Kalman
   * filter run as a Luenberger observer to give simply K, as expected for
   * separate steps.
   */
  m_Xhat += loop.GetPlant().Ainv() * L() *
            (Y - loop.GetPlant().C() * Xhat() - loop.GetPlant().D() * U);
}

template <int States, int Inputs, int Outputs>
void LuenbergerObserver<States, Inputs, Outputs>::AddCoefficients(
    const LuenbergerObserverCoeffs<States, Inputs, Outputs>& coefficients) {
  m_coefficients.emplace_back(coefficients);
}

template <int States, int Inputs, int Outputs>
const LuenbergerObserverCoeffs<States, Inputs, Outputs>&
LuenbergerObserver<States, Inputs, Outputs>::GetCoefficients(int index) const {
  return m_coefficients[index];
}

template <int States, int Inputs, int Outputs>
const LuenbergerObserverCoeffs<States, Inputs, Outputs>&
LuenbergerObserver<States, Inputs, Outputs>::GetCoefficients() const {
  return m_coefficients[m_index];
}

template <int States, int Inputs, int Outputs>
void LuenbergerObserver<States, Inputs, Outputs>::SetIndex(int index) {
  if (index < 0) {
    m_index = 0;
  } else if (index >= static_cast<int>(m_coefficients.size())) {
    m_index = static_cast<int>(m_coefficients.size()) - 1;
  } else {
    m_index = index;
  }
}

template <int States, int Inputs, int Outputs>
int LuenbergerObserver<States, Inputs, Outputs>::GetIndex() const {
  return m_index;
}

}  // namespace frc
