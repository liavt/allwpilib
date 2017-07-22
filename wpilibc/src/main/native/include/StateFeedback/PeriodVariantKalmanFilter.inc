/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <utility>

#include <Eigen/MatrixFunctions>

namespace frc {

template <int States, int Inputs, int Outputs>
PeriodVariantKalmanFilter<States, Inputs, Outputs>::PeriodVariantKalmanFilter(
    const std::chrono::nanoseconds nominalLoopPeriod)
    : m_nominalLoopPeriod(nominalLoopPeriod) {}

template <int States, int Inputs, int Outputs>
PeriodVariantKalmanFilter<States, Inputs, Outputs>::PeriodVariantKalmanFilter(
    PeriodVariantKalmanFilter&& rhs)
    : m_nominalLoopPeriod(rhs.m_nominalLoopPeriod),
      m_Xhat(rhs.m_Xhat),
      m_index(rhs.m_index) {
  std::swap(m_coefficients, rhs.m_coefficients);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, States>&
PeriodVariantKalmanFilter<States, Inputs, Outputs>::Q() const {
  return m_Q;
}

template <int States, int Inputs, int Outputs>
double PeriodVariantKalmanFilter<States, Inputs, Outputs>::Q(int i,
                                                             int j) const {
  return Q()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Outputs, Outputs>&
PeriodVariantKalmanFilter<States, Inputs, Outputs>::R() const {
  return m_R;
}

template <int States, int Inputs, int Outputs>
double PeriodVariantKalmanFilter<States, Inputs, Outputs>::R(int i,
                                                             int j) const {
  return R()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, States>&
PeriodVariantKalmanFilter<States, Inputs, Outputs>::P() const {
  return m_P;
}

template <int States, int Inputs, int Outputs>
double PeriodVariantKalmanFilter<States, Inputs, Outputs>::P(int i,
                                                             int j) const {
  return P()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, 1>&
PeriodVariantKalmanFilter<States, Inputs, Outputs>::Xhat() const {
  return m_Xhat;
}

template <int States, int Inputs, int Outputs>
Eigen::Matrix<double, States, 1>&
PeriodVariantKalmanFilter<States, Inputs, Outputs>::MutableXhat() {
  return m_Xhat;
}

template <int States, int Inputs, int Outputs>
void PeriodVariantKalmanFilter<States, Inputs, Outputs>::Reset(
    StateFeedbackLoop<States, Inputs, Outputs,
                      PeriodVariantPlant<States, Inputs, Outputs>,
                      PeriodVariantKalmanFilter>* loop) {
  m_Xhat.setZero();
  m_P = GetCoefficients().PsteadyState;
  UpdateQR(loop, m_nominalLoopPeriod);
}

template <int States, int Inputs, int Outputs>
void PeriodVariantKalmanFilter<States, Inputs, Outputs>::Predict(
    StateFeedbackLoop<States, Inputs, Outputs,
                      PeriodVariantPlant<States, Inputs, Outputs>,
                      PeriodVariantKalmanFilter>* loop,
    const Eigen::Matrix<double, Inputs, 1>& newU, std::chrono::nanoseconds dt) {
  // Trigger the predict step. This will update A() and B() in the plant.
  m_Xhat = loop->MutablePlant()->UpdateX(Xhat(), newU, dt);

  UpdateQR(loop, dt);
  m_P = loop->GetPlant().A() * P() * loop->GetPlant().A().transpose() + Q();
}

template <int States, int Inputs, int Outputs>
void PeriodVariantKalmanFilter<States, Inputs, Outputs>::Correct(
    const StateFeedbackLoop<States, Inputs, Outputs,
                            PeriodVariantPlant<States, Inputs, Outputs>,
                            PeriodVariantKalmanFilter>& loop,
    const Eigen::Matrix<double, Inputs, 1>& U,
    const Eigen::Matrix<double, Outputs, 1>& Y) {
  Eigen::Matrix<double, Outputs, 1> Ybar =
      Y - (loop.GetPlant().C() * Xhat() + loop.GetPlant().D() * U);
  Eigen::Matrix<double, Outputs, Outputs> S =
      loop.GetPlant().C() * P() * loop.GetPlant().C().transpose() + R();
  Eigen::Matrix<double, States, Outputs> kalmanGain =
      (S.transpose().ldlt().solve(
           (P() * loop.GetPlant().C().transpose()).transpose()))
          .transpose();
  m_Xhat = Xhat() + kalmanGain * Ybar;
  m_P = (loop.GetPlant().GetCoefficients().Acontinuous.Identity() -
         kalmanGain * loop.GetPlant().C()) *
        P();
}

template <int States, int Inputs, int Outputs>
void PeriodVariantKalmanFilter<States, Inputs, Outputs>::AddCoefficients(
    const PeriodVariantKalmanFilterCoeffs<States, Inputs, Outputs>&
        coefficients) {
  m_coefficients.emplace_back(coefficients);
}

template <int States, int Inputs, int Outputs>
const PeriodVariantKalmanFilterCoeffs<States, Inputs, Outputs>&
PeriodVariantKalmanFilter<States, Inputs, Outputs>::GetCoefficients(
    int index) const {
  return m_coefficients[index];
}

template <int States, int Inputs, int Outputs>
const PeriodVariantKalmanFilterCoeffs<States, Inputs, Outputs>&
PeriodVariantKalmanFilter<States, Inputs, Outputs>::GetCoefficients() const {
  return m_coefficients[m_index];
}

template <int States, int Inputs, int Outputs>
void PeriodVariantKalmanFilter<States, Inputs, Outputs>::SetIndex(int index) {
  if (index < 0) {
    m_index = 0;
  } else if (index >= static_cast<int>(m_coefficients.size())) {
    m_index = static_cast<int>(m_coefficients.size()) - 1;
  } else {
    m_index = index;
  }
}

template <int States, int Inputs, int Outputs>
int PeriodVariantKalmanFilter<States, Inputs, Outputs>::GetIndex() const {
  return m_index;
}

/**
 * Rediscretizes Q and R matrices based on provided period.
 */
template <int States, int Inputs, int Outputs>
void PeriodVariantKalmanFilter<States, Inputs, Outputs>::UpdateQR(
    StateFeedbackLoop<States, Inputs, Outputs,
                      PeriodVariantPlant<States, Inputs, Outputs>,
                      PeriodVariantKalmanFilter>* loop,
    std::chrono::nanoseconds dt) {
  using std::chrono::duration_cast;

  // Now, compute the discrete time Q and R coefficients.
  Eigen::Matrix<double, States, States> Qtemp =
      (GetCoefficients().Qcontinuous +
       GetCoefficients().Qcontinuous.transpose()) /
      2.0;
  Eigen::Matrix<double, Outputs, Outputs> Rtemp =
      (GetCoefficients().Rcontinuous +
       GetCoefficients().Rcontinuous.transpose()) /
      2.0;

  Eigen::Matrix<double, 2 * States, 2 * States> Mgain;
  Mgain.setZero();

  // Set up the matrix M = [[-A, Q], [0, A.T]]
  Mgain.template block<States, States>(0, 0) =
      -loop->GetPlant().GetCoefficients().Acontinuous;
  Mgain.template block<States, States>(0, States) = Qtemp;
  Mgain.template block<States, States>(States, States) =
      loop->GetPlant().GetCoefficients().Acontinuous.transpose();

  Eigen::Matrix<double, 2 * States, 2 * States> phi =
      (Mgain * duration_cast<std::chrono::duration<double>>(dt).count()).exp();

  // Phi12 = phi[0:States,        States:2*States]
  // Phi22 = phi[States:2*States, States:2*States]
  Eigen::Matrix<double, States, States> phi12 =
      phi.block(0, States, States, States);
  Eigen::Matrix<double, States, States> phi22 =
      phi.block(States, States, States, States);

  m_Q = phi22.transpose() * phi12;
  m_Q = (m_Q + m_Q.transpose()) / 2.0;
  m_R = Rtemp / duration_cast<std::chrono::duration<double>>(dt).count();
}

}  // namespace frc
