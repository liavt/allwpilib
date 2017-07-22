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

#include "StateFeedback/PeriodVariantKalmanFilterCoeffs.h"
#include "StateFeedback/PeriodVariantPlant.h"

namespace frc {

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
class StateFeedbackLoop;

/**
 * A Kalman filter that discretizes its model and covariance matrices after
 * every sample period.
 *
 * Typical discrete state feedback implementations discretize with a nominal
 * sample period offline. If the real system doesn't maintain this period, this
 * nonlinearity can negatively affect the state estimate. This class discretizes
 * the continuous model after each measurement based on the measured sample
 * period.
 *
 * Since the sample period changes during runtime, the process and measurement
 * noise covariance matrices as well as the true steady state error covariance
 * change as well. During runtime, the error covariance matrix is initialized
 * with the discrete steady state value, but is updated during runtime as well.
 */
template <int States, int Inputs, int Outputs>
class PeriodVariantKalmanFilter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PeriodVariantKalmanFilter(const std::chrono::nanoseconds nominalLoopPeriod =
                                std::chrono::milliseconds(5));
  PeriodVariantKalmanFilter(PeriodVariantKalmanFilter&& rhs);

  // Getters for Q
  const Eigen::Matrix<double, States, States>& Q() const;
  double Q(int i, int j) const;

  // Getters for R
  const Eigen::Matrix<double, Outputs, Outputs>& R() const;
  double R(int i, int j) const;

  // Getters for P
  const Eigen::Matrix<double, States, States>& P() const;
  double P(int i, int j) const;

  const Eigen::Matrix<double, States, 1>& Xhat() const;
  Eigen::Matrix<double, States, 1>& MutableXhat();

  void Reset(StateFeedbackLoop<States, Inputs, Outputs,
                               PeriodVariantPlant<States, Inputs, Outputs>,
                               PeriodVariantKalmanFilter>* loop);

  void Predict(StateFeedbackLoop<States, Inputs, Outputs,
                                 PeriodVariantPlant<States, Inputs, Outputs>,
                                 PeriodVariantKalmanFilter>* loop,
               const Eigen::Matrix<double, Inputs, 1>& newU,
               std::chrono::nanoseconds dt);

  void Correct(
      const StateFeedbackLoop<States, Inputs, Outputs,
                              PeriodVariantPlant<States, Inputs, Outputs>,
                              PeriodVariantKalmanFilter>& loop,
      const Eigen::Matrix<double, Inputs, 1>& U,
      const Eigen::Matrix<double, Outputs, 1>& Y);

  void AddCoefficients(
      const PeriodVariantKalmanFilterCoeffs<States, Inputs, Outputs>&
          coefficients);
  const PeriodVariantKalmanFilterCoeffs<States, Inputs, Outputs>&
  GetCoefficients(int index) const;
  const PeriodVariantKalmanFilterCoeffs<States, Inputs, Outputs>&
  GetCoefficients() const;

  /**
   * Sets the current observer to be index, clamped to be within range. This can
   * be used for gain scheduling.
   */
  void SetIndex(int index);

  int GetIndex() const;

 private:
  void UpdateQR(StateFeedbackLoop<States, Inputs, Outputs,
                                  PeriodVariantPlant<States, Inputs, Outputs>,
                                  PeriodVariantKalmanFilter>* loop,
                std::chrono::nanoseconds dt);

  const std::chrono::nanoseconds m_nominalLoopPeriod;

  // Internal state estimate.
  Eigen::Matrix<double, States, 1> m_Xhat;

  // Internal error covariance estimate.
  Eigen::Matrix<double, States, States> m_P;

  // Discretized Q and R for the kalman filter.
  Eigen::Matrix<double, States, States> m_Q;
  Eigen::Matrix<double, Outputs, Outputs> m_R;

  std::vector<PeriodVariantKalmanFilterCoeffs<States, Inputs, Outputs>>
      m_coefficients;
  int m_index = 0;
};

}  // namespace frc

#include "StateFeedback/PeriodVariantKalmanFilter.inc"
