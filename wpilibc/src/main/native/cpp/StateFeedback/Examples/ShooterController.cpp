/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <chrono>
#include <cmath>

#include "StateFeedback/Examples/Shooter.h"

using namespace frc;

constexpr std::chrono::nanoseconds ShooterController::kLoopPeriod;

namespace {
constexpr double kTolerance = 10.0;
}  // namespace

PeriodVariantPlantCoeffs<3, 1, 1> MakeIntegralShooterPlantCoeffs() {
  Eigen::Matrix<double, 1, 3> C;
  C(0, 0) = 1.0;
  C(0, 1) = 0.0;
  C(0, 2) = 0.0;
  Eigen::Matrix<double, 1, 1> D;
  D(0, 0) = 0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max(0, 0) = 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min(0, 0) = -12.0;
  Eigen::Matrix<double, 3, 3> A_continuous;
  A_continuous(0, 0) = 0.0;
  A_continuous(0, 1) = 1.0;
  A_continuous(0, 2) = 0.0;
  A_continuous(1, 0) = 0.0;
  A_continuous(1, 1) = -8.1021414789556374;
  A_continuous(1, 2) = 443.75;
  A_continuous(2, 0) = 0.0;
  A_continuous(2, 1) = 0.0;
  A_continuous(2, 2) = 0.0;
  Eigen::Matrix<double, 3, 1> B_continuous;
  B_continuous(0, 0) = 0.0;
  B_continuous(1, 0) = 443.75;
  B_continuous(2, 0) = 0.0;
  return PeriodVariantPlantCoeffs<3, 1, 1>(A_continuous, B_continuous, C, D,
                                           U_max, U_min);
}

ControllerCoeffs<3, 1, 1> MakeIntegralShooterControllerCoeffs() {
  Eigen::Matrix<double, 1, 3> K;
  K(0, 0) = 0.0;
  K(0, 1) = 0.027731156542808996;
  K(0, 2) = 1.0;
  Eigen::Matrix<double, 1, 3> Kff;
  Kff(0, 0) = 0.0;
  Kff(0, 1) = 0.45989503537638587;
  Kff(0, 2) = 0.0;
  return ControllerCoeffs<3, 1, 1>(K, Kff);
}

PeriodVariantKalmanFilterCoeffs<3, 1, 1> MakeIntegralShooterObserverCoeffs() {
  Eigen::Matrix<double, 3, 3> Q_continuous;
  Q_continuous(0, 0) = 0.0001;
  Q_continuous(0, 1) = 0.0;
  Q_continuous(0, 2) = 0.0;
  Q_continuous(1, 0) = 0.0;
  Q_continuous(1, 1) = 4.0;
  Q_continuous(1, 2) = 0.0;
  Q_continuous(2, 0) = 0.0;
  Q_continuous(2, 1) = 0.0;
  Q_continuous(2, 2) = 0.040000000000000008;
  Eigen::Matrix<double, 1, 1> R_continuous;
  R_continuous(0, 0) = 9.9999999999999995e-07;
  Eigen::Matrix<double, 3, 3> P_steady_state;
  P_steady_state(0, 0) = 7.1645559451160497e-05;
  P_steady_state(0, 1) = 0.0031205034236441768;
  P_steady_state(0, 2) = 0.00016022137220036598;
  P_steady_state(1, 0) = 0.0031205034236441768;
  P_steady_state(1, 1) = 0.25313549121689616;
  P_steady_state(1, 2) = 0.015962850974712596;
  P_steady_state(2, 0) = 0.00016022137220036598;
  P_steady_state(2, 1) = 0.015962850974712596;
  P_steady_state(2, 2) = 0.0019821816120708254;
  return PeriodVariantKalmanFilterCoeffs<3, 1, 1>(Q_continuous, R_continuous,
                                                  P_steady_state);
}

PeriodVariantPlant<3, 1, 1> MakeIntegralShooterPlant() {
  PeriodVariantPlant<3, 1, 1> plant;
  plant.AddCoefficients(MakeIntegralShooterPlantCoeffs());
  return plant;
}

Controller<3, 1, 1> MakeIntegralShooterController() {
  Controller<3, 1, 1> controller;
  controller.AddCoefficients(MakeIntegralShooterControllerCoeffs());
  return controller;
}

PeriodVariantKalmanFilter<3, 1, 1> MakeIntegralShooterObserver() {
  PeriodVariantKalmanFilter<3, 1, 1> observer;
  observer.AddCoefficients(MakeIntegralShooterObserverCoeffs());
  return observer;
}

StateFeedbackLoop<3, 1, 1, PeriodVariantPlant<3, 1, 1>,
                  PeriodVariantKalmanFilter<3, 1, 1>>
MakeIntegralShooterLoop() {
  return StateFeedbackLoop<3, 1, 1, PeriodVariantPlant<3, 1, 1>,
                           PeriodVariantKalmanFilter<3, 1, 1>>(
      MakeIntegralShooterPlant(), MakeIntegralShooterController(),
      MakeIntegralShooterObserver());
}

// TODO(austin): Pseudo current limit?

ShooterController::ShooterController()
    : m_loop(new StateFeedbackLoop<3, 1, 1, PeriodVariantPlant<3, 1, 1>,
                                   PeriodVariantKalmanFilter<3, 1, 1>>(
          MakeIntegralShooterLoop())) {
  m_history.fill(0);
  m_Y.setZero();
}

void ShooterController::SetGoal(double angularVelocityGoal) {
  m_loop->MutableNextR() << 0.0, angularVelocityGoal, angularVelocityGoal, 0.0;
}

void ShooterController::SetPosition(double currentPosition) {
  // Update position in the model.
  m_Y << currentPosition;

  // Add the position to the history.
  m_history[m_historyPosition] = currentPosition;
  m_historyPosition = (m_historyPosition + 1) % kHistoryLength;

  m_dtPosition = currentPosition - m_lastPosition;
  m_lastPosition = currentPosition;
}

double ShooterController::Voltage() const { return m_loop->U(0, 0); }

void ShooterController::Reset() { m_reset = true; }

void ShooterController::Update(bool disable, std::chrono::nanoseconds dt) {
  using std::chrono::duration_cast;

  m_loop->MutableR() = m_loop->NextR();
  if (std::abs(m_loop->R(2, 0)) < 1.0) {
    // Kill power at low angular velocities.
    disable = true;
  }

  m_loop->Correct(m_Y);

  // Compute the oldest point in the history.
  const int oldestHistoryPosition =
      m_historyPosition == 0 ? kHistoryLength - 1 : m_historyPosition - 1;

  // Compute the distance moved over that time period.
  m_averageAngularVelocity =
      (m_history[oldestHistoryPosition] - m_history[m_historyPosition]) /
      (duration_cast<std::chrono::duration<double>>(kLoopPeriod).count() *
       static_cast<double>(kHistoryLength - 1));

  // Ready if average angular velocity is close to the goal.
  m_error = m_averageAngularVelocity - m_loop->NextR(2, 0);

  m_ready = std::abs(m_error) < kTolerance && m_loop->NextR(2, 0) > 1.0;

  // If we are no longer ready, but were, and are spinning, then we shot a ball.
  // Reset the KF.
  if (m_lastReady && !m_ready && m_loop->NextR(2, 0) > 1.0 && m_error < 0.0) {
    m_needsReset = true;
    m_minVelocity = Velocity();
  }
  if (m_needsReset) {
    m_minVelocity = std::min(m_minVelocity, Velocity());
    if (Velocity() > m_minVelocity + 5.0) {
      m_reset = true;
      m_needsReset = false;
    }
  }
  if (m_reset) {
    // TODO(austin): I'd rather not be incrementing X_hat each time. Sort out
    // something better.
    m_loop->MutableXhat(3, 0) += 1.0;
    m_reset = false;
  }
  m_lastReady = m_ready;

  m_XhatCurrent = m_loop->Xhat();
  m_positionError = m_XhatCurrent(0, 0) - m_Y(0, 0);
  m_dtVelocity =
      m_dtPosition / duration_cast<std::chrono::duration<double>>(dt).count();
  m_fixedDtVelocity = m_dtPosition / 0.00505;

  m_loop->Update(disable, dt);
}
