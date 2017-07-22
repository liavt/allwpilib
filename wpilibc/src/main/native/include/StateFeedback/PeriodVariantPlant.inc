/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cassert>
#include <utility>

#include <Eigen/MatrixFunctions>
#include <HAL/cpp/Log.h>

#include "DriverStation.h"

namespace frc {

template <int States, int Inputs, int Outputs>
PeriodVariantPlant<States, Inputs, Outputs>::PeriodVariantPlant(
    const std::chrono::nanoseconds nominalLoopPeriod)
    : m_nominalLoopPeriod(nominalLoopPeriod) {
  Reset();
}

template <int States, int Inputs, int Outputs>
PeriodVariantPlant<States, Inputs, Outputs>::PeriodVariantPlant(
    PeriodVariantPlant&& rhs)
    : m_nominalLoopPeriod(rhs.m_nominalLoopPeriod), m_index(rhs.m_index) {
  std::swap(m_coefficients, rhs.m_coefficients);
  m_X.swap(rhs.m_X);
  m_Y.swap(rhs.m_Y);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, States>&
PeriodVariantPlant<States, Inputs, Outputs>::A() const {
  return m_A;
}

template <int States, int Inputs, int Outputs>
double PeriodVariantPlant<States, Inputs, Outputs>::A(int i, int j) const {
  return A()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, Inputs>&
PeriodVariantPlant<States, Inputs, Outputs>::B() const {
  return m_B;
}

template <int States, int Inputs, int Outputs>
double PeriodVariantPlant<States, Inputs, Outputs>::B(int i, int j) const {
  return B()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Outputs, States>&
PeriodVariantPlant<States, Inputs, Outputs>::C() const {
  return GetCoefficients().C;
}

template <int States, int Inputs, int Outputs>
double PeriodVariantPlant<States, Inputs, Outputs>::C(int i, int j) const {
  return C()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Outputs, Inputs>&
PeriodVariantPlant<States, Inputs, Outputs>::D() const {
  return GetCoefficients().D;
}

template <int States, int Inputs, int Outputs>
double PeriodVariantPlant<States, Inputs, Outputs>::D(int i, int j) const {
  return D()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Inputs, 1>&
PeriodVariantPlant<States, Inputs, Outputs>::Umin() const {
  return GetCoefficients().Umin;
}

template <int States, int Inputs, int Outputs>
double PeriodVariantPlant<States, Inputs, Outputs>::Umin(int i, int j) const {
  return Umin()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Inputs, 1>&
PeriodVariantPlant<States, Inputs, Outputs>::Umax() const {
  return GetCoefficients().Umax;
}

template <int States, int Inputs, int Outputs>
double PeriodVariantPlant<States, Inputs, Outputs>::Umax(int i, int j) const {
  return Umax()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, 1>&
PeriodVariantPlant<States, Inputs, Outputs>::X() const {
  return m_X;
}

template <int States, int Inputs, int Outputs>
double PeriodVariantPlant<States, Inputs, Outputs>::X(int i, int j) const {
  return X()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Outputs, 1>&
PeriodVariantPlant<States, Inputs, Outputs>::Y() const {
  return m_Y;
}

template <int States, int Inputs, int Outputs>
double PeriodVariantPlant<States, Inputs, Outputs>::Y(int i, int j) const {
  return Y()(i, j);
}

template <int States, int Inputs, int Outputs>
Eigen::Matrix<double, States, 1>&
PeriodVariantPlant<States, Inputs, Outputs>::MutableX() {
  return m_X;
}

template <int States, int Inputs, int Outputs>
double& PeriodVariantPlant<States, Inputs, Outputs>::MutableX(int i, int j) {
  return MutableX()(i, j);
}

template <int States, int Inputs, int Outputs>
Eigen::Matrix<double, Outputs, 1>&
PeriodVariantPlant<States, Inputs, Outputs>::MutableY() {
  return m_Y;
}

template <int States, int Inputs, int Outputs>
double& PeriodVariantPlant<States, Inputs, Outputs>::MutableY(int i, int j) {
  return MutableY()(i, j);
}

template <int States, int Inputs, int Outputs>
void PeriodVariantPlant<States, Inputs, Outputs>::AddCoefficients(
    const PeriodVariantPlantCoeffs<States, Inputs, Outputs>& coefficients) {
  m_coefficients.emplace_back(coefficients);
}

template <int States, int Inputs, int Outputs>
const PeriodVariantPlantCoeffs<States, Inputs, Outputs>&
PeriodVariantPlant<States, Inputs, Outputs>::GetCoefficients(int index) const {
  return m_coefficients[index];
}

template <int States, int Inputs, int Outputs>
const PeriodVariantPlantCoeffs<States, Inputs, Outputs>&
PeriodVariantPlant<States, Inputs, Outputs>::GetCoefficients() const {
  return m_coefficients[m_index];
}

template <int States, int Inputs, int Outputs>
void PeriodVariantPlant<States, Inputs, Outputs>::SetIndex(int index) {
  assert(index >= 0);
  assert(index < static_cast<int>(m_coefficients.size()));
  m_index = index;
}

template <int States, int Inputs, int Outputs>
int PeriodVariantPlant<States, Inputs, Outputs>::GetIndex() const {
  return m_index;
}

template <int States, int Inputs, int Outputs>
void PeriodVariantPlant<States, Inputs, Outputs>::Reset() {
  m_X.setZero();
  m_Y.setZero();
  m_A.setZero();
  m_B.setZero();
  m_delayedU.setZero();
  UpdateAB(m_nominalLoopPeriod);
}

template <int States, int Inputs, int Outputs>
void PeriodVariantPlant<States, Inputs, Outputs>::CheckU(
    const Eigen::Matrix<double, Inputs, 1>& U) {
  for (int i = 0; i < kNumInputs; ++i) {
    if (U(i, 0) > Umax(i, 0) + 0.00001 || U(i, 0) < Umin(i, 0) - 0.00001) {
      FILE_LOG(logERROR) << "U out of range\n";
    }
  }
}

template <int States, int Inputs, int Outputs>
void PeriodVariantPlant<States, Inputs, Outputs>::Update(
    const Eigen::Matrix<double, Inputs, 1>& U, std::chrono::nanoseconds dt) {
  // Powers outside of the range are more likely controller bugs than things
  // that the plant should deal with.
  CheckU(U);

  Eigen::Matrix<double, Inputs, 1> currentU =
      m_delayedU * DriverStation::GetInstance().GetBatteryVoltage() / 12.0;
  m_X = UpdateX(X(), currentU);
  m_Y = UpdateY(currentU);
  m_delayedU = U;
}

template <int States, int Inputs, int Outputs>
Eigen::Matrix<double, States, 1>
PeriodVariantPlant<States, Inputs, Outputs>::UpdateX(
    const Eigen::Matrix<double, States, 1>& X,
    const Eigen::Matrix<double, Inputs, 1>& U, std::chrono::nanoseconds dt) {
  UpdateAB(dt);
  return A() * X + B() * U;
}

template <int States, int Inputs, int Outputs>
Eigen::Matrix<double, Outputs, 1>
PeriodVariantPlant<States, Inputs, Outputs>::UpdateY(
    const Eigen::Matrix<double, Inputs, 1>& U) {
  return C() * X() + D() * U;
}

template <int States, int Inputs, int Outputs>
void PeriodVariantPlant<States, Inputs, Outputs>::UpdateAB(
    std::chrono::nanoseconds dt) {
  // Matrices are blocked here to minimize matrix exponentiation calculations.
  Eigen::Matrix<double, States + Inputs, States + Inputs> MstateContinuous;
  MstateContinuous.setZero();
  MstateContinuous.template block<States, States>(0, 0) =
      GetCoefficients().Acontinuous *
      std::chrono::duration_cast<std::chrono::duration<double>>(dt).count();
  MstateContinuous.template block<States, Inputs>(0, States) =
      GetCoefficients().Bcontinuous *
      std::chrono::duration_cast<std::chrono::duration<double>>(dt).count();

  Eigen::Matrix<double, States + Inputs, States + Inputs> Mstate =
      MstateContinuous.exp();
  m_A = Mstate.template block<States, States>(0, 0);
  m_B = Mstate.template block<States, Inputs>(0, States);
}

}  // namespace frc
