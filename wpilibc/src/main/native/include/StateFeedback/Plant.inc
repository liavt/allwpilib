/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cassert>
#include <utility>

#include <HAL/cpp/Log.h>

namespace frc {

template <int States, int Inputs, int Outputs>
Plant<States, Inputs, Outputs>::Plant() {
  Reset();
}

template <int States, int Inputs, int Outputs>
Plant<States, Inputs, Outputs>::Plant(Plant&& other) : m_index(other.m_index) {
  std::swap(m_coefficients, other.m_coefficients);
  m_X.swap(other.m_X);
  m_Y.swap(other.m_Y);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, States>& Plant<States, Inputs, Outputs>::A()
    const {
  return GetCoefficients().A;
}

template <int States, int Inputs, int Outputs>
double Plant<States, Inputs, Outputs>::A(int i, int j) const {
  return A()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, States>&
Plant<States, Inputs, Outputs>::Ainv() const {
  return GetCoefficients().Ainv;
}

template <int States, int Inputs, int Outputs>
double Plant<States, Inputs, Outputs>::Ainv(int i, int j) const {
  return Ainv()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, Inputs>& Plant<States, Inputs, Outputs>::B()
    const {
  return GetCoefficients().B;
}

template <int States, int Inputs, int Outputs>
double Plant<States, Inputs, Outputs>::B(int i, int j) const {
  return B()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Outputs, States>&
Plant<States, Inputs, Outputs>::C() const {
  return GetCoefficients().C;
}

template <int States, int Inputs, int Outputs>
double Plant<States, Inputs, Outputs>::C(int i, int j) const {
  return C()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Outputs, Inputs>&
Plant<States, Inputs, Outputs>::D() const {
  return GetCoefficients().D;
}

template <int States, int Inputs, int Outputs>
double Plant<States, Inputs, Outputs>::D(int i, int j) const {
  return D()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Inputs, 1>& Plant<States, Inputs, Outputs>::Umin()
    const {
  return GetCoefficients().Umin;
}

template <int States, int Inputs, int Outputs>
double Plant<States, Inputs, Outputs>::Umin(int i, int j) const {
  return Umin()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Inputs, 1>& Plant<States, Inputs, Outputs>::Umax()
    const {
  return GetCoefficients().Umax;
}

template <int States, int Inputs, int Outputs>
double Plant<States, Inputs, Outputs>::Umax(int i, int j) const {
  return Umax()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, 1>& Plant<States, Inputs, Outputs>::X()
    const {
  return m_X;
}

template <int States, int Inputs, int Outputs>
double Plant<States, Inputs, Outputs>::X(int i, int j) const {
  return Plant<States, Inputs, Outputs>::X()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Outputs, 1>& Plant<States, Inputs, Outputs>::Y()
    const {
  return m_Y;
}

template <int States, int Inputs, int Outputs>
double Plant<States, Inputs, Outputs>::Y(int i, int j) const {
  return Plant<States, Inputs, Outputs>::Y()(i, j);
}

template <int States, int Inputs, int Outputs>
Eigen::Matrix<double, States, 1>& Plant<States, Inputs, Outputs>::MutableX() {
  return m_X;
}

template <int States, int Inputs, int Outputs>
double& Plant<States, Inputs, Outputs>::MutableX(int i, int j) {
  return MutableX()(i, j);
}

template <int States, int Inputs, int Outputs>
Eigen::Matrix<double, Outputs, 1>& Plant<States, Inputs, Outputs>::MutableY() {
  return m_Y;
}

template <int States, int Inputs, int Outputs>
double& Plant<States, Inputs, Outputs>::MutableY(int i, int j) {
  return MutableY()(i, j);
}

template <int States, int Inputs, int Outputs>
void Plant<States, Inputs, Outputs>::AddCoefficients(
    const PlantCoeffs<States, Inputs, Outputs>& coefficients) {
  m_coefficients.emplace_back(coefficients);
}

template <int States, int Inputs, int Outputs>
const PlantCoeffs<States, Inputs, Outputs>&
Plant<States, Inputs, Outputs>::GetCoefficients(int index) const {
  return m_coefficients[index];
}

template <int States, int Inputs, int Outputs>
const PlantCoeffs<States, Inputs, Outputs>&
Plant<States, Inputs, Outputs>::GetCoefficients() const {
  return m_coefficients[m_index];
}

template <int States, int Inputs, int Outputs>
void Plant<States, Inputs, Outputs>::SetIndex(int index) {
  assert(index >= 0);
  assert(index < static_cast<int>(m_coefficients.size()));
  m_index = index;
}

template <int States, int Inputs, int Outputs>
int Plant<States, Inputs, Outputs>::GetIndex() const {
  return m_index;
}

template <int States, int Inputs, int Outputs>
void Plant<States, Inputs, Outputs>::Reset() {
  m_X.setZero();
  m_Y.setZero();
}

template <int States, int Inputs, int Outputs>
void Plant<States, Inputs, Outputs>::CheckU(
    const Eigen::Matrix<double, Inputs, 1>& U) {
  for (int i = 0; i < kNumInputs; ++i) {
    if (U(i, 0) > Umax(i, 0) + 0.00001 || U(i, 0) < Umin(i, 0) - 0.00001) {
      FILE_LOG(logERROR) << "U out of range\n";
    }
  }
}

template <int States, int Inputs, int Outputs>
void Plant<States, Inputs, Outputs>::Update(
    const Eigen::Matrix<double, Inputs, 1>& U) {
  // Powers outside of the range are more likely controller bugs than things
  // that the plant should deal with.
  CheckU(U);
  m_X = UpdateX(X(), U);
  m_Y = UpdateY(U);
}

template <int States, int Inputs, int Outputs>
Eigen::Matrix<double, States, 1> Plant<States, Inputs, Outputs>::UpdateX(
    const Eigen::Matrix<double, States, 1> X,
    const Eigen::Matrix<double, Inputs, 1>& U) const {
  return A() * X + B() * U;
}

template <int States, int Inputs, int Outputs>
Eigen::Matrix<double, Outputs, 1> Plant<States, Inputs, Outputs>::UpdateY(
    const Eigen::Matrix<double, Inputs, 1>& U) {
  return C() * X() + D() * U;
}

}  // namespace frc
