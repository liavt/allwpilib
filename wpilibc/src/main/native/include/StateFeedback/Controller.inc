/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <utility>
#include <vector>

namespace frc {

template <int States, int Inputs, int Outputs>
Controller<States, Inputs, Outputs>::Controller(Controller&& rhs)
    : m_index(rhs.m_index) {
  std::swap(m_coefficients, rhs.m_coefficients);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Inputs, States>&
Controller<States, Inputs, Outputs>::K() const {
  return GetCoefficients().K;
}

template <int States, int Inputs, int Outputs>
double Controller<States, Inputs, Outputs>::K(int i, int j) const {
  return K()(i, j);
}

template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, Inputs, States>&
Controller<States, Inputs, Outputs>::Kff() const {
  return GetCoefficients().Kff;
}

template <int States, int Inputs, int Outputs>
double Controller<States, Inputs, Outputs>::Kff(int i, int j) const {
  return Kff()(i, j);
}

template <int States, int Inputs, int Outputs>
void Controller<States, Inputs, Outputs>::Reset() {}

template <int States, int Inputs, int Outputs>
void Controller<States, Inputs, Outputs>::AddCoefficients(
    const ControllerCoeffs<States, Inputs, Outputs>& coefficients) {
  m_coefficients.emplace_back(coefficients);
}

template <int States, int Inputs, int Outputs>
const ControllerCoeffs<States, Inputs, Outputs>&
Controller<States, Inputs, Outputs>::GetCoefficients(int index) const {
  return m_coefficients[index];
}

template <int States, int Inputs, int Outputs>
const ControllerCoeffs<States, Inputs, Outputs>&
Controller<States, Inputs, Outputs>::GetCoefficients() const {
  return m_coefficients[m_index];
}

template <int States, int Inputs, int Outputs>
void Controller<States, Inputs, Outputs>::SetIndex(int index) {
  if (index < 0) {
    m_index = 0;
  } else if (index >= static_cast<int>(m_coefficients.size())) {
    m_index = static_cast<int>(m_coefficients.size()) - 1;
  } else {
    m_index = index;
  }
}

template <int States, int Inputs, int Outputs>
int Controller<States, Inputs, Outputs>::GetIndex() const {
  return m_index;
}

}  // namespace frc
