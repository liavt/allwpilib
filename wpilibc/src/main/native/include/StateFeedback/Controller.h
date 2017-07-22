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

#include <Eigen/Core>

#include "StateFeedback/ControllerCoeffs.h"

namespace frc {

template <int States, int Inputs, int Outputs>
class Controller {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Controller() = default;
  Controller(Controller&& rhs);

  const Eigen::Matrix<double, Inputs, States>& K() const;
  double K(int i, int j) const;
  const Eigen::Matrix<double, Inputs, States>& Kff() const;
  double Kff(int i, int j) const;

  void Reset();

  /**
   * Sets the current controller to be index, clamped to be within range. This
   * can be used for gain scheduling.
   */
  void SetIndex(int index);

  int GetIndex() const;

  void AddCoefficients(
      const ControllerCoeffs<States, Inputs, Outputs>& coefficients);
  const ControllerCoeffs<States, Inputs, Outputs>& GetCoefficients(
      int index) const;
  const ControllerCoeffs<States, Inputs, Outputs>& GetCoefficients() const;

 private:
  std::vector<ControllerCoeffs<States, Inputs, Outputs>> m_coefficients;
  int m_index = 0;
};

}  // namespace frc

#include "StateFeedback/Controller.inc"
