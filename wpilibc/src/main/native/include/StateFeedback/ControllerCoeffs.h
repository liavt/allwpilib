/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Eigen/Core>

namespace frc {

// A container for all the controller coefficients.
template <int States, int Inputs, int Outputs>
struct ControllerCoeffs final {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Controller gain matrix
  const Eigen::Matrix<double, Inputs, States> K;

  // Controller feedforward gain matrix
  const Eigen::Matrix<double, Inputs, States> Kff;

  ControllerCoeffs(const Eigen::Matrix<double, Inputs, States>& K,
                   const Eigen::Matrix<double, Inputs, States>& Kff);
};

}  // namespace frc

#include "StateFeedback/ControllerCoeffs.inc"
