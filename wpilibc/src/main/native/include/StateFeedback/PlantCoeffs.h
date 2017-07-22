/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Eigen/Core>

namespace frc {

template <int States, int Inputs, int Outputs>
struct PlantCoeffs final {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PlantCoeffs(const PlantCoeffs& other);
  PlantCoeffs(const Eigen::Matrix<double, States, States>& A,
              const Eigen::Matrix<double, States, States>& Ainv,
              const Eigen::Matrix<double, States, Inputs>& B,
              const Eigen::Matrix<double, Outputs, States>& C,
              const Eigen::Matrix<double, Outputs, Inputs>& D,
              const Eigen::Matrix<double, Inputs, 1>& Umax,
              const Eigen::Matrix<double, Inputs, 1>& Umin);

  // System matrix
  const Eigen::Matrix<double, States, States> A;

  // Inverse system matrix (A^-1)
  const Eigen::Matrix<double, States, States> Ainv;

  // Input matrix
  const Eigen::Matrix<double, States, Inputs> B;

  // Output matrix
  const Eigen::Matrix<double, Outputs, States> C;

  // Feedthrough matrix
  const Eigen::Matrix<double, Outputs, Inputs> D;

  // Minimum control input
  const Eigen::Matrix<double, Inputs, 1> Umin;

  // Maximum control input
  const Eigen::Matrix<double, Inputs, 1> Umax;
};

}  // namespace frc

#include "StateFeedback/PlantCoeffs.inc"
