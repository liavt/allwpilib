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
struct PeriodVariantPlantCoeffs final {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PeriodVariantPlantCoeffs(const PeriodVariantPlantCoeffs& rhs);
  PeriodVariantPlantCoeffs(
      const Eigen::Matrix<double, States, States>& Acontinuous,
      const Eigen::Matrix<double, States, Inputs>& Bcontinuous,
      const Eigen::Matrix<double, Outputs, States>& C,
      const Eigen::Matrix<double, Outputs, Inputs>& D,
      const Eigen::Matrix<double, Inputs, 1>& Umax,
      const Eigen::Matrix<double, Inputs, 1>& Umin);

  const Eigen::Matrix<double, States, States> Acontinuous;
  const Eigen::Matrix<double, States, Inputs> Bcontinuous;
  const Eigen::Matrix<double, Outputs, States> C;
  const Eigen::Matrix<double, Outputs, Inputs> D;
  const Eigen::Matrix<double, Inputs, 1> Umin;
  const Eigen::Matrix<double, Inputs, 1> Umax;
};

}  // namespace frc

#include "StateFeedback/PeriodVariantPlantCoeffs.inc"
