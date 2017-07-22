/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

namespace frc {

template <int States, int Inputs, int Outputs>
PlantCoeffs<States, Inputs, Outputs>::PlantCoeffs(const PlantCoeffs& other)
    : A(other.A),
      Ainv(other.Ainv),
      B(other.B),
      C(other.C),
      D(other.D),
      Umin(other.Umin),
      Umax(other.Umax) {}

template <int States, int Inputs, int Outputs>
PlantCoeffs<States, Inputs, Outputs>::PlantCoeffs(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, States>& Ainv,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, Outputs, States>& C,
    const Eigen::Matrix<double, Outputs, Inputs>& D,
    const Eigen::Matrix<double, Inputs, 1>& Umax,
    const Eigen::Matrix<double, Inputs, 1>& Umin)
    : A(A), Ainv(Ainv), B(B), C(C), D(D), Umin(Umin), Umax(Umax) {}

}  // namespace frc
