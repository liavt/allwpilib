/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

namespace frc {

// A container for all the controller coefficients.
template <int States, int Inputs, int Outputs>
ControllerCoeffs<States, Inputs, Outputs>::ControllerCoeffs(
    const Eigen::Matrix<double, Inputs, States>& K,
    const Eigen::Matrix<double, Inputs, States>& Kff)
    : K(K), Kff(Kff) {}

}  // namespace frc
