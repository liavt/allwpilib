/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <vector>

#include <Eigen/Core>

#include "StateFeedback/PlantCoeffs.h"

namespace frc {

template <int States, int Inputs, int Outputs>
class Plant {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Plant();
  Plant(Plant&& other);

  virtual ~Plant() = default;

  Plant(const Plant&) = delete;
  Plant& operator=(const Plant&) = delete;

  const Eigen::Matrix<double, States, States>& A() const;
  double A(int i, int j) const;
  const Eigen::Matrix<double, States, States>& Ainv() const;
  double Ainv(int i, int j) const;
  const Eigen::Matrix<double, States, Inputs>& B() const;
  double B(int i, int j) const;
  const Eigen::Matrix<double, Outputs, States>& C() const;
  double C(int i, int j) const;
  const Eigen::Matrix<double, Outputs, Inputs>& D() const;
  double D(int i, int j) const;
  const Eigen::Matrix<double, Inputs, 1>& Umin() const;
  double Umin(int i, int j) const;
  const Eigen::Matrix<double, Inputs, 1>& Umax() const;
  double Umax(int i, int j) const;

  const Eigen::Matrix<double, States, 1>& X() const;
  double X(int i, int j) const;
  const Eigen::Matrix<double, Outputs, 1>& Y() const;
  double Y(int i, int j) const;

  Eigen::Matrix<double, States, 1>& MutableX();
  double& MutableX(int i, int j);
  Eigen::Matrix<double, Outputs, 1>& MutableY();
  double& MutableY(int i, int j);

  void AddCoefficients(
      const PlantCoeffs<States, Inputs, Outputs>& coefficients);
  const PlantCoeffs<States, Inputs, Outputs>& GetCoefficients(int index) const;
  const PlantCoeffs<States, Inputs, Outputs>& GetCoefficients() const;

  /**
   * Sets the current plant to be index, clamped to be within range. This can be
   * used for gain scheduling to make the current model match changed plant
   * behavior.
   */
  void SetIndex(int index);

  int GetIndex() const;

  void Reset();

  // Assert that U is within the hardware range.
  virtual void CheckU(const Eigen::Matrix<double, Inputs, 1>& U);

  // Computes the new X and Y given the control input.
  void Update(const Eigen::Matrix<double, Inputs, 1>& U);

  /**
   * Computes the new X given the old X and the control input.
   *
   * This is used by state observers directly to run updates based on state
   * estimate.
   */
  Eigen::Matrix<double, States, 1> UpdateX(
      const Eigen::Matrix<double, States, 1> X,
      const Eigen::Matrix<double, Inputs, 1>& U) const;

  /**
   * Computes the new Y given the control input.
   *
   * This should be used when setting X manually to update Y separately.
   */
  Eigen::Matrix<double, Outputs, 1> UpdateY(
      const Eigen::Matrix<double, Inputs, 1>& U);

 protected:
  // These are accessible from non-templated subclasses.
  static constexpr int kNumStates = States;
  static constexpr int kNumOutputs = Outputs;
  static constexpr int kNumInputs = Inputs;

 private:
  Eigen::Matrix<double, States, 1> m_X;
  Eigen::Matrix<double, Outputs, 1> m_Y;

  std::vector<PlantCoeffs<States, Inputs, Outputs>> m_coefficients;
  int m_index = 0;
};

}  // namespace frc

#include "StateFeedback/Plant.inc"
