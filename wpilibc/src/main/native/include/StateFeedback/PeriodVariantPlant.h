/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <chrono>
#include <vector>

#include <Eigen/Core>

#include "PeriodVariantPlantCoeffs.h"

namespace frc {

/**
 * A plant which discretizes its model after every sample period.
 *
 * Typical discrete state feedback implementations discretize with a nominal
 * sample period offline. If the real system doesn't maintain this period, this
 * nonlinearity can negatively affect the state estimate. This class discretizes
 * the continuous model after each measurement based on the measured sample
 * period.
 */
template <int States, int Inputs, int Outputs>
class PeriodVariantPlant {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PeriodVariantPlant(const std::chrono::nanoseconds nominalLoopPeriod =
                         std::chrono::milliseconds(5));
  PeriodVariantPlant(PeriodVariantPlant&& rhs);

  virtual ~PeriodVariantPlant() = default;

  PeriodVariantPlant(const PeriodVariantPlant&) = delete;
  PeriodVariantPlant& operator=(const PeriodVariantPlant&) = delete;

  const Eigen::Matrix<double, States, States>& A() const;
  double A(int i, int j) const;
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
      const PeriodVariantPlantCoeffs<States, Inputs, Outputs>& coefficients);
  const PeriodVariantPlantCoeffs<States, Inputs, Outputs>& GetCoefficients(
      int index) const;
  const PeriodVariantPlantCoeffs<States, Inputs, Outputs>& GetCoefficients()
      const;

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
  void Update(const Eigen::Matrix<double, Inputs, 1>& U,
              std::chrono::nanoseconds dt);

  /**
   * Computes the new X given the old X and the control input.
   *
   * This is used by state observers directly to run updates based on state
   * estimate.
   */
  Eigen::Matrix<double, States, 1> UpdateX(
      const Eigen::Matrix<double, States, 1>& X,
      const Eigen::Matrix<double, Inputs, 1>& U, std::chrono::nanoseconds dt);

  /**
   * Computes the new Y given the control input.
   *
   * This should be used when setting X manually to update Y separately.
   */
  Eigen::Matrix<double, Outputs, 1> UpdateY(
      const Eigen::Matrix<double, Inputs, 1>& U);

 protected:
  // These are accessible from non-templated subclasses.
  static const int kNumStates = States;
  static const int kNumOutputs = Outputs;
  static const int kNumInputs = Inputs;

 private:
  void UpdateAB(std::chrono::nanoseconds dt);

  const std::chrono::nanoseconds m_nominalLoopPeriod;

  // Current state
  Eigen::Matrix<double, States, 1> m_X;

  // Current output
  Eigen::Matrix<double, Outputs, 1> m_Y;

  // Discrete A and B matrices
  Eigen::Matrix<double, States, States> m_A;
  Eigen::Matrix<double, States, Inputs> m_B;

  Eigen::Matrix<double, Inputs, 1> m_delayedU;

  std::vector<PeriodVariantPlantCoeffs<States, Inputs, Outputs>> m_coefficients;
  int m_index = 0;
};

}  // namespace frc

#include "StateFeedback/PeriodVariantPlant.inc"
