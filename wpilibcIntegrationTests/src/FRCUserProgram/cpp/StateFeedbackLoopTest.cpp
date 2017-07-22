/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "StateFeedback/StateFeedbackLoop.h"  // NOLINT(build/include_order)

#include "StateFeedback/PeriodVariantKalmanFilter.h"
#include "StateFeedback/PeriodVariantPlant.h"
#include "StateFeedback/Plant.h"
#include "gtest/gtest.h"

frc::PeriodVariantPlantCoeffs<3, 1, 1> MakeIntegralShooterPlantCoeffs() {
  Eigen::Matrix<double, 1, 3> C;
  C(0, 0) = 1.0;
  C(0, 1) = 0.0;
  C(0, 2) = 0.0;

  Eigen::Matrix<double, 1, 1> D;
  D(0, 0) = 0;

  Eigen::Matrix<double, 1, 1> Umax;
  Umax(0, 0) = 12.0;

  Eigen::Matrix<double, 1, 1> Umin;
  Umin(0, 0) = -12.0;

  Eigen::Matrix<double, 3, 3> Acontinuous;
  Acontinuous(0, 0) = 0.0;
  Acontinuous(0, 1) = 1.0;
  Acontinuous(0, 2) = 0.0;
  Acontinuous(1, 0) = 0.0;
  Acontinuous(1, 1) = -8.1021414789556374;
  Acontinuous(1, 2) = 443.75;
  Acontinuous(2, 0) = 0.0;
  Acontinuous(2, 1) = 0.0;
  Acontinuous(2, 2) = 0.0;

  Eigen::Matrix<double, 3, 1> Bcontinuous;
  Bcontinuous(0, 0) = 0.0;
  Bcontinuous(1, 0) = 443.75;
  Bcontinuous(2, 0) = 0.0;

  return frc::PeriodVariantPlantCoeffs<3, 1, 1>(Acontinuous, Bcontinuous, C, D,
                                                Umax, Umin);
}

frc::ControllerCoeffs<3, 1, 1> MakeIntegralShooterControllerCoeffs() {
  Eigen::Matrix<double, 1, 3> K;
  K(0, 0) = 0.0;
  K(0, 1) = 0.027731156542808996;
  K(0, 2) = 1.0;

  Eigen::Matrix<double, 1, 3> Kff;
  Kff(0, 0) = 0.0;
  Kff(0, 1) = 0.45989503537638587;
  Kff(0, 2) = 0.0;

  return frc::ControllerCoeffs<3, 1, 1>(K, Kff);
}

frc::PeriodVariantKalmanFilterCoeffs<3, 1, 1>
MakeIntegralShooterObserverCoeffs() {
  Eigen::Matrix<double, 3, 3> Qcontinuous;
  Qcontinuous(0, 0) = 0.0001;
  Qcontinuous(0, 1) = 0.0;
  Qcontinuous(0, 2) = 0.0;
  Qcontinuous(1, 0) = 0.0;
  Qcontinuous(1, 1) = 4.0;
  Qcontinuous(1, 2) = 0.0;
  Qcontinuous(2, 0) = 0.0;
  Qcontinuous(2, 1) = 0.0;
  Qcontinuous(2, 2) = 0.040000000000000008;

  Eigen::Matrix<double, 1, 1> Rcontinuous;
  Rcontinuous(0, 0) = 9.9999999999999995e-07;

  Eigen::Matrix<double, 3, 3> PsteadyState;
  PsteadyState(0, 0) = 7.1645559451160497e-05;
  PsteadyState(0, 1) = 0.0031205034236441768;
  PsteadyState(0, 2) = 0.00016022137220036598;
  PsteadyState(1, 0) = 0.0031205034236441768;
  PsteadyState(1, 1) = 0.25313549121689616;
  PsteadyState(1, 2) = 0.015962850974712596;
  PsteadyState(2, 0) = 0.00016022137220036598;
  PsteadyState(2, 1) = 0.015962850974712596;
  PsteadyState(2, 2) = 0.0019821816120708254;

  return frc::PeriodVariantKalmanFilterCoeffs<3, 1, 1>(Qcontinuous, Rcontinuous,
                                                       PsteadyState);
}

frc::PeriodVariantPlant<3, 1, 1> MakeIntegralShooterPlant() {
  frc::PeriodVariantPlant<3, 1, 1> plant;
  plant.AddCoefficients(MakeIntegralShooterPlantCoeffs());

  return plant;
}

frc::Controller<3, 1, 1> MakeIntegralShooterController() {
  frc::Controller<3, 1, 1> controller;
  controller.AddCoefficients(MakeIntegralShooterControllerCoeffs());

  return controller;
}

frc::PeriodVariantKalmanFilter<3, 1, 1> MakeIntegralShooterObserver() {
  frc::PeriodVariantKalmanFilter<3, 1, 1> observer;
  observer.AddCoefficients(MakeIntegralShooterObserverCoeffs());

  return observer;
}

frc::StateFeedbackLoop<3, 1, 1, frc::PeriodVariantPlant<3, 1, 1>,
                       frc::PeriodVariantKalmanFilter<3, 1, 1>>
MakeIntegralShooterLoop() {
  return frc::StateFeedbackLoop<3, 1, 1, frc::PeriodVariantPlant<3, 1, 1>,
                                frc::PeriodVariantKalmanFilter<3, 1, 1>>(
      MakeIntegralShooterPlant(), MakeIntegralShooterController(),
      MakeIntegralShooterObserver());
}

// Tests that everything compiles and nothing crashes even if Inputs != Outputs.
TEST(StateFeedbackLoopTest, UnequalSizes) {
  // Create a plant
  frc::Plant<2, 4, 7> plant;
  const frc::PlantCoeffs<2, 4, 7> plantCoeffs(
      Eigen::Matrix<double, 2, 2>::Identity(),
      Eigen::Matrix<double, 2, 2>::Identity(),
      Eigen::Matrix<double, 2, 4>::Identity(),
      Eigen::Matrix<double, 7, 2>::Identity(),
      Eigen::Matrix<double, 7, 4>::Identity(),
      Eigen::Matrix<double, 4, 1>::Constant(1),
      Eigen::Matrix<double, 4, 1>::Constant(-1));
  plant.AddCoefficients(plantCoeffs);
  plant.Update(Eigen::Matrix<double, 4, 1>::Zero());
  plant.Reset();
  plant.CheckU(Eigen::Matrix<double, 4, 1>::Zero());

  // Create a controller
  frc::Controller<2, 4, 7> controller;
  const frc::ControllerCoeffs<2, 4, 7> controllerCoeffs(
      Eigen::Matrix<double, 4, 2>::Identity(),
      Eigen::Matrix<double, 4, 2>::Identity());
  controller.AddCoefficients(controllerCoeffs);

  // Create an observer
  frc::LuenbergerObserver<2, 4, 7> observer;
  const frc::LuenbergerObserverCoeffs<2, 4, 7> observerCoeffs(
      Eigen::Matrix<double, 2, 7>::Identity());
  observer.AddCoefficients(observerCoeffs);

  // Create a feedback loop using the plant, controller, and observer
  frc::StateFeedbackLoop<2, 4, 7> testLoop(
      std::move(plant), std::move(controller), std::move(observer));
  testLoop.Correct(Eigen::Matrix<double, 7, 1>::Identity());
  testLoop.Update(false);
  testLoop.CapU();
}

// Tests that the continuous to discrete calculation for the Kalman filter
// matches what was computed both in Python and in MATLAB.
TEST(StateFeedbackLoopTest, PythonMatch) {
  auto testLoop = MakeIntegralShooterLoop();
  testLoop.Update(false, std::chrono::milliseconds(5));

  Eigen::Matrix<double, 3, 3> Adiscrete;
  Adiscrete << 1, 0.00490008, 0.00547272, 0, 0.96029888, 2.17440921, 0, 0, 1;

  Eigen::Matrix<double, 3, 1> Bdiscrete;
  Bdiscrete << 0.00547272, 2.17440921, 0;

  Eigen::Matrix<double, 3, 3> Qdiscrete;
  Qdiscrete << 6.62900602e-07, 4.86205253e-05, 3.66076676e-07, 4.86205253e-05,
      1.95296358e-02, 2.18908995e-04, 3.66076676e-07, 2.18908995e-04,
      2.00000000e-04;

  Eigen::Matrix<double, 1, 1> Rdiscrete;
  Rdiscrete << 0.0002;

  EXPECT_TRUE(Adiscrete.isApprox(testLoop.GetPlant().A(), 0.001));
  EXPECT_TRUE(Bdiscrete.isApprox(testLoop.GetPlant().B(), 0.001));
  EXPECT_TRUE(Qdiscrete.isApprox(testLoop.GetObserver().Q(), 0.001));
  EXPECT_TRUE(Rdiscrete.isApprox(testLoop.GetObserver().R(), 0.001));
}
