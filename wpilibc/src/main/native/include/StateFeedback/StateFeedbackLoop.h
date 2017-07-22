/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <chrono>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "StateFeedback/Controller.h"
#include "StateFeedback/LuenbergerObserver.h"
#include "StateFeedback/Plant.h"

namespace frc {

/**
 * Combines a plant, controller, and observer for controlling a mechanism with
 * full state feedback.
 *
 * For everything in this file, "inputs" and "outputs" are defined from the
 * perspective of the plant. This means U is an input and Y is an output
 * (because you give the plant U (powers) and it gives you back a Y (sensor
 * values). This is the opposite of what they mean from the perspective of the
 * controller (U is an output because that's what goes to the motors and Y is an
 * input because that's what comes back from the sensors).
 */
template <int States, int Inputs, int Outputs,
          typename PlantType = Plant<States, Inputs, Outputs>,
          typename ObserverType = LuenbergerObserver<States, Inputs, Outputs>>
class StateFeedbackLoop {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit StateFeedbackLoop(PlantType&& plant,
                             Controller<States, Inputs, Outputs>&& controller,
                             ObserverType&& observer);
  StateFeedbackLoop(StateFeedbackLoop&& other);

  virtual ~StateFeedbackLoop() = default;

  StateFeedbackLoop(const StateFeedbackLoop&) = delete;
  StateFeedbackLoop& operator=(const StateFeedbackLoop&) = delete;

  const Eigen::Matrix<double, States, 1>& Xhat() const;
  double Xhat(int i, int j) const;
  const Eigen::Matrix<double, States, 1>& R() const;
  double R(int i, int j) const;
  const Eigen::Matrix<double, States, 1>& NextR() const;
  double NextR(int i, int j) const;
  const Eigen::Matrix<double, Inputs, 1>& U() const;
  double U(int i, int j) const;
  const Eigen::Matrix<double, Inputs, 1>& Uuncapped() const;
  double Uuncapped(int i, int j) const;
  const Eigen::Matrix<double, Inputs, 1>& ffU() const;
  double ffU(int i, int j) const;

  Eigen::Matrix<double, States, 1>& MutableXhat();
  double& MutableXhat(int i, int j);
  Eigen::Matrix<double, States, 1>& MutableR();
  double& MutableR(int i, int j);
  Eigen::Matrix<double, States, 1>& MutableNextR();
  double& MutableNextR(int i, int j);
  Eigen::Matrix<double, Inputs, 1>& MutableU();
  double& MutableU(int i, int j);
  Eigen::Matrix<double, Inputs, 1>& MutableUuncapped();
  double& MutableUuncapped(int i, int j);

  const PlantType& GetPlant() const;
  PlantType* MutablePlant();
  const Controller<States, Inputs, Outputs>& GetController() const;
  const ObserverType& GetObserver() const;

  /**
   * Zeroes reference R, controller output U, plant output Y, and state estimate
   * Xhat.
   */
  void Reset();

  /**
   * If U is outside the hardware range, limit it before the plant tries to use
   * it.
   */
  virtual void CapU();

  /**
   * Returns difference between reference R and Xhat.
   */
  const Eigen::Matrix<double, States, 1> Error() const;

  /**
   * Sets new controller output, projects model forward, and runs observer
   * prediction.
   *
   * After calling this, the user should send the elements of U to the
   * actuators.
   *
   * @param disable True will zero controller output U.
   */
  void Update(bool disable,
              std::chrono::nanoseconds dt = std::chrono::milliseconds(5));

  /**
   * Corrects Xhat given the observation in Y.
   *
   * @param Y System output/measurements.
   */
  void Correct(const Eigen::Matrix<double, Outputs, 1>& Y);

  /**
   * Sets the current controller to be index. This can be used for gain
   * scheduling.
   */
  void SetIndex(int index);

  int GetIndex() const;

 protected:
  PlantType m_plant;
  Controller<States, Inputs, Outputs> m_controller;
  ObserverType m_observer;

  // These are accessible from non-templated subclasses.
  static constexpr int kNumStates = States;
  static constexpr int kNumOutputs = Outputs;
  static constexpr int kNumInputs = Inputs;

  // Portion of U which is based on the feedforwards.
  Eigen::Matrix<double, Inputs, 1> m_ffU;

 private:
  // Current reference (used by the feedback controller).
  Eigen::Matrix<double, States, 1> m_R;

  // Reference to go to in the next cycle (used by feedforward controller).
  Eigen::Matrix<double, States, 1> m_nextR;

  // Computed controller output after being capped.
  Eigen::Matrix<double, Inputs, 1> m_U;

  // Computed controller output before being capped.
  Eigen::Matrix<double, Inputs, 1> m_Uuncapped;

  // Returns the calculated controller power.
  virtual const Eigen::Matrix<double, Inputs, 1> ControllerOutput();

  // Calculates the feedforwards power.
  virtual const Eigen::Matrix<double, Inputs, 1> FeedForward();

  // Updates R after any CapU operations happen on U.
  void UpdateFFReference();
};

}  // namespace frc

#include "StateFeedback/StateFeedbackLoop.inc"
