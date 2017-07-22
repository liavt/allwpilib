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

namespace frc {

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
StateFeedbackLoop<States, Inputs, Outputs, PlantType, ObserverType>::
    StateFeedbackLoop(PlantType&& plant,
                      Controller<States, Inputs, Outputs>&& controller,
                      ObserverType&& observer)
    : m_plant(std::move(plant)),
      m_controller(std::move(controller)),
      m_observer(std::move(observer)) {
  Reset();
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                  ObserverType>::StateFeedbackLoop(StateFeedbackLoop&& other)
    : m_plant(std::move(other.m_plant)),
      m_controller(std::move(other.m_controller)),
      m_observer(std::move(other.m_observer)) {
  m_R.swap(other.m_R);
  m_nextR.swap(other.m_nextR);
  m_U.swap(other.m_U);
  m_Uuncapped.swap(other.m_Uuncapped);
  m_ffU.swap(other.m_ffU);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
const Eigen::Matrix<double, States, 1>& StateFeedbackLoop<
    States, Inputs, Outputs, PlantType, ObserverType>::Xhat() const {
  return GetObserver().Xhat();
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
double StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                         ObserverType>::Xhat(int i, int j) const {
  return Xhat()(i, j);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
const Eigen::Matrix<double, States, 1>&
StateFeedbackLoop<States, Inputs, Outputs, PlantType, ObserverType>::R() const {
  return m_R;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
double StateFeedbackLoop<States, Inputs, Outputs, PlantType, ObserverType>::R(
    int i, int j) const {
  return R()(i, j);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
const Eigen::Matrix<double, States, 1>& StateFeedbackLoop<
    States, Inputs, Outputs, PlantType, ObserverType>::NextR() const {
  return m_nextR;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
double StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                         ObserverType>::NextR(int i, int j) const {
  return NextR()(i, j);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
const Eigen::Matrix<double, Inputs, 1>&
StateFeedbackLoop<States, Inputs, Outputs, PlantType, ObserverType>::U() const {
  return m_U;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
double StateFeedbackLoop<States, Inputs, Outputs, PlantType, ObserverType>::U(
    int i, int j) const {
  return U()(i, j);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
const Eigen::Matrix<double, Inputs, 1>& StateFeedbackLoop<
    States, Inputs, Outputs, PlantType, ObserverType>::Uuncapped() const {
  return m_Uuncapped;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
double StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                         ObserverType>::Uuncapped(int i, int j) const {
  return Uuncapped()(i, j);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
const Eigen::Matrix<double, Inputs, 1>& StateFeedbackLoop<
    States, Inputs, Outputs, PlantType, ObserverType>::ffU() const {
  return m_ffU;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
double StateFeedbackLoop<States, Inputs, Outputs, PlantType, ObserverType>::ffU(
    int i, int j) const {
  return ffU()(i, j);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
Eigen::Matrix<double, States, 1>& StateFeedbackLoop<
    States, Inputs, Outputs, PlantType, ObserverType>::MutableXhat() {
  return m_observer.MutableXhat();
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
double& StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                          ObserverType>::MutableXhat(int i, int j) {
  return MutableXhat()(i, j);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
Eigen::Matrix<double, States, 1>& StateFeedbackLoop<
    States, Inputs, Outputs, PlantType, ObserverType>::MutableR() {
  return m_R;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
double& StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                          ObserverType>::MutableR(int i, int j) {
  return MutableR()(i, j);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
Eigen::Matrix<double, States, 1>& StateFeedbackLoop<
    States, Inputs, Outputs, PlantType, ObserverType>::MutableNextR() {
  return m_nextR;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
double& StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                          ObserverType>::MutableNextR(int i, int j) {
  return MutableNextR()(i, j);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
Eigen::Matrix<double, Inputs, 1>& StateFeedbackLoop<
    States, Inputs, Outputs, PlantType, ObserverType>::MutableU() {
  return m_U;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
double& StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                          ObserverType>::MutableU(int i, int j) {
  return MutableU()(i, j);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
Eigen::Matrix<double, Inputs, 1>& StateFeedbackLoop<
    States, Inputs, Outputs, PlantType, ObserverType>::MutableUuncapped() {
  return m_Uuncapped;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
double& StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                          ObserverType>::MutableUuncapped(int i, int j) {
  return MutableUuncapped()(i, j);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
const PlantType& StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                                   ObserverType>::GetPlant() const {
  return m_plant;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
PlantType* StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                             ObserverType>::MutablePlant() {
  return &m_plant;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
const Controller<States, Inputs, Outputs>& StateFeedbackLoop<
    States, Inputs, Outputs, PlantType, ObserverType>::GetController() const {
  return m_controller;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
const ObserverType& StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                                      ObserverType>::GetObserver() const {
  return m_observer;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
void StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                       ObserverType>::Reset() {
  m_R.setZero();
  m_nextR.setZero();
  m_U.setZero();
  m_Uuncapped.setZero();
  m_ffU.setZero();

  m_plant.Reset();
  m_controller.Reset();
  m_observer.Reset(this);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
void StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                       ObserverType>::CapU() {
  for (int i = 0; i < kNumInputs; ++i) {
    if (U(i, 0) > GetPlant().Umax(i, 0)) {
      m_U(i, 0) = GetPlant().Umax(i, 0);
    } else if (U(i, 0) < GetPlant().Umin(i, 0)) {
      m_U(i, 0) = GetPlant().Umin(i, 0);
    }
  }
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
const Eigen::Matrix<double, States, 1> StateFeedbackLoop<
    States, Inputs, Outputs, PlantType, ObserverType>::Error() const {
  return R() - Xhat();
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
void StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                       ObserverType>::Update(bool disable,
                                             std::chrono::nanoseconds dt) {
  if (disable) {
    m_U.setZero();
    m_Uuncapped.setZero();
    m_ffU.setZero();
  } else {
    m_U = m_Uuncapped = ControllerOutput();
    CapU();
  }

  m_observer.Predict(this, m_U, dt);

  UpdateFFReference();
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
void StateFeedbackLoop<States, Inputs, Outputs, PlantType, ObserverType>::
    Correct(const Eigen::Matrix<double, Outputs, 1>& Y) {
  m_observer.Correct(*this, U(), Y);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
void StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                       ObserverType>::SetIndex(int index) {
  m_plant.SetIndex(index);
  m_controller.SetIndex(index);
  m_observer.SetIndex(index);
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
int StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                      ObserverType>::GetIndex() const {
  return m_plant.GetIndex();
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
const Eigen::Matrix<double, Inputs, 1> StateFeedbackLoop<
    States, Inputs, Outputs, PlantType, ObserverType>::ControllerOutput() {
  // TODO(austin): Should this live in StateSpaceController?
  m_ffU = FeedForward();
  return m_controller.K() * Error() + m_ffU;
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
const Eigen::Matrix<double, Inputs, 1> StateFeedbackLoop<
    States, Inputs, Outputs, PlantType, ObserverType>::FeedForward() {
  // TODO(austin): Should this live in StateSpaceController?
  return m_controller.Kff() * (NextR() - GetPlant().A() * R());
}

template <int States, int Inputs, int Outputs, typename PlantType,
          typename ObserverType>
void StateFeedbackLoop<States, Inputs, Outputs, PlantType,
                       ObserverType>::UpdateFFReference() {
  m_ffU -= Uuncapped() - U();
  if (!m_controller.Kff().isZero(0)) {
    m_R = GetPlant().A() * R() + GetPlant().B() * m_ffU;
  }
}

}  // namespace frc
