/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <eigen3/Eigen/Dense>

/**
 * This class implements a Kalman filter.
 *
 * <b><i>a priori</i> prediction equations</b><br>
 * <i>x<sub>k|k-1</sub> = A * x<sub>k-1|k-1</sub> + B * u<sub>k</sub></i>
 *
 * Where<br>
 *  <i>x<sub>k|k-1</sub></i> is the <i>a priori</i> state estimate<br>
 *  <i>A</i> is the transition matrix<br>
 *  <i>x<sub>k-1|k-1</sub></i> is the <i>a posteriori</i> state estimate from
 *    the previous iteration<br>
 *  <i>B</i> is the control matrix<br>
 *  <i>u<sub>k</sub></i> is the current control input
 *
 * <i>P<sub>k|k-1</sub> = A * P<sub>k|k</sub> * A<sup>T</sup> + Q</i>
 *
 * Where<br>
 *   <i>P<sub>k|k-1</sub></i> is the <i>a priori</i> error covariance matrix<br>
 *   <i>P<sub>k|k</sub></i> is the <i>a posteriori</i> error covariance matrix
 *     from the previous iteration<br>
 *   <i>Q</i> is the process noise covariance matrix
 *
 * <b><i>a posteriori</i> update equations</b><br>
 * <i>S<sub>k</sub> = H * P<sub>k|k-1</sub> * H<sup>T</sup> + R</i><br>
 *
 * Where<br>
 *   <i>S<sub>k</sub></i> is the innovation covariance<br>
 *   <i>H</i> is the measurement matrix<br>
 *   <i>R</i> is the measurement noise covariance matrix
 *
 * <i>K<sub>k</sub> = P<sub>k|k-1</sub> * H<sup>T</sup> * S<sup>-1</sup></i>
 *
 * Where<br>
 *   <i>K<sub>k</sub></i> is the Kalman gain matrix
 *
 * <i>y<sub>k</sub> = z<sub>k</sub> - H * x<sub>k|k-1</sub></i>
 *
 * Where<br>
 *   <i>y<sub>k</sub></i> is the innovation<br>
 *   <i>z<sub>k</sub></i> is the current measurement
 *
 * <i>x<sub>k|k</sub> =
 *   x<sub>k|k-1</sub> + K<sub>k</sub> * y<sub>k</sub></i><br>
 * <i>P<sub>k|k</sub> = (I - K<sub>k</sub> * H) * P<sub>k|k-1</sub></i>
 *
 * Read https://en.wikipedia.org/wiki/Kalman_filter for more information.
 *
 * @tparam States Dimensionality of the system state
 * @tparam Inputs Dimensionality of the system input
 * @tparam Outputs Dimensionality of the system output
 */
template <int States, int Inputs, int Outputs>
class KalmanFilter {
 public:
  KalmanFilter();
  KalmanFilter(const KalmanFilter&) = delete;
  KalmanFilter(KalmanFilter&&) = default;

  KalmanFilter& operator=(const KalmanFilter&) = delete;
  KalmanFilter& operator=(KalmanFilter&&) = default;

  const Eigen::Matrix<double, States, 1>& Get() const;
  void Reset();

  const Eigen::Matrix<double, States, 1>& Predict(
      const Eigen::Matrix<double, Inputs, 1>& input);

  const Eigen::Matrix<double, States, 1>& Correct(
      const Eigen::Matrix<double, Outputs, 1>& measurement);

  void SetA(const Eigen::Matrix<double, States, States>& A);
  void SetB(const Eigen::Matrix<double, States, Inputs>& B);
  void SetH(const Eigen::Matrix<double, Outputs, States>& H);
  void SetQ(const Eigen::Matrix<double, States, States>& Q);
  void SetR(const Eigen::Matrix<double, Outputs, Outputs>& R);

  void SetP(const Eigen::Matrix<double, States, States>& P);
  const Eigen::Matrix<double, States, States>& GetP() const;

 private:
  // State estimate (x(k))
  Eigen::Matrix<double, States, 1> m_x;

  // State transition matrix (A)
  Eigen::Matrix<double, States, States> m_A{
      Eigen::Matrix<double, States, States>::Identity()};

  // Control matrix (B) (not used if there is no control)
  Eigen::Matrix<double, States, Inputs> m_B{
      Eigen::Matrix<double, States, Inputs>::Zero()};

  // Measurement matrix (H)
  Eigen::Matrix<double, Outputs, States> m_H{
      Eigen::Matrix<double, Outputs, States>::Zero()};

  // Error estimate covariance matrix (P(k))
  Eigen::Matrix<double, States, States> m_P{
      Eigen::Matrix<double, States, States>::Zero()};

  // Process noise covariance matrix (Q)
  Eigen::Matrix<double, States, States> m_Q{
      Eigen::Matrix<double, States, States>::Zero()};

  // Measurement noise covariance matrix (R)
  Eigen::Matrix<double, Outputs, Outputs> m_R{
      Eigen::Matrix<double, Outputs, Outputs>::Zero()};

  // Innovation covariance (S(k))
  Eigen::Matrix<double, Outputs, Outputs> m_S;

  // Kalman gain matrix (K(k))
  Eigen::Matrix<double, States, Outputs> m_K{
      Eigen::Matrix<double, States, Outputs>::Zero()};

  // Innovation (y(k))
  Eigen::Matrix<double, Outputs, 1> m_y;

  bool m_updateErrorCov = true;
};

// Static methods to create example filters
namespace Kalman {
KalmanFilter<2, 1, 1> GyroAngle(double period);
}  // namespace Kalman

#include "KalmanFilter.inc"
