/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

template <int States, int Inputs, int Outputs>
KalmanFilter<States, Inputs, Outputs>::KalmanFilter() {
  Reset();
}

/**
 * @return The current filter estimate
 */
template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, 1>&
KalmanFilter<States, Inputs, Outputs>::Get() const {
  return m_x;
}

/**
 * Resets Kalman filter's state.
 */
template <int States, int Inputs, int Outputs>
void KalmanFilter<States, Inputs, Outputs>::Reset() {
  m_x = Eigen::Matrix<double, States, 1>::Zero();
}

/**
 * Predicts current system state with a control input. A control input of zero
 * is equivalent to no control input being applied.
 *
 * @param input Control input
 */
template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, 1>&
KalmanFilter<States, Inputs, Outputs>::Predict(
    const Eigen::Matrix<double, Inputs, 1>& input) {
  // Update the state: x(k|k-1) = A * x(k-1|k-1)
  m_x = m_A * m_x;

  // x(k|k-1) += B * u(k)
  m_x.noalias() += m_B * input;

  if (m_updateErrorCov) {
    /* Update error covariance matrices
     * P(k|k-1) = A * P(k|k) * A^T + Q
     */
    m_P = m_A * m_P * m_A.transpose() + m_Q;
  }

  return m_x;
}

/**
 * Corrects state estimate using a measurement of the system state.
 *
 * @param measurement Measurement of system state
 */
template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, 1>&
KalmanFilter<States, Inputs, Outputs>::Correct(
    const Eigen::Matrix<double, Outputs, 1>& measurement) {
  if (m_updateErrorCov) {
    // S(k) = H * P(k|k-1) * H^T + R
    m_S.noalias() = m_H * m_P * m_H.transpose() + m_R;

    // K(k) = P(k|k-1) * H^T * S(k)^-1
    m_K.noalias() = m_P * m_H.transpose() *
                    m_S.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                        .solve(decltype(m_S)::Identity());
  }

  // y(k) = z(k) - H * x(k|k-1)
  m_y.noalias() = measurement - m_H * m_x;

  // x(k|k) = x(k|k-1) + K(k) * y(k)
  m_x.noalias() += m_K * m_y;

  if (m_updateErrorCov) {
    // P(k|k) = (I - K(k) * H) * P(k|k-1)
    m_P.noalias() =
        (Eigen::Matrix<double, States, States>::Identity() - m_K * m_H) * m_P;
  }

  return m_x;
}

/**
 * Sets the transition matrix.
 *
 * @param A Transition matrix
 */
template <int States, int Inputs, int Outputs>
void KalmanFilter<States, Inputs, Outputs>::SetA(
    const Eigen::Matrix<double, States, States>& A) {
  m_A = A;
}

/**
 * Sets the control matrix.
 *
 * @param B Control matrix
 */
template <int States, int Inputs, int Outputs>
void KalmanFilter<States, Inputs, Outputs>::SetB(
    const Eigen::Matrix<double, States, Inputs>& B) {
  m_B = B;
}

/**
 * Sets the measurement matrix.
 *
 * @param H Measurement matrix
 */
template <int States, int Inputs, int Outputs>
void KalmanFilter<States, Inputs, Outputs>::SetH(
    const Eigen::Matrix<double, Outputs, States>& H) {
  m_H = H;
}

/**
 * Sets the process noise covariance matrix.
 *
 * @param Q Process noise covariance matrix
 */
template <int States, int Inputs, int Outputs>
void KalmanFilter<States, Inputs, Outputs>::SetQ(
    const Eigen::Matrix<double, States, States>& Q) {
  m_Q = Q;
}

/**
 * Sets the measurement noise covariance matrix.
 *
 * @param R Measurement noise covariance matrix
 */
template <int States, int Inputs, int Outputs>
void KalmanFilter<States, Inputs, Outputs>::SetR(
    const Eigen::Matrix<double, Outputs, Outputs>& R) {
  m_R = R;
}

/**
 * Sets the error covariance matrix.
 *
 * By default, this filter will iteratively compute an error covariance
 * internally and use it to compute the Kalman gain. Providing a precomputed one
 * here will permanently disable this.
 *
 * Since stable filters converge upon a steady state error covariance, it is
 * common practice to precompute it and skip updating the error covariance and
 * Kalman gain during runtime.
 *
 * Since the calculations performed by this function require the measurement
 * matrix, the user should set one before calling this function.
 *
 * @param P Error covariance matrix
 */
template <int States, int Inputs, int Outputs>
void KalmanFilter<States, Inputs, Outputs>::SetP(
    const Eigen::Matrix<double, States, States>& P) {
  m_P = P;

  // S(k) = H * P(k|k-1) * H^T + R
  m_S.noalias() = m_H * m_P * m_H.transpose() + m_R;

  // K(k) = P(k|k-1) * H^T * S(k)^-1
  m_K.noalias() = m_P * m_H.transpose() *
                  m_S.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                      .solve(decltype(m_S)::Identity());

  if (m_updateErrorCov) {
    m_updateErrorCov = false;
  }
}

/**
 * @return The filter's current error covariance
 */
template <int States, int Inputs, int Outputs>
const Eigen::Matrix<double, States, States>&
KalmanFilter<States, Inputs, Outputs>::GetP() const {
  return m_P;
}
