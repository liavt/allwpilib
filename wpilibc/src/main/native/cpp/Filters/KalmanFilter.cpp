/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Filters/KalmanFilter.h"

/**
 * Creates a Kalman filter which estimates orientation. It uses a gyroscope and
 * compensates for drift by measuring the rate bias of the gyroscope and
 * subtracting it from the current rate.
 *
 * The angle used to determine the bias could be obtained using accelerometer
 * axes for a balancing robot or from a magnetometer for yaw rotation.
 *
 * This filter has two states, one output, and one input. The states are the
 * current angle and the rate bias, the output is the angle, and the input is
 * the angular rate.
 *
 * Users should set the process noise covariance matrix, the measurement noise
 * covariance matrix before using the filter. The error covariance matrix, if
 * applicable, should be set last.
 *
 * TODO: Example usage and tests
 *
 * See
 * http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
 * for an explanation of the matrices used here.
 *
 * @param period The period in seconds between samples taken by the user
 */
KalmanFilter<2, 1, 1> Kalman::GyroAngle(double period) {
  KalmanFilter<2, 1, 1> filter;

  Eigen::Matrix<double, 2, 2> a;
  a << 1, -period, 0, 1;
  filter.SetA(a);

  Eigen::Matrix<double, 2, 1> b;
  b << period, 0;
  filter.SetB(b);

  Eigen::Matrix<double, 1, 2> h;
  h << 1, 0;
  filter.SetH(h);

  return filter;
}
