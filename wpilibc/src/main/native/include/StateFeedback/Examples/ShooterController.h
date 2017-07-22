/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>
#include <chrono>
#include <memory>

#include <Eigen/Core>

#include "StateFeedback/PeriodVariantKalmanFilter.h"
#include "StateFeedback/PeriodVariantPlant.h"
#include "StateFeedback/StateFeedbackLoop.h"

namespace frc {

class ShooterController {
 public:
  static constexpr std::chrono::nanoseconds kLoopPeriod =
      std::chrono::milliseconds(5);

  ShooterController();

  ShooterController(const ShooterController&) = delete;
  ShooterController& operator=(const ShooterController&) = delete;

  // Sets the velocity goal in radians/sec
  void SetGoal(double angularVelocityGoal);

  // Sets the current encoder position in radians
  void SetPosition(double currentPosition);

  // Returns the control loop calculated voltage.
  double Voltage() const;

  // Returns the instantaneous velocity.
  double Velocity() const { return m_loop->Xhat(2, 0); }
  double VoltageError() const { return m_loop->Xhat(3, 0); }
  bool Ready() const { return m_ready; }

  double DtVelocity() const { return m_dtVelocity; }

  double Error() const { return m_error; }

  // Executes the control loop for a cycle.
  void Update(bool disable, std::chrono::nanoseconds dt);

  // Resets the kalman filter and any other internal state.
  void Reset();

 private:
  // The current sensor measurement.
  Eigen::Matrix<double, 1, 1> m_Y;
  // The control loop.
  std::unique_ptr<StateFeedbackLoop<3, 1, 1, PeriodVariantPlant<3, 1, 1>,
                                    PeriodVariantKalmanFilter<3, 1, 1>>>
      m_loop;

  // History array for calculating a filtered angular velocity.
  static constexpr int kHistoryLength = 5;
  std::array<double, kHistoryLength> m_history;
  std::ptrdiff_t m_historyPosition = 0;

  double m_error = 0.0;
  double m_dtPosition = 0.0;
  double m_dtVelocity = 0.0;
  double m_fixedDtVelocity = 0.0;
  double m_lastPosition = 0.0;
  double m_averageAngularVelocity = 0.0;
  double m_minVelocity = 0.0;
  double m_positionError = 0.0;

  Eigen::Matrix<double, 3, 1> m_XhatCurrent;

  bool m_ready = false;
  bool m_needsReset = false;
  bool m_reset = false;

  bool m_lastReady = false;
};

}  // namespace frc
