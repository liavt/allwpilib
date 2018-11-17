/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <limits>

#include <experimental/optional>

#include "frc/CAN.h"
#include "frc/SpeedController.h"

namespace std {
using experimental::optional;
}  // namespace std

namespace frc {

/**
 * REV Robotics Spark MAX motor controller with CAN control.
 */
class CANSparkMax : public SpeedController {
 public:
  enum class ClosedLoopMode { kPosition, kVelocity, kCurrent };
  enum class ClosedLoopSensor { kEncoder, kPotentiometer };
  enum class Leader { kCANSparkMax, kTalonSRX };

  /**
   * Construct a CANSparkMax with the given CAN ID.
   *
   * @param deviceId The CAN ID of the controller.
   */
  explicit CANSparkMax(int deviceId);

  virtual ~CANSparkMax() = default;

  /**
   * Sets a speed as percent Vbus normalized between -1.0 and 1.0.
   *
   * This automatically switches the controller to percent Vbus mode, which
   * disables PID control.
   *
   * @param speed The speed to set [-1.0..1.0].
   */
  void Set(double speed) override;

  double Get() const override;

  /**
   * Enables and sets the rate at which the controller's output voltage should
   * change per second.
   *
   * @param voltagePerSecond Amount output voltage should be allowed to change
   *                         per second.
   */
  void EnableVoltageRampRate(double voltagePerSecond);

  /**
   * Disables voltage ramp rate.
   */
  void DisableVoltageRampRate();

  /**
   * Enables output current limiting and sets the maximum current the controller
   * is allowed to output.
   *
   * The controller does this by reducing the output voltage (I = V/R). This can
   * be used to avoid pulling too much current in stall scenarios and tripping
   * circuit breakers.
   *
   * @param maxCurrent Maximum allowed current in Amps.
   */
  void EnableCurrentLimit(double maxCurrent);

  /**
   * Disables output current limiting.
   */
  void DisableCurrentLimit();

  /**
   * Sets the controller gains of the internal PID controller.
   *
   * This only affects the currently selected set of gains.
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   */
  void SetPID(double Kp, double Ki, double Kd);

  /**
   * Sets the proportional gain of the internal PID controller.
   *
   * This only affects the currently selected set of gains.
   *
   * @param Kp Proportional gain.
   */
  void SetP(double Kp);

  /**
   * Sets the integral gain of the internal PID controller.
   *
   * This only affects the currently selected set of gains.
   *
   * @param Ki Integral gain.
   */
  void SetI(double Ki);

  /**
   * Sets the derivative gain of the internal PID controller.
   *
   * This only affects the currently selected set of gains.
   *
   * @param Kd Derivative gain.
   */
  void SetD(double Kd);

  /**
   * Selects the set of gains used by the internal PID controller.
   *
   * Switching between different sets of gains is known as gain scheduling.
   *
   * @param index The index of the set of gains desired [0..2].
   */
  void SetPIDIndex(int index);

  /**
   * Sets the voltage feedforward used by the internal PID controller.
   *
   * This feedforward uses u_ff = Kv * v + Ka * a + Kstatic where Kv is the
   * velocity feedforward coefficient, v is the desired velocity, Ka is the
   * acceleration feedforward coefficient, a is the desired acceleration, and
   * Kstatic is a constant.
   *
   * Kstatic can be used for overcoming static friction or compensating for
   * gravity.
   *
   * @param Kv      Velocity feedforward coefficient.
   * @param Ka      Acceleration feedforward coefficient.
   * @param Kstatic Constant feedforward coefficient.
   */
  void SetFeedforward(double Kv, double Ka, double Kstatic);

  /**
   * Enables motion profiling and sets kinematic constraints of the motion
   * profile on internal PID controller reference.
   *
   * @param maxVelocity Maximum velocity of sensor in meters per second.
   * @param maxAcceleration Maximum acceleration of sensor in meters per second.
   */
  void EnableMotionProfiling(
      double maxVelocity,
      double maxAcceleration = std::numeric_limits<double>::infinity());

  /**
   * Disables motion profiling.
   */
  void DisableMotionProfiling();

  /**
   * If a new control mode has been specified, it is automatically switch to.
   *
   * This obeys previously set motion profiling constraints.
   *
   * @param mode      The closed loop controller type (position, velocity,
   *                  current).
   * @param reference The reference for the feedback controller.
   */
  void SetReference(ClosedLoopMode mode, double reference);

  /**
   * Sets sensor used by internal PID controller.
   *
   * The minimum and maximum sensor value are consider to be the same point in
   * space and the error is calculated to reflect the shortest route to the
   * reference.
   *
   * @param sensor Type of sensor (encoder, potentiometer, etc.).
   * @param minValue Minimum sensor value in ticks or Volts depending on the
   *                 sensor.
   * @param maxValue Maximum sensor value in ticks or Volts depending on the
   *                 sensor.
   */
  void SetClosedLoopSensor(ClosedLoopSensor sensor, double minValue,
                           double maxValue);

  /**
   * Sets sensor used by internal PID controller.
   *
   * This function disables continuous error wrapping if it was enabled.
   *
   * @param sensor Type of sensor (encoder, potentiometer, etc.).
   */
  void SetClosedLoopSensor(ClosedLoopSensor sensor);

  /**
   * Sets distance per pulse of encoder.
   *
   * @param distancePerPulse Distance per pulse of encoder.
   */
  void SetEncoderDistancePerPulse(double distancePerPulse);

  /**
   * Sets distance per volt of potentiometer.
   *
   * @param distancePerVolt Distance per volt of potentiometer.
   */
  void SetPotentiometerDistancePerVolt(double distancePerVolt);

  /**
   * Enables hard limits based on digital input readings.
   *
   * If the corresponding limit switch is pressed, the controller will stop
   * applying voltage in that direction. The default setting upon object
   * construction is both limits being disabled (both arguments are false).
   *
   * @param useForwardLimit Whether to use forward limit switch.
   * @param useReverseLimit Whether to use reverse limit switch.
   */
  void EnableHardLimits(bool useForwardLimit, bool useReverseLimit);

  /**
   * Enables and sets soft limits based on sensor readings.
   *
   * If the corresponding sensor reading has been reached, the controller will
   * stop applying voltage in that direction. The default setting upon object
   * construction is both limits being disabled.
   *
   * @param minValue Minimum sensor value in distance or Volts depending on the
   *                 sensor.
   * @param maxValue Maximum sensor value in distance or Volts depending on the
   *                 sensor.
   */
  void EnableSoftLimits(double minValue, double maxValue);

  /**
   * Disables soft limits.
   */
  void DisableSoftLimits();

  /**
   * Returns true if encoder is connected.
   */
  bool IsEncoderConnected();

  /**
   * Returns encoder position.
   *
   * The optional will only contain a value if the sensor is connected.
   */
  std::optional<double> GetEncoderPosition();

  /**
   * Returns encoder velocity.
   *
   * The optional will only contain a value if the sensor is connected.
   */
  std::optional<double> GetEncoderVelocity();

  /**
   * Returns true if potentiometer is connected.
   */
  bool IsPotentiometerConnected();

  /**
   * Returns potentiometer distance (voltage times distance per volt).
   *
   * The optional will only contain a value if the sensor is connected.
   */
  std::optional<double> GetPotentiometerDistance();

  /**
   * Returns potentiometer voltage.
   *
   * The optional will only contain a value if the sensor is connected.
   */
  std::optional<double> GetPotentiometerVoltage();

  /**
   * Returns status of forward limit switch.
   *
   * The optional will only contain a value if the sensor is connected.
   */
  std::optional<bool> GetForwardLimitSwitch();

  /**
   * Returns status of reverse limit switch.
   *
   * The optional will only contain a value if the sensor is connected.
   */
  std::optional<bool> GetReverseLimitSwitch();

  /**
   * Instructs this motor controller to follow the given controller.
   *
   * TODO: document what settings propagate from leader to followers. Could be
   * output voltage, feedback controller enable/settings, motion profile
   * settings, etc.
   *
   * @param leader The controller to follow.
   */
  void Follow(const CANSparkMax& leader);

  /**
   * Instructs this motor controller to follow the controller with the given CAN
   * ID.
   *
   * TODO: document what settings propagate from leader to followers. Could be
   * output voltage, feedback controller enable/settings, motion profile
   * settings, etc.
   *
   * @param leader   The type of CAN motor controller to follow.
   * @param deviceId The CAN ID of the leader.
   */
  void Follow(Leader leader, int deviceId);

  void SetInverted(bool isInverted) override;
  bool GetInverted() const override;
  void Disable() override;
  void StopMotor() override;

  void PIDWrite(double output) override;

 private:
  static constexpr int kDeviceManufacturer = 1;
  static constexpr int kDeviceType = 1;

  CAN m_can;
  const int m_deviceId;
};

}  // namespace frc
