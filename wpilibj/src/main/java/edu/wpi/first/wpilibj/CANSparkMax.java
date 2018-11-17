/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj;

import java.util.Optional;

// import edu.wpi.first.hal.FRCNetComm.tResourceType;
// import edu.wpi.first.hal.HAL;

/**
 * REV Robotics Spark MAX motor controller with CAN control.
 */
@SuppressWarnings("PMD.TooManyMethods")
public class CANSparkMax implements SpeedController {
  private static int kDeviceManufacturer = 1;
  private static int kDeviceType = 1;

  private final CAN m_can;
  private final int m_deviceId;

  public enum ClosedLoopMode {
    kPosition,
    kVelocity,
    kCurrent
  }

  public enum ClosedLoopSensor {
    kEncoder,
    kPotentiometer
  }

  public enum Leader {
    kCANSparkMax,
    kTalonSRX
  }

  /**
   * Construct a CANSparkMax with the given CAN ID.
   *
   * @param deviceId The CAN ID of the controller.
   */
  public CANSparkMax(int deviceId) {
    m_can = new CAN(deviceId, kDeviceManufacturer, kDeviceType);
    m_deviceId = deviceId;

    // HAL.report(tResourceType.kResourceType_CANSparkMax, deviceId);
    // setName("CANSparkMax", deviceId);
  }

  /**
   * Sets a speed as percent Vbus normalized between -1.0 and 1.0.
   *
   * <p>This automatically switches the controller to percent Vbus mode, which disables PID control.
   *
   * @param speed The speed to set [-1.0..1.0].
   */
  @Override
  public void set(double speed) {
    // FIXME
    byte[] buf = new byte[8];
    m_can.writePacket(buf, 10);
  }

  @Override
  public double get() {
    return 0.0;
  }

  /**
   * Enables and sets the rate at which the controller's output voltage should change per second.
   *
   * @param voltagePerSecond Amount output voltage should be allowed to change per second.
   */
  public void enableVoltageRampRate(double voltagePerSecond) {
  }

  /**
   * Disables voltage ramp rate.
   */
  public void disableVoltageRampRate() {}

  /**
   * Enables output current limiting and sets the maximum current the controller is allowed to
   * output.
   *
   * <p>The controller does this by reducing the output voltage (I = V/R). This can be used to avoid
   * pulling too much current in stall scenarios and tripping circuit breakers.
   *
   * @param maxCurrent Maximum allowed current in Amps.
   */
  public void enableCurrentLimit(double maxCurrent) {
  }

  /**
   * Disables output current limiting.
   */
  public void disableCurrentLimit() {
  }

  /**
   * Sets the controller gains of the internal PID controller.
   *
   * <p>This only affects the currently selected set of gains.
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   */
  @SuppressWarnings("ParameterName")
  public void setPID(double Kp, double Ki, double Kd) {
  }

  /**
   * Sets the proportional gain of the internal PID controller.
   *
   * <p>This only affects the currently selected set of gains.
   *
   * @param Kp Proportional gain.
   */
  @SuppressWarnings("ParameterName")
  public void setP(double Kp) {
  }

  /**
   * Sets the integral gain of the internal PID controller.
   *
   * <p>This only affects the currently selected set of gains.
   *
   * @param Ki Integral gain.
   */
  @SuppressWarnings("ParameterName")
  public void setI(double Ki) {
  }

  /**
   * Sets the derivative gain of the internal PID controller.
   *
   * <p>This only affects the currently selected set of gains.
   *
   * @param Kd Derivative gain.
   */
  @SuppressWarnings("ParameterName")
  public void setD(double Kd) {
  }

  /**
   * Selects the set of gains used by the internal PID controller.
   *
   * <p>Switching between different sets of gains is known as gain scheduling.
   *
   * @param index The index of the set of gains desired [0..2].
   */
  public void setPIDIndex(int index) {
  }

  /**
   * Sets the voltage feedforward used by the internal PID controller.
   *
   * <p>This feedforward uses u_ff = Kv * v + Ka * a + Kstatic where Kv is the velocity feedforward
   * coefficient, v is the desired velocity, Ka is the acceleration feedforward coefficient, a is
   * the desired acceleration, and Kstatic is a constant.
   *
   * <p>Kstatic can be used for overcoming static friction or compensating for gravity.
   *
   * @param Kv      Velocity feedforward coefficient.
   * @param Ka      Acceleration feedforward coefficient.
   * @param Kstatic Constant feedforward coefficient.
   */
  @SuppressWarnings("ParameterName")
  public void setFeedforward(double Kv, double Ka, double Kstatic) {
  }

  /**
   * Enables motion profiling and sets kinematic constraints of the motion profile on internal PID
   * controller reference.
   *
   * @param maxVelocity Maximum velocity of sensor in meters per second.
   */
  public void enableMotionProfiling(double maxVelocity) {
    enableMotionProfiling(maxVelocity, Double.POSITIVE_INFINITY);
  }

  /**
   * Enables motion profiling and sets kinematic constraints of the motion profile on internal PID
   * controller reference.
   *
   * @param maxVelocity Maximum velocity of sensor in meters per second.
   * @param maxAcceleration Maximum acceleration of sensor in meters per second.
   */
  public void enableMotionProfiling(double maxVelocity, double maxAcceleration) {
  }

  /**
   * Disables motion profiling.
   */
  public void disableMotionProfiling() {
  }

  /**
   * If a new control mode has been specified, it is automatically switch to.
   *
   * <p>This obeys previously set motion profiling constraints.
   *
   * @param mode      The closed loop controller type (position, velocity, current).
   * @param reference The reference for the feedback controller.
   */
  public void setReference(ClosedLoopMode mode, double reference) {
  }

  /**
   * Sets sensor used by internal PID controller.
   *
   * <p>The minimum and maximum sensor value are consider to be the same point in space and the
   * error is calculated to reflect the shortest route to the reference.
   *
   * @param sensor Type of sensor (encoder, potentiometer, etc.).
   * @param minValue Minimum sensor value in ticks or Volts depending on the sensor.
   * @param maxValue Maximum sensor value in ticks or Volts depending on the sensor.
   */
  public void setClosedLoopSensor(ClosedLoopSensor sensor, double minValue, double maxValue) {
  }

  /**
   * Sets sensor used by internal PID controller.
   *
   * <p>This function disables continuous error wrapping if it was enabled.
   *
   * @param sensor Type of sensor (encoder, potentiometer, etc.).
   */
  public void setClosedLoopSensor(ClosedLoopSensor sensor) {
  }

  /**
   * Sets distance per pulse of encoder.
   *
   * @param distancePerPulse Distance per pulse of encoder.
   */
  public void setEncoderDistancePerPulse(double distancePerPulse) {
  }

  /**
   * Sets distance per volt of potentiometer.
   *
   * @param distancePerVolt Distance per volt of potentiometer.
   */
  public void setPotentiometerDistancePerVolt(double distancePerVolt) {
  }

  /**
   * Enables hard limits based on digital input readings.
   *
   * <p>If the corresponding limit switch is pressed, the controller will stop applying voltage in
   * that direction. The default setting upon object construction is both limits being disabled
   * (both arguments are false).
   *
   * @param useForwardLimit Whether to use forward limit switch.
   * @param useReverseLimit Whether to use reverse limit switch.
   */
  public void enableHardLimits(boolean useForwardLimit, boolean useReverseLimit) {
  }

  /**
   * Enables and sets soft limits based on sensor readings.
   *
   * <p>If the corresponding sensor reading has been reached, the controller will stop applying
   * voltage in that direction. The default setting upon object construction is both limits being
   * disabled.
   *
   * @param minValue Minimum sensor value in distance or Volts depending on the sensor.
   * @param maxValue Maximum sensor value in distance or Volts depending on the sensor.
   */
  public void enableSoftLimits(double minValue, double maxValue) {
  }

  /**
   * Disables soft limits.
   */
  public void disableSoftLimits() {
  }

  /**
   * Returns true if encoder is connected.
   */
  public boolean isEncoderConnected() {
    return false;
  }

  /**
   * Returns encoder position.
   *
   * <p>The optional will only contain a value if the sensor is connected.
   */
  public Optional<Double> getEncoderPosition() {
    return Optional.ofNullable(null);
  }

  /**
   * Returns encoder velocity.
   *
   * <p>The optional will only contain a value if the sensor is connected.
   */
  public Optional<Double> getEncoderVelocity() {
    return Optional.ofNullable(null);
  }

  /**
   * Returns true if potentiometer is connected.
   */
  public boolean isPotentiometerConnected() {
    return false;
  }

  /**
   * Returns potentiometer distance (voltage times distance per volt).
   *
   * <p>The optional will only contain a value if the sensor is connected.
   */
  public Optional<Double> getPotentiometerDistance() {
    return Optional.ofNullable(null);
  }

  /**
   * Returns potentiometer voltage.
   *
   * <p>The optional will only contain a value if the sensor is connected.
   */
  public Optional<Double> getPotentiometerVoltage() {
    return Optional.ofNullable(null);
  }

  /**
   * Returns status of forward limit switch.
   *
   * <p>The optional will only contain a value if the sensor is connected.
   */
  public Optional<Boolean> getForwardLimitSwitch() {
    return Optional.ofNullable(null);
  }

  /**
   * Returns status of reverse limit switch.
   *
   * <p>The optional will only contain a value if the sensor is connected.
   */
  public Optional<Boolean> getReverseLimitSwitch() {
    return Optional.ofNullable(null);
  }

  /**
   * Instructs this motor controller to follow the given controller.
   *
   * <p>TODO: document what settings propagate from leader to followers. Could be output voltage,
   * feedback controller enable/settings, motion profile settings, etc.
   *
   * @param leader The controller to follow.
   */
  public void follow(CANSparkMax leader) {
    follow(Leader.kCANSparkMax, leader.m_deviceId);
  }

  /**
   * Instructs this motor controller to follow the controller with the given CAN ID.
   *
   * <p>TODO: document what settings propagate from leader to followers. Could be output voltage,
   * feedback controller enable/settings, motion profile settings, etc.
   *
   * @param leader   The type of CAN motor controller to follow.
   * @param deviceId The CAN ID of the leader.
   */
  public void follow(Leader leader, int deviceId) {
  }

  @Override
  public void setInverted(boolean isInverted) {
  }

  @Override
  public boolean getInverted() {
    return false;
  }

  @Override
  public void disable() {
  }

  @Override
  public void stopMotor() {
  }

  @Override
  public void pidWrite(double output) {
  }
}
