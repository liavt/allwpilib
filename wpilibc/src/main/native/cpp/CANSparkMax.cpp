/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/CANSparkMax.h"

#include <hal/HAL.h>

using namespace frc;

CANSparkMax::CANSparkMax(int deviceId)
    : m_can(deviceId, kDeviceManufacturer, kDeviceType), m_deviceId(deviceId) {
  // HAL_Report(HALUsageReporting::kResourceType_CANSparkMax, deviceId);
  // SetName("CANSparkMax", deviceId);
}

void CANSparkMax::Set(double speed) {}

double CANSparkMax::Get() const { return 0.0; }

void CANSparkMax::EnableVoltageRampRate(double voltagePerSecond) {}

void CANSparkMax::DisableVoltageRampRate() {}

void CANSparkMax::EnableCurrentLimit(double maxCurrent) {}

void CANSparkMax::DisableCurrentLimit() {}

void CANSparkMax::SetPID(double Kp, double Ki, double Kd) {}

void CANSparkMax::SetP(double Kp) {}

void CANSparkMax::SetI(double Ki) {}

void CANSparkMax::SetD(double Kd) {}

void CANSparkMax::SetPIDIndex(int index) {}

void CANSparkMax::SetFeedforward(double Kv, double Ka, double Kstatic) {}

void CANSparkMax::EnableMotionProfiling(double maxVelocity,
                                        double maxAcceleration) {}

void CANSparkMax::DisableMotionProfiling() {}

void CANSparkMax::SetReference(ClosedLoopMode mode, double reference) {}

void CANSparkMax::SetClosedLoopSensor(ClosedLoopSensor sensor, double minValue,
                                      double maxValue) {}

void CANSparkMax::SetClosedLoopSensor(ClosedLoopSensor sensor) {}

void CANSparkMax::SetEncoderDistancePerPulse(double distancePerPulse) {}

void CANSparkMax::SetPotentiometerDistancePerVolt(double distancePerVolt) {}

void CANSparkMax::EnableHardLimits(bool useForwardLimit, bool useReverseLimit) {
}

void CANSparkMax::EnableSoftLimits(double minValue, double maxValue) {}

void CANSparkMax::DisableSoftLimits() {}

bool CANSparkMax::IsEncoderConnected() { return false; }

std::optional<double> CANSparkMax::GetEncoderPosition() { return {}; }

std::optional<double> CANSparkMax::GetEncoderVelocity() { return {}; }

bool CANSparkMax::IsPotentiometerConnected() { return false; }

std::optional<double> CANSparkMax::GetPotentiometerDistance() { return {}; }

std::optional<double> CANSparkMax::GetPotentiometerVoltage() { return {}; }

std::optional<bool> CANSparkMax::GetForwardLimitSwitch() { return {}; }

std::optional<bool> CANSparkMax::GetReverseLimitSwitch() { return {}; }

void CANSparkMax::Follow(const CANSparkMax& leader) {
  Follow(Leader::kCANSparkMax, leader.m_deviceId);
}

void CANSparkMax::Follow(Leader leader, int deviceId) {}

void CANSparkMax::SetInverted(bool isInverted) {}

bool CANSparkMax::GetInverted() const { return false; }

void CANSparkMax::Disable() {}

void CANSparkMax::StopMotor() {}

void CANSparkMax::PIDWrite(double output) {}
