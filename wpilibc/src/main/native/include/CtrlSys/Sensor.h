/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "PIDSource.h"

namespace frc {

/**
 * SourceNode adapter for PIDSource subclasses.
 *
 * Wraps a PIDSource object in the SourceNode interface by returning the output
 * of PIDGet() from GetOutput().
 */
class Sensor {
 public:
  Sensor(PIDSource& source);  // NOLINT
  virtual ~Sensor() = default;

  double GetOutput();

 private:
  PIDSource& m_source;
};

}  // namespace frc
